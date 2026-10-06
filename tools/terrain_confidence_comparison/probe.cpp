#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "map/terrain_observation_estimator.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"
#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"
#include "lunar_incremental_navigation_core/types/terrain_fit_method.hpp"
#include "capabilities.hpp"

using namespace lunar::incremental_navigation;
using namespace lunar::incremental_navigation::map;
namespace {
double Milliseconds(SteadyClock::time_point start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}
std::string JsonString(const std::string& value) {
  std::string out = "\"";
  for (char c : value) {
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n') out += "\\n";
    else out += c;
  }
  return out + '"';
}
void WriteFloats(const std::string& path, const std::vector<float>& values) {
  std::ofstream stream(path, std::ios::binary);
  stream.write(reinterpret_cast<const char*>(values.data()),
               static_cast<std::streamsize>(values.size() * sizeof(float)));
  if (!stream) throw std::runtime_error("cannot write " + path);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 7 || argc > 10) {
      std::cerr << "usage: probe INPUT_F32 OUTPUT_PREFIX N RES METHOD fit|integrated "
                   "[CONF_WEIGHT=2] [MIN_CONF=.2] [MAX_RESIDUAL=.03]\n";
      return 2;
    }
    const int n = std::stoi(argv[3]);
    const double resolution = std::stod(argv[4]);
    const auto method = ParseTerrainFitMethod(argv[5]);
    const std::string mode = argv[6], prefix = argv[2];
    const double confidence_weight = argc > 7 ? std::stod(argv[7]) : 2.;
    const double minimum_confidence = argc > 8 ? std::stod(argv[8]) : .2;
    const double maximum_residual = argc > 9 ? std::stod(argv[9]) : .03;
    if (n < 3 || n > 4096 || resolution <= 0. ||
        (mode != "fit" && mode != "integrated")) throw std::invalid_argument("invalid geometry/mode");
    if (!std::isfinite(confidence_weight) || confidence_weight < 0. ||
        !std::isfinite(minimum_confidence) || minimum_confidence < 0. || minimum_confidence > 1. ||
        !std::isfinite(maximum_residual) || maximum_residual <= 0.)
      throw std::invalid_argument("invalid comparison parameters");
    if (mode == "integrated" &&
        (std::abs(minimum_confidence-.2) > 1e-12 || std::abs(maximum_residual-.03) > 1e-12))
      throw std::invalid_argument("integrated production profile exposes default fit thresholds only; use fit mode for threshold sensitivity");
    GridGeometry geometry{.frame_id = "map", .width = static_cast<unsigned>(n),
                          .height = static_cast<unsigned>(n), .resolution_m = resolution};
    std::vector<float> heights(static_cast<std::size_t>(n) * n);
    std::ifstream input(argv[1], std::ios::binary);
    input.read(reinterpret_cast<char*>(heights.data()),
               static_cast<std::streamsize>(heights.size() * sizeof(float)));
    if (!input || input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("wrong input size");
    PersistentElevationMap elevation;
    (void)elevation.Apply({.geometry = geometry, .elevation_m = heights,
                          .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
    auto raw = elevation.Snapshot();
    TerrainObservationEstimator observations(*raw, {.maximum_plane_residual_m = maximum_residual,
                                                    .minimum_confidence = minimum_confidence,
                                                    .fit_method = method});
    std::vector<float> ground(heights.size() * 6);
    const auto fitting_start = SteadyClock::now();
    for (int y = 0; y < n; ++y) for (int x = 0; x < n; ++x) {
      const auto& estimate = observations.Estimate({x, y});
      const std::size_t offset = (static_cast<std::size_t>(y) * n + x) * 6;
      ground[offset] = NAN; ground[offset + 1] = 0.;
      ground[offset + 2] = NAN; ground[offset + 3] = NAN;
      ground[offset + 4] = NAN; ground[offset + 5] = 0.;
      if (estimate) {
        ground[offset] = estimate->surface_height_m;
        ground[offset + 1] = estimate->confidence;
        ground[offset + 2] = estimate->gradient.x;
        ground[offset + 3] = estimate->gradient.y;
        ground[offset + 4] = estimate->predictive_variance_m2;
        ground[offset + 5] = estimate->used_fallback ? 1.f : 0.f;
      }
    }
    const double fit_ms = Milliseconds(fitting_start);
    WriteFloats(prefix + "-ground.bin", ground);
    std::ofstream summary(prefix + "-summary.json");
    summary << std::setprecision(12) << "{\"method\":" << JsonString(std::string(TerrainFitMethodName(method)))
            << ",\"mode\":" << JsonString(mode) << ",\"n\":" << n
            << ",\"resolution_m\":" << resolution << ",\"confidence_weight\":" << confidence_weight
            << ",\"minimum_confidence\":" << minimum_confidence
            << ",\"maximum_residual_m\":" << maximum_residual << ",\"fit_ms\":" << fit_ms
            << ",\"evaluated_cells\":" << observations.evaluated_cells()
            << ",\"allocated_bytes\":" << observations.allocated_bytes() << ",\"platforms\":{";
    bool first = true;
    if (mode == "integrated") for (const bool foot : {true, false}) {
      const auto wheel = MakeComparisonWheel();
      const auto legged = MakeComparisonLegged();
      PlatformCapability capability = foot ? PlatformCapability(legged) : PlatformCapability(wheel);
      TraversabilityProfile profile;
      profile.planar_envelope_xy_m = foot ? std::vector<Point2>{{-.34, -.165}, {.34, -.165}, {.34, .165}, {-.34, .165}}
                                            : wheel.footprint_xy_m;
      profile.terrain_background_half_width_m = foot ? .2 : .4;
      profile.preferred_clearance_m = .4; profile.clearance_weight = 2.;
      profile.slope_weight = 1.; profile.relief_weight = 1.;
      profile.confidence_weight = confidence_weight; profile.terrain_fit_method = method;
      auto start_time = SteadyClock::now();
      auto fine = FineTraversabilityBuilder().Derive(raw, capability, profile);
      const double map_ms = Milliseconds(start_time);
      const std::string name = foot ? "foot" : "wheel";
      std::vector<float> states(heights.size() * 2);
      for (int y = 0; y < n; ++y) for (int x = 0; x < n; ++x) {
        const GridIndex index{x, y}; const auto state = fine->State(index);
        const auto offset = (static_cast<std::size_t>(y) * n + x) * 2;
        states[offset] = state == FineCellState::kFree ? 0.f : state == FineCellState::kBlocked ? 100.f : -1.f;
        states[offset + 1] = static_cast<float>(fine->Confidence(index));
      }
      WriteFloats(prefix + "-" + name + ".bin", states);
      const double extent = n * resolution, midpoint = .5 * extent;
      Pose2 start{.position_m = {1.225, midpoint}, .yaw_rad = 0.};
      LocalTarget target{.center = {extent - 1.225, midpoint}, .is_final_goal = true};
      RequestLocalPlanningView view(fine, start, 0., {});
      start_time = SteadyClock::now();
      const auto deadline = start_time + std::chrono::seconds(5);
      auto result = foot ? LeggedLocalPlanner(legged).Plan(view, start, target, deadline, {})
                         : WheelLocalPlanner(wheel).Plan(view, start, target, deadline, {});
      const double plan_ms = Milliseconds(start_time);
      std::ofstream path(prefix + "-" + name + "-path.csv");
      path << std::setprecision(12) << "x,y\n";
      for (const auto& point : result.path)
        path << point.pose.position_m.x << ',' << point.pose.position_m.y << '\n';
      if (!first) summary << ',';
      first = false;
      summary << JsonString(name) << ":{\"map_ms\":" << map_ms << ",\"plan_ms\":" << plan_ms
              << ",\"status\":" << static_cast<int>(result.status)
              << ",\"reason\":" << JsonString(result.reason_code)
              << ",\"reached\":" << (result.reaches_final_goal ? "true" : "false")
              << ",\"path_points\":" << result.path.size()
              << ",\"has_reference\":" << (!result.path.empty() ? "true" : "false")
              << ",\"plan_found\":" << (result.status == LocalPlanResult::Status::kPlanFound && !result.path.empty() ? "true" : "false") << '}';
    }
    summary << "}}\n";
    if (!summary) throw std::runtime_error("cannot write summary");
    std::cout << prefix << " " << fit_ms << " ms\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
