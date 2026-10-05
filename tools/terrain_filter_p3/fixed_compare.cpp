// Frozen P3 single-frame comparison. No ground-truth accuracy claim.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"
#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"
#include "lunar_incremental_navigation_core/types/terrain_fit_method.hpp"
#include "lunar_incremental_navigation_core/types/terrain_filter_method.hpp"
#include "map/terrain_observation_estimator.hpp"
#include "capabilities.hpp"

using namespace lunar::incremental_navigation;
using namespace lunar::incremental_navigation::map;
namespace {
constexpr int kN = 320;
constexpr double kResolution = .05;
double Milliseconds(SteadyClock::time_point start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now()-start).count();
}
std::string Quoted(const std::string& text) {
  std::string output = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\') output += '\\';
    if (c == '\n') output += "\\n";
    else output += c;
  }
  return output + '"';
}
std::string Bool(bool value) { return value ? "true" : "false"; }
std::string NumberOrNull(const std::optional<double> value) {
  if(!value) return "null";
  std::ostringstream output;
  output << std::setprecision(12) << *value;
  return output.str();
}
const char* StateName(FineCellState value) {
  switch (value) {
    case FineCellState::kFree: return "FREE";
    case FineCellState::kBlocked: return "BLOCKED";
    case FineCellState::kUnknown: return "UNKNOWN";
  }
  return "INVALID";
}
const char* PatchName(StartPatchResult::Status value) {
  switch (value) {
    case StartPatchResult::Status::kNotNeeded: return "NOT_NEEDED";
    case StartPatchResult::Status::kReady: return "READY";
    case StartPatchResult::Status::kStartBlocked: return "START_BLOCKED";
    case StartPatchResult::Status::kUnresolved: return "UNRESOLVED";
  }
  return "INVALID";
}
const char* PlanName(LocalPlanResult::Status value) {
  switch (value) {
    case LocalPlanResult::Status::kPlanFound: return "PLAN_FOUND";
    case LocalPlanResult::Status::kNoPath: return "NO_PATH";
    case LocalPlanResult::Status::kTimeout: return "TIMEOUT";
    case LocalPlanResult::Status::kCanceled: return "CANCELED";
  }
  return "INVALID";
}
GridIndex Cell(Point2 point) {
  return {static_cast<std::int64_t>(std::floor(point.x/kResolution)),
          static_cast<std::int64_t>(std::floor(point.y/kResolution))};
}
template<class Value>
void WriteBinary(const std::string& path, const std::vector<Value>& values) {
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(values.data()),
               static_cast<std::streamsize>(values.size()*sizeof(Value)));
  if (!output) throw std::runtime_error("cannot write "+path);
}
void WriteRange(std::ostream& output, const std::optional<ElevationRange>& range) {
  if (range) output << "{\"min_m\":" << range->min_m << ",\"max_m\":" << range->max_m << '}';
  else output << "null";
}
void WriteEndpoint(std::ostream& output, const Point2 position,
                   const ElevationSnapshot& raw, const FineTraversabilitySnapshot& fine,
                   const RequestLocalPlanningView& view, TerrainObservationEstimator& observations) {
  const auto cell = Cell(position);
  const auto raw_range = raw.ElevationRangeAt(cell);
  const auto effective = fine.PlanningElevationRangeAt(cell);
  const auto& estimate = observations.Estimate(cell);
  output << "{\"position_m\":[" << position.x << ',' << position.y << "],\"cell\":["
         << cell.x << ',' << cell.y << "],\"raw_available\":" << Bool(bool(raw_range))
         << ",\"effective_available\":" << Bool(bool(effective))
         << ",\"estimate_available\":" << Bool(bool(estimate))
         << ",\"estimate_confidence\":" << NumberOrNull(estimate ? std::optional<double>(estimate->confidence) : std::nullopt)
         << ",\"estimated_height_m\":" << NumberOrNull(estimate ? std::optional<double>(estimate->surface_height_m) : std::nullopt)
         << ",\"fine_state\":" << Quoted(StateName(fine.State(cell)))
         << ",\"fine_confidence\":" << fine.Confidence(cell)
         << ",\"request_source\":" << static_cast<int>(view.Source(cell))
         << ",\"can_be_endpoint\":" << Bool(view.CanBeEndpoint(cell)) << ",\"raw_range\":";
  WriteRange(output, raw_range);
  output << ",\"effective_range\":";
  WriteRange(output, effective);
  output << '}';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 5 && argc != 6) {
      std::cerr << "usage: fixed_compare INPUT_F32 OUTPUT_PREFIX METHOD FILTER [REPEAT=0]\n";
      return 2;
    }
    const std::string prefix = argv[2];
    const auto method = ParseTerrainFitMethod(argv[3]);
    const auto filter = ParseTerrainFilterMethod(argv[4]);
    const int repeat = argc == 6 ? std::stoi(argv[5]) : 0;
    if (repeat < 0) throw std::invalid_argument("repeat must be nonnegative");
    const auto parent = std::filesystem::path(prefix).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::vector<float> heights(kN*kN);
    std::ifstream input(argv[1], std::ios::binary);
    input.read(reinterpret_cast<char*>(heights.data()), heights.size()*sizeof(float));
    if (!input || input.peek() != std::char_traits<char>::eof())
      throw std::runtime_error("P3 input must contain exactly 320x320 float32 cells");
    GridGeometry geometry{.frame_id="map", .width=kN, .height=kN,
                          .resolution_m=kResolution, .origin_m={}};
    PersistentElevationMap map;
    (void)map.Apply({.geometry=geometry, .elevation_m=heights,
                    .map_from_source={.parent_frame="map", .child_frame="map",
                                      .stamp={}, .translation_m={}, .rotation={}}});
    auto raw = map.Snapshot();
    const auto capability = MakeComparisonLegged();
    TraversabilityProfile profile;
    profile.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};
    profile.terrain_background_half_width_m=.2;
    profile.preferred_clearance_m=.4;
    profile.clearance_weight=2.; profile.slope_weight=1.; profile.relief_weight=1.;
    profile.confidence_weight=2.; profile.terrain_fit_method=method;
    profile.terrain_filter_method=filter;
    // Preserve the historical fixed request's 3 m start-blind-zone allowance.
    // Its assumed cells are reported separately from inferred ground.
    profile.start_blind_zone_margin_m=3.;
    const Pose2 start{.position_m={7.6133745667,7.9915921022}, .yaw_rad=-3.022577};
    const LocalTarget goal{.center={8.6133745667,7.9915921022}, .is_final_goal=true,
                           .terminal_yaw_rad=std::nullopt, .region={}};

    const auto map_start = SteadyClock::now();
    auto fine = FineTraversabilityBuilder().Derive(raw, capability, profile);
    const double map_ms = Milliseconds(map_start);
    const auto patch_start = SteadyClock::now();
    auto patch = RequestLocalStartPatchBuilder().Build(fine, start, capability, profile);
    const double start_patch_ms = Milliseconds(patch_start);
    RequestLocalPlanningView base_view(fine, start, 0., {});
    const auto& view = patch.view ? *patch.view : base_view;
    const auto plan_start = SteadyClock::now();
    const auto result = LeggedLocalPlanner(capability).Plan(
        view, start, goal, plan_start+std::chrono::seconds(5), {});
    const double plan_ms = Milliseconds(plan_start);
    const bool plan_found = result.status == LocalPlanResult::Status::kPlanFound && !result.path.empty();

    std::ofstream path(prefix+"-path.csv");
    path << std::setprecision(12) << "x,y,phase,request_source,raw_available,effective_available\n";
    std::size_t path_assumed_points=0, path_unknown_points=0;
    std::size_t path_infilled_points=0, path_raw_observed_points=0;
    std::size_t path_free_points=0, path_blocked_points=0, path_fine_unknown_points=0;
    double minimum_path_fine_confidence=1.;
    double path_length=0.;
    for (std::size_t i=0; i<result.path.size(); ++i) {
      const auto& point = result.path[i];
      const Point2 position{point.pose.position_m.x,point.pose.position_m.y};
      const auto cell = Cell(position); const auto source = view.Source(cell);
      path_assumed_points += source == LocalCellSource::kStartAssumedFree;
      path_unknown_points += source == LocalCellSource::kUnknown;
      const bool raw_available=raw->ElevationRangeAt(cell).has_value();
      const bool effective_available=fine->PlanningElevationRangeAt(cell).has_value();
      path_raw_observed_points += raw_available;
      path_infilled_points += !raw_available && effective_available;
      path_free_points += fine->State(cell)==FineCellState::kFree;
      path_blocked_points += fine->State(cell)==FineCellState::kBlocked;
      path_fine_unknown_points += fine->State(cell)==FineCellState::kUnknown;
      minimum_path_fine_confidence=std::min(minimum_path_fine_confidence,fine->Confidence(cell));
      if (i) path_length += std::hypot(position.x-result.path[i-1].pose.position_m.x,
                                      position.y-result.path[i-1].pose.position_m.y);
      // Coordinates are the original 0..16 m map lattice. No display transform.
      path << position.x << ',' << position.y << ',' << static_cast<int>(point.phase)
           << ',' << static_cast<int>(source) << ',' << Bool(bool(raw->ElevationRangeAt(cell)))
           << ',' << Bool(bool(fine->PlanningElevationRangeAt(cell))) << '\n';
    }
    if (!path) throw std::runtime_error("cannot write path");

    TerrainObservationEstimator observations(*raw,{.fit_method=method,.filter_method=filter});
    std::vector<float> ground(heights.size()*6);
    std::vector<std::int8_t> states(heights.size());
    std::vector<float> confidence(heights.size());
    std::size_t raw_observed=0, missing=0, candidate_infilled=0, planning_infilled=0;
    std::size_t observed_estimated=0, unknown=0, blocked=0, free=0, near_free=0, near_total=0;
    std::size_t high_conf_infilled=0, fallback_infilled=0;
    const auto fit_start = SteadyClock::now();
    for (int y=0; y<kN; ++y) for (int x=0; x<kN; ++x) {
      const GridIndex cell{x,y};
      const std::size_t flat=static_cast<std::size_t>(y)*kN+x;
      const std::size_t offset=flat*6;
      const bool observed=raw->ElevationRangeAt(cell).has_value();
      raw_observed+=observed; missing+=!observed;
      const auto& estimate=observations.Estimate(cell);
      ground[offset]=NAN; ground[offset+1]=0.; ground[offset+2]=NAN;
      ground[offset+3]=NAN; ground[offset+4]=NAN; ground[offset+5]=0.;
      if (estimate) {
        ground[offset]=estimate->surface_height_m; ground[offset+1]=estimate->confidence;
        ground[offset+2]=estimate->gradient.x; ground[offset+3]=estimate->gradient.y;
        ground[offset+4]=estimate->predictive_variance_m2;
        ground[offset+5]=estimate->used_fallback ? 1.f : 0.f;
        candidate_infilled+=!observed; observed_estimated+=observed;
        high_conf_infilled+=!observed && estimate->confidence>=.6;
        fallback_infilled+=!observed && estimate->used_fallback;
      }
      const auto state=fine->State(cell);
      unknown+=state==FineCellState::kUnknown;
      blocked+=state==FineCellState::kBlocked;
      free+=state==FineCellState::kFree;
      states[flat]=state==FineCellState::kFree ? 0 : state==FineCellState::kBlocked ? 100 : -1;
      confidence[flat]=static_cast<float>(fine->Confidence(cell));
      planning_infilled+=!observed && fine->PlanningElevationRangeAt(cell).has_value();
      const double dx=(x+.5)*kResolution-start.position_m.x;
      const double dy=(y+.5)*kResolution-start.position_m.y;
      if (dx*dx+dy*dy<=1.) { ++near_total; near_free+=state==FineCellState::kFree; }
    }
    // This includes per-cell state/counter reads as well as independent fitting.
    // map_ms and plan_ms above are separately timed and exclude this export pass.
    const double fit_export_ms=Milliseconds(fit_start);
    WriteBinary(prefix+"-ground.bin",ground);
    WriteBinary(prefix+"-fine.i8",states);
    WriteBinary(prefix+"-confidence.f32",confidence);
    std::ofstream output(prefix+"-summary.json");
    output << std::setprecision(12)
           << "{\"method\":" << Quoted(std::string(TerrainFitMethodName(method)))
           << ",\"filter\":" << Quoted(std::string(TerrainFilterMethodName(filter)))
           << ",\"repeat\":" << repeat << ",\"platform\":\"foot\",\"n\":" << kN
           << ",\"resolution_m\":" << kResolution
           << ",\"raw_observed_count\":" << raw_observed << ",\"missing_count\":" << missing
           << ",\"candidate_ground_infilled_count\":" << candidate_infilled
           << ",\"planning_ground_infilled_count\":" << planning_infilled
           << ",\"observed_ground_estimated_count\":" << observed_estimated
           << ",\"high_confidence_infilled_count\":" << high_conf_infilled
           << ",\"fallback_infilled_count\":" << fallback_infilled
           << ",\"fine_free_count\":" << free << ",\"fine_blocked_count\":" << blocked
           << ",\"fine_unknown_count\":" << unknown << ",\"near_1m_free_count\":" << near_free
           << ",\"near_1m_total_count\":" << near_total << ",\"map_ms\":" << map_ms
           << ",\"start_patch_ms\":" << start_patch_ms << ",\"plan_ms\":" << plan_ms
           << ",\"fit_and_export_loop_ms\":" << fit_export_ms
           << ",\"deadline_seconds\":5,\"confidence_weight\":2,\"minimum_confidence\":0.2"
           << ",\"maximum_residual_m\":0.03,\"start_blind_zone_margin_m\":3"
           << ",\"start_patch_status\":" << static_cast<int>(patch.status)
           << ",\"start_patch_status_name\":" << Quoted(PatchName(patch.status))
           << ",\"assumed_cells\":" << patch.assumed_cells
           << ",\"local_status\":" << static_cast<int>(result.status)
           << ",\"local_status_name\":" << Quoted(PlanName(result.status))
           << ",\"local_reason_code\":" << Quoted(result.reason_code)
           << ",\"plan_found\":" << Bool(plan_found)
           << ",\"has_reference\":" << Bool(!result.path.empty())
           << ",\"reaches_final_goal\":" << Bool(result.reaches_final_goal)
           << ",\"path_points\":" << result.path.size()
           << ",\"path_length_m\":" << path_length
           << ",\"path_assumed_points\":" << path_assumed_points
           << ",\"path_unknown_points\":" << path_unknown_points
           << ",\"path_infilled_points\":" << path_infilled_points
           << ",\"path_raw_observed_points\":" << path_raw_observed_points
           << ",\"path_fine_free_points\":" << path_free_points
           << ",\"path_fine_blocked_points\":" << path_blocked_points
           << ",\"path_fine_unknown_points\":" << path_fine_unknown_points
           << ",\"minimum_path_fine_confidence\":" << NumberOrNull(result.path.empty() ? std::nullopt : std::optional<double>(minimum_path_fine_confidence))
           << ",\"platform_profile_hash\":" << Quoted(fine->platform_profile_hash())
           << ",\"start_yaw_rad\":" << start.yaw_rad << ",\"start\":";
    WriteEndpoint(output,start.position_m,*raw,*fine,view,observations);
    output << ",\"goal\":";
    WriteEndpoint(output,goal.center,*raw,*fine,view,observations);
    output << ",\"boundaries\":{\"ground_truth\":\"unavailable; no accuracy claim\","
              "\"input\":\"historical frozen P3 height frame; not live sensor validation\","
              "\"pose\":\"historical pose about 2.2 seconds later; not synchronized replay\","
              "\"planning\":\"standalone core result; not ROS Action or closed-loop result\"}}\n";
    if (!output) throw std::runtime_error("cannot write summary");
    std::cout << prefix << ": status=" << static_cast<int>(result.status)
              << " path=" << result.path.size() << " map_ms=" << map_ms << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
