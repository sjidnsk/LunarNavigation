// 对照程序直接调用既有 core，独立于生产桥的累计、分组和 RLE 实现。
#include <cmath>
#include <cstdint>
#include <optional>

#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"

namespace nav = lunar::incremental_navigation;

namespace {
class DirectGrid final : public nav::ElevationRangeView {
 public:
  DirectGrid(const float* data, int h, int w, double r)
      : values(data), width(w), geometry_("map", r, {}, {0, 0}, {w, h}) {}
  const nav::SparseGridGeometry& geometry() const noexcept override { return geometry_; }
  std::optional<nav::ElevationRange> ElevationRangeAt(nav::GridIndex i) const noexcept override {
    if (!geometry_.Contains(i)) return std::nullopt;
    float value = values[i.y * width + i.x];
    if (!std::isfinite(value)) return std::nullopt;
    return nav::ElevationRange{value, value};
  }
  const float* values;
  int width;
  nav::SparseGridGeometry geometry_;
};
}  // namespace

#if defined(_WIN32)
#define TERRAIN_EXPORT __declspec(dllexport)
#else
#define TERRAIN_EXPORT __attribute__((visibility("default")))
#endif

extern "C" TERRAIN_EXPORT int nav1cm_terrain_reference_v1(
    const float* data, int height, int width, double resolution, double slope,
    double relief, double clearance, std::int8_t* intrinsic, std::int8_t* inflated) {
  try {
    DirectGrid grid(data, height, width, resolution);
    nav::WheeledCapability wheel;
    wheel.maximum_slope_rad = slope;
    wheel.maximum_local_obstacle_relief_m = relief;
    wheel.minimum_underbody_clearance_m = clearance;
    wheel.footprint_xy_m = {{.591, .409}, {.591, -.409}, {-.591, -.409}, {-.591, .409}};
    nav::TraversabilityProfile profile;
    profile.planar_envelope_xy_m = wheel.footprint_xy_m;
    auto evaluator = nav::MakePlatformElevationEvaluator(wheel);
    nav::FineCellEvaluator fine(grid, wheel, profile);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto raw = evaluator->Evaluate(grid, {x, y}).state;
        const auto expanded = fine.Evaluate({x, y}).state;
        intrinsic[y * width + x] = raw == nav::IntrinsicCellState::kBlocked ? 1 :
            raw == nav::IntrinsicCellState::kFree ? 0 : -1;
        inflated[y * width + x] = expanded == nav::FineCellState::kBlocked ? 1 :
            expanded == nav::FineCellState::kFree ? 0 : -1;
      }
    }
    return 0;
  } catch (...) {
    return 1;
  }
}
