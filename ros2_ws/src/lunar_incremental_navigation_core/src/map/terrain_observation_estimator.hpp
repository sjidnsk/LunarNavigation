#pragma once

#include <array>
#include <bitset>
#include <map>
#include <memory>
#include <optional>
#include <vector>
#include "lunar_incremental_navigation_core/elevation_map.hpp"

namespace lunar::incremental_navigation::map {
int ObservationBackgroundRadiusCells(double half_width_m, double resolution_m);

struct TerrainObservationParameters {
  double support_axis_distance_m{.20};
  double maximum_plane_residual_m{.03};
  double minimum_confidence{.20};
};
struct GroundEstimate {
  double surface_height_m{};
  Vec2 gradient{};
  double residual_rms_m{};
  double residual_min_m{}, residual_max_m{};
  double confidence{};
};

// One immutable input view per instance. Request-local assumed views must use
// their own instance; results never become observations in the input view.
class TerrainObservationEstimator {
 public:
  explicit TerrainObservationEstimator(const ElevationRangeView& view,
                                        TerrainObservationParameters parameters = {});
  const std::optional<GroundEstimate>& Estimate(GridIndex index);
  int dependency_radius_cells() const noexcept { return radius_; }
  std::size_t evaluated_cells() const noexcept { return evaluated_; }
  std::size_t allocated_bytes() const noexcept;
 private:
  static constexpr std::int64_t kCacheWidth = 16;
  static constexpr std::size_t kCacheCells = kCacheWidth * kCacheWidth;
  struct Tile {
    std::array<std::optional<GroundEstimate>, kCacheCells> values;
    std::bitset<kCacheCells> populated;
  };
  std::optional<GroundEstimate> Compute(GridIndex index) const;
  const ElevationRangeView& view_;
  TerrainObservationParameters parameters_;
  int radius_{};
  std::size_t evaluated_{};
  std::map<TileIndex, std::unique_ptr<Tile>> tiles_;
  TileIndex last_tile_index_{};
  Tile* last_tile_{};
};

struct TerrainPatchPoint {
  int x{}, y{};
  double low{}, high{}, z{}, residual{};
};
struct TerrainPatchObservation {
  std::vector<TerrainPatchPoint> points;
  std::size_t observed_count{};
  double refined_slope_rad{}, median_slope_rad{};
  double low{}, high{}, center_residual{}, confidence{};
  bool central_complete{}, continuous_ground{}, center_available{};
};
// Physical-scale background and unsmoothed central residuals. No platform
// limits are applied here; points beyond observed_count are inferred only.
std::optional<TerrainPatchObservation> MeasureTerrainPatch(
    const ElevationRangeView&, GridIndex, double half_width_m,
    TerrainObservationEstimator&);
}  // namespace lunar::incremental_navigation::map
