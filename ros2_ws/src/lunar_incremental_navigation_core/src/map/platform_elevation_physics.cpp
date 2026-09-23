#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <bitset>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "map/grid_bounds.hpp"

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] bool FiniteNonnegative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] Point2 CellCenter(const SparseGridGeometry& geometry,
                                const GridIndex index) noexcept {
  const Vec3 origin = geometry.origin_m();
  const double resolution_m = geometry.resolution_m();
  return Point2{
      .x = std::fma(static_cast<double>(index.x) + 0.5, resolution_m,
                    origin.x),
      .y = std::fma(static_cast<double>(index.y) + 0.5, resolution_m,
                    origin.y),
  };
}

[[nodiscard]] double DistanceToCellArea(
    const Point2 point, const SparseGridGeometry& geometry,
    const GridIndex cell) noexcept {
  const Vec3 origin = geometry.origin_m();
  const double resolution_m = geometry.resolution_m();
  const double point_x_cells = (point.x - origin.x) / resolution_m;
  const double point_y_cells = (point.y - origin.y) / resolution_m;
  const double minimum_x = static_cast<double>(cell.x);
  const double minimum_y = static_cast<double>(cell.y);
  const double dx = std::max(
      {minimum_x - point_x_cells, 0.0,
       point_x_cells - minimum_x - 1.0});
  const double dy = std::max(
      {minimum_y - point_y_cells, 0.0,
       point_y_cells - minimum_y - 1.0});
  return std::hypot(dx, dy) * resolution_m;
}

[[nodiscard]] bool CircleLeavesGeometry(
    const Point2 center, const double radius_m,
    const SparseGridGeometry& geometry) noexcept {
  const Vec3 origin = geometry.origin_m();
  const GridIndex minimum = geometry.min_inclusive();
  const GridIndex maximum = geometry.max_exclusive();
  const double minimum_x = std::fma(
      static_cast<double>(minimum.x), geometry.resolution_m(), origin.x);
  const double minimum_y = std::fma(
      static_cast<double>(minimum.y), geometry.resolution_m(), origin.y);
  const double maximum_x = std::fma(
      static_cast<double>(maximum.x), geometry.resolution_m(), origin.x);
  const double maximum_y = std::fma(
      static_cast<double>(maximum.y), geometry.resolution_m(), origin.y);
  return center.x - radius_m < minimum_x ||
         center.y - radius_m < minimum_y ||
         center.x + radius_m > maximum_x ||
         center.y + radius_m > maximum_y;
}

[[nodiscard]] double DistanceToGeometryBoundary(
    const Point2 center, const SparseGridGeometry& geometry) noexcept {
  const Vec3 origin = geometry.origin_m();
  const GridIndex minimum = geometry.min_inclusive();
  const GridIndex maximum = geometry.max_exclusive();
  const double minimum_x = std::fma(
      static_cast<double>(minimum.x), geometry.resolution_m(), origin.x);
  const double minimum_y = std::fma(
      static_cast<double>(minimum.y), geometry.resolution_m(), origin.y);
  const double maximum_x = std::fma(
      static_cast<double>(maximum.x), geometry.resolution_m(), origin.x);
  const double maximum_y = std::fma(
      static_cast<double>(maximum.y), geometry.resolution_m(), origin.y);
  return std::min({center.x - minimum_x, maximum_x - center.x,
                   center.y - minimum_y, maximum_y - center.y});
}

[[nodiscard]] double SlopeLimit(const PlatformCapability& capability) {
  return std::visit(
      [](const auto& typed_capability) {
        using Capability = std::decay_t<decltype(typed_capability)>;
        if constexpr (std::is_same_v<Capability, WheeledCapability> ||
                      std::is_same_v<Capability, LeggedCapability>) {
          return typed_capability.maximum_slope_rad;
        }
        return 0.0;
      },
      capability);
}

[[nodiscard]] double ReliefLimit(const PlatformCapability& capability) {
  return std::visit(
      [](const auto& typed_capability) {
        using Capability = std::decay_t<decltype(typed_capability)>;
        if constexpr (std::is_same_v<Capability, WheeledCapability>) {
          return typed_capability.maximum_local_obstacle_relief_m;
        } else if constexpr (std::is_same_v<Capability, LeggedCapability>) {
          return typed_capability.maximum_step_height_m;
        }
        return 0.0;
      },
      capability);
}

[[nodiscard]] double NormalizedSquared(const double value,
                                       const double limit) noexcept {
  if (limit <= 0.0) {
    return 0.0;
  }
  const double normalized = std::clamp(value / limit, 0.0, 1.0);
  return normalized * normalized;
}

[[nodiscard]] double SaturatingProduct(const double left,
                                       const double right) noexcept {
  if (left == 0.0 || right == 0.0) {
    return 0.0;
  }
  constexpr double kLargestFinite = std::numeric_limits<double>::max();
  if (left > kLargestFinite / right) {
    return kLargestFinite;
  }
  const double product = left * right;
  return std::isfinite(product) ? product : kLargestFinite;
}

[[nodiscard]] double SaturatingSum(const double left,
                                   const double right) noexcept {
  constexpr double kLargestFinite = std::numeric_limits<double>::max();
  if (left >= kLargestFinite - right) {
    return kLargestFinite;
  }
  const double sum = left + right;
  return std::isfinite(sum) ? sum : kLargestFinite;
}

// Shared terrain measurement: physical window for a robust background plane,
// unsmoothed 3x3 residual extrema for local discontinuities. Platform policies
// below keep wheel clearance and legged support/step limits independent.
struct TerrainPatchPoint {
  int x{}, y{};
  double low{}, high{}, z{}, residual{};
};

template <typename Container>
double PatchMedian(Container& values, std::size_t count) {
  std::sort(values.begin(), values.begin() + count);
  return values[count / 2];
}

template <typename Capability>
IntrinsicTraversalEvaluation EvaluateTerrainPatch(
    const ElevationRangeView& elevation, GridIndex center,
    const Capability& capability, double half_width_m) {
  constexpr bool legged = std::is_same_v<Capability, LeggedCapability>;
  if (!elevation.ElevationRangeAt(center)) return {};
  const double resolution = elevation.geometry().resolution_m();
  const int radius = TerrainBackgroundRadiusCells(half_width_m, resolution);
  const int side = 2 * radius + 1;
  const std::size_t area = side * side;
  std::vector<TerrainPatchPoint> points(area);
  std::vector<double> heights(area);
  std::vector<int> indices(area, -1);
  std::size_t count = 0;
  int min_x = radius, max_x = -radius, min_y = radius, max_y = -radius;
  for (int y = -radius; y <= radius; ++y)
    for (int x = -radius; x <= radius; ++x) {
      const auto cell =
          detail::OffsetWithinGeometry(elevation.geometry(), center, x, y);
      const auto range =
          cell ? elevation.ElevationRangeAt(*cell) : std::nullopt;
      if (!range) continue;
      indices[(y + radius) * side + x + radius] = static_cast<int>(count);
      const double z = .5 * (double(range->min_m) + double(range->max_m));
      points[count++] = {x, y, range->min_m, range->max_m, z, 0.};
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  if (count < (3 * area + 4) / 5 || min_x >= 0 || max_x <= 0 || min_y >= 0 || max_y <= 0)
    return {};

  if constexpr (legged) {
    // Plane coverage is not landing support. Retain the local known-support
    // gate; missing returns must not be smoothed into traversable ground.
    for (int y = radius - 1; y <= radius + 1; ++y)
      for (int x = radius - 1; x <= radius + 1; ++x)
        if (indices[y * side + x] < 0) return {};
  }

  // Median adjacent derivatives initialize the background: a sharp step must
  // not tilt the initial fit into an artificial ramp joining its two surfaces.
  std::vector<double> gx(side * (side - 1)), gy(side * (side - 1));
  std::size_t nx = 0, ny = 0;
  for (int y = 0; y < side; ++y)
    for (int x = 0; x < side; ++x) {
      const int i = indices[y * side + x];
      if (i < 0) continue;
      if (x < side - 1 && indices[y * side + x + 1] >= 0)
        gx[nx++] =
            (points[indices[y * side + x + 1]].z - points[i].z) / resolution;
      if (y < side - 1 && indices[(y + 1) * side + x] >= 0)
        gy[ny++] =
            (points[indices[(y + 1) * side + x]].z - points[i].z) / resolution;
    }
  if (nx < static_cast<std::size_t>(side - 1) ||
      ny < static_cast<std::size_t>(side - 1)) return {};
  double a = PatchMedian(gx, nx), b = PatchMedian(gy, ny);
  for (std::size_t i = 0; i < count; ++i)
    heights[i] = points[i].z - resolution * (a * points[i].x + b * points[i].y);
  double c = PatchMedian(heights, count);
  const double background_a = a, background_b = b, background_c = c;
  for (std::size_t i = 0; i < count; ++i) heights[i] = std::abs(heights[i] - c);
  // A 5 mm robust scale floor prevents nearly exact flats from producing
  // singular weights. It is a numerical/noise scale, not an obstacle threshold.
  const double cutoff =
      2.5 * std::max(.005, 1.4826 * PatchMedian(heights, count));
  double sw = 0., sx = 0., sy = 0., sz = 0., sxx = 0., syy = 0., sxy = 0.,
         sxz = 0., syz = 0.;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& p = points[i];
    const double x = p.x * resolution, y = p.y * resolution;
    const double residual = p.z - (a * x + b * y + c);
    const double w = std::min(1., cutoff / std::max(std::abs(residual), 1e-12));
    sw += w;
    sx += w * x;
    sy += w * y;
    sz += w * p.z;
    sxx += w * x * x;
    syy += w * y * y;
    sxy += w * x * y;
    sxz += w * x * p.z;
    syz += w * y * p.z;
  }
  sxx -= sx * sx / sw;
  syy -= sy * sy / sw;
  sxy -= sx * sy / sw;
  sxz -= sx * sz / sw;
  syz -= sy * sz / sw;
  const double det = sxx * syy - sxy * sxy;
  if (!(sxx > 0. && syy > 0. && det > 1e-8 * sxx * syy)) return {};
  a = (sxz * syy - syz * sxy) / det;
  b = (syz * sxx - sxz * sxy) / det;
  c = (sz - a * sx - b * sy) / sw;
  // For legged steps, even a robust least-squares refinement can tilt toward
  // the other landing surface. Use the median background derivatives for the
  // slope decision; retain the established refined wheel estimate unchanged.
  const double slope = legged ? std::atan(std::hypot(background_a, background_b))
                              : std::atan(std::hypot(a, b));
  if (!std::isfinite(slope) || !std::isfinite(c)) return {};
  double low = std::numeric_limits<double>::infinity(), high = -low,
         center_residual = 0.;
  for (std::size_t i = 0; i < count; ++i) {
    auto& p = points[i];
    // Use the robust background before least-squares refinement for
    // discontinuities: even a small fitted tilt must not shave a 21 cm
    // step below the existing 20 cm limit.
    const double plane =
        resolution * (background_a * p.x + background_b * p.y) + background_c;
    p.residual = p.z - plane;
    p.low -= plane;
    p.high -= plane;
    if (p.x == 0 && p.y == 0) center_residual = p.residual;
    if (std::abs(p.x) <= 1 && std::abs(p.y) <= 1) {
      low = std::min(low, p.low);
      high = std::max(high, p.high);
    }
  }
  const double relief = high - low;
  const double rise = std::max(0., high - center_residual);
  const double relief_limit = [&] {
    if constexpr (legged) return capability.maximum_step_height_m;
    else return capability.maximum_local_obstacle_relief_m;
  }();
  const double rise_limit = [&] {
    if constexpr (legged) return std::numeric_limits<double>::infinity();
    else return capability.minimum_underbody_clearance_m;
  }();
  // A legged step can raise the body onto another supported surface. Wheel
  // underbody rise is not a legged step limit; full 3D body/footfall execution
  // certification remains outside this 2.5D terrain evaluator.
  const bool discontinuity = relief > relief_limit || rise > rise_limit;
  if (discontinuity || (legged && relief > .0125)) {
    // Meaningful legged relief requires two supported surfaces even below the
    // step limit. An isolated tall/low return is UNKNOWN, never free landing.
    const double band = legged ? std::min(.0125, .25 * relief_limit)
                               : .25 * std::min(relief_limit, rise_limit);
    bool low_supported = false, high_supported = false;
    // A larger plane fit must not enlarge the local support certificate.
    std::array<std::size_t, 25> support{};
    std::size_t support_count = 0;
    for (std::size_t i = 0; i < count; ++i)
      if (std::abs(points[i].x) <= 2 && std::abs(points[i].y) <= 2)
        support[support_count++] = i;
    for (std::size_t i = 0; i < support_count; ++i)
      for (std::size_t j = i + 1; j < support_count; ++j) {
        const auto& p = points[support[i]];
        const auto& q = points[support[j]];
        if (std::max(std::abs(p.x - q.x), std::abs(p.y - q.y)) != 1) continue;
        if constexpr (legged) {
          // Compare interval endpoints with interval extrema. Midpoints cannot
          // certify even identical adjacent ranges wider than the support band.
          low_supported |= p.low <= low + band && q.low <= low + band;
          high_supported |= p.high >= high - band && q.high >= high - band;
        } else {
          low_supported |= p.residual <= low + band && q.residual <= low + band;
          high_supported |=
              p.residual >= high - band && q.residual >= high - band;
        }
      }
    if (!low_supported || !high_supported)
      return {.state = IntrinsicCellState::kUnknown,
              .slope_rad = slope,
              .relief_m = relief};
  }
  return {.state = (discontinuity || slope > capability.maximum_slope_rad)
                       ? IntrinsicCellState::kBlocked
                       : IntrinsicCellState::kFree,
          .slope_rad = slope,
          .relief_m = relief};
}

class WheelElevationEvaluator final : public PlatformElevationEvaluator {
 public:
  explicit WheelElevationEvaluator(WheeledCapability capability, double half_width_m)
      : capability_(std::move(capability)), half_width_m_(half_width_m) {
    if (!FiniteNonnegative(capability_.maximum_slope_rad) ||
        !FiniteNonnegative(capability_.maximum_local_obstacle_relief_m) ||
        !FiniteNonnegative(capability_.minimum_underbody_clearance_m)) {
      throw std::invalid_argument(
          "wheel elevation limits must be finite and nonnegative");
    }
  }

  [[nodiscard]] IntrinsicTraversalEvaluation Evaluate(
      const ElevationRangeView& elevation,
      const GridIndex index) const override {
    return EvaluateTerrainPatch(elevation, index, capability_, half_width_m_);
  }

 private:
  WheeledCapability capability_;
  double half_width_m_{};
};

class LeggedElevationEvaluator final : public PlatformElevationEvaluator {
 public:
  explicit LeggedElevationEvaluator(LeggedCapability capability, double half_width_m)
      : capability_(std::move(capability)), half_width_m_(half_width_m) {
    if (!FiniteNonnegative(capability_.maximum_slope_rad) ||
        !FiniteNonnegative(capability_.maximum_step_height_m) ||
        !FiniteNonnegative(capability_.maximum_gap_width_m)) {
      throw std::invalid_argument(
          "legged elevation limits must be finite and nonnegative");
    }
  }

  [[nodiscard]] IntrinsicTraversalEvaluation Evaluate(
      const ElevationRangeView& elevation,
      const GridIndex index) const override {
    return EvaluateTerrainPatch(elevation, index, capability_, half_width_m_);
  }

 private:
  LeggedCapability capability_;
  double half_width_m_{};
};

}  // namespace

int TerrainBackgroundRadiusCells(double half_width_m, double resolution_m) {
  if (!std::isfinite(half_width_m) || half_width_m < 0.0 ||
      !std::isfinite(resolution_m) || resolution_m <= 0.0)
    throw std::invalid_argument("invalid terrain background geometry");
  const double cells = std::ceil(std::nextafter(half_width_m / resolution_m, 0.0));
  if (cells > 32.0)
    throw std::invalid_argument("terrain background exceeds 32-cell half-width");
  return std::max(2, static_cast<int>(cells));
}

std::unique_ptr<const PlatformElevationEvaluator>
MakePlatformElevationEvaluator(const PlatformCapability& capability,
                               const double background_half_width_m) {
  if (!FiniteNonnegative(background_half_width_m))
    throw std::invalid_argument("terrain background width must be finite and nonnegative");
  return std::visit(
      [background_half_width_m](const auto& typed_capability)
          -> std::unique_ptr<const PlatformElevationEvaluator> {
        using Capability = std::decay_t<decltype(typed_capability)>;
        if constexpr (std::is_same_v<Capability, WheeledCapability>) {
          return std::make_unique<const WheelElevationEvaluator>(
              typed_capability, background_half_width_m);
        } else if constexpr (std::is_same_v<Capability, LeggedCapability>) {
          return std::make_unique<const LeggedElevationEvaluator>(
              typed_capability, background_half_width_m);
        } else {
          throw std::invalid_argument(
              "fine traversability supports wheel or legged capability");
        }
      },
      capability);
}

struct FineCellEvaluator::Impl final {
  struct IntrinsicCacheTile final {
    std::array<IntrinsicTraversalEvaluation, kGridTileCellCount> evaluations;
    std::bitset<kGridTileCellCount> populated;
    // Lazy 16-cell row summaries; scoped to this immutable elevation view.
    std::array<std::uint16_t, kGridTileCellCount / 16> hazards{};
    std::array<std::uint16_t, kGridTileCellCount / 16> blocked{};
    std::bitset<kGridTileCellCount / 16> summarized;
  };

  Impl(const ElevationRangeView& elevation_value,
       PlatformCapability capability_value,
       TraversabilityProfile profile_value)
      : elevation(elevation_value),
        capability(std::move(capability_value)),
        profile(std::move(profile_value)),
        intrinsic_evaluator(MakePlatformElevationEvaluator(
            capability, profile.terrain_background_half_width_m)),
        hard_radius_m(CircumscribedRadius(profile.planar_envelope_xy_m)) {
    if (!FiniteNonnegative(profile.preferred_clearance_m) ||
        !FiniteNonnegative(profile.slope_weight) ||
        !FiniteNonnegative(profile.relief_weight) ||
        !FiniteNonnegative(profile.clearance_weight)) {
      throw std::invalid_argument(
          "fine cell soft costs must be finite and nonnegative");
    }
  }

  [[nodiscard]] const IntrinsicTraversalEvaluation& IntrinsicAt(
      const GridIndex index) {
    const TileIndex tile_index = TileForCell(index);
    if (last_cache_tile == nullptr || tile_index != last_cache_tile_index) {
      auto [found, inserted] = intrinsic_cache.try_emplace(tile_index);
      if (inserted) {
        found->second = std::make_unique<IntrinsicCacheTile>();
      }
      last_cache_tile_index = tile_index;
      last_cache_tile = found->second.get();
    }
    const std::size_t offset = TileCellOffset(index);
    if (!last_cache_tile->populated.test(offset)) {
      last_cache_tile->evaluations[offset] =
          intrinsic_evaluator->Evaluate(elevation, index);
      last_cache_tile->populated.set(offset);
      ++evaluated_elevation_cell_count;
    }
    return last_cache_tile->evaluations[offset];
  }

  // Invoke only for non-FREE intrinsic cells, preserving row-major order.
  // Summaries skip empty spans without changing circle/cell intersection math.
  template <typename Visitor>
  bool VisitHazards(GridIndex minimum, GridIndex maximum, Visitor visitor,
                    bool blocked_only = false) {
    for (auto y = minimum.y; y < maximum.y; ++y) {
      for (auto x = minimum.x; x < maximum.x;) {
        const GridIndex seed{x, y};
        const auto offset = TileCellOffset(seed);
        const auto lane = offset % 16;
        const auto count = std::min<std::int64_t>(maximum.x - x, 16 - lane);
        static_cast<void>(IntrinsicAt(seed));
        auto* tile = last_cache_tile;
        const auto block = offset / 16;
        if (!tile->summarized.test(block)) {
          std::uint16_t mask = 0, blocked = 0;
          const auto base = x - static_cast<std::int64_t>(lane);
          for (int i = 0; i < 16; ++i) {
            const GridIndex cell{base + i, y};
            if (!elevation.geometry().Contains(cell)) continue;
            const auto state = IntrinsicAt(cell).state;
            if (state != IntrinsicCellState::kFree)
              mask |= std::uint16_t(1U << i);
            if (state == IntrinsicCellState::kBlocked)
              blocked |= std::uint16_t(1U << i);
          }
          tile->hazards[block] = mask;
          tile->blocked[block] = blocked;
          tile->summarized.set(block);
        }
        const unsigned candidates = blocked_only ? tile->blocked[block] : tile->hazards[block];
        unsigned mask = (candidates >> lane) & ((1U << count) - 1);
        while (mask) {
          const auto bit = std::countr_zero(mask);
          if (!visitor(GridIndex{x + bit, y})) return false;
          mask &= mask - 1;
        }
        x += count;
      }
    }
    return true;
  }

  [[nodiscard]] FineCellEvaluation Evaluate(const GridIndex candidate) {
    const SparseGridGeometry& geometry = elevation.geometry();
    if (!geometry.Contains(candidate)) {
      return {};
    }
    const Point2 center = CellCenter(geometry, candidate);
    bool intersects_unknown =
        CircleLeavesGeometry(center, hard_radius_m, geometry);
    const auto [minimum, maximum] =
        detail::CellBoundsForRadius(geometry, candidate, hard_radius_m);
    // Resolve BLOCKED first. Once absent, the first intersecting UNKNOWN
    // certifies UNKNOWN; scanning every other unknown cell cannot change it.
    const bool clear = VisitHazards(minimum, maximum, [&](GridIndex seed) {
      return !CircleIntersectsCellArea(center, hard_radius_m, geometry, seed);
    }, true);
    if (!clear) return {.state = FineCellState::kBlocked};
    if (!intersects_unknown) {
      intersects_unknown = !VisitHazards(minimum, maximum, [&](GridIndex seed) {
        return !CircleIntersectsCellArea(center, hard_radius_m, geometry, seed);
      });
    }
    if (intersects_unknown) {
      return {.state = FineCellState::kUnknown};
    }

    const IntrinsicTraversalEvaluation& intrinsic = IntrinsicAt(candidate);
    const double slope_cost = SaturatingProduct(
        profile.slope_weight,
        NormalizedSquared(intrinsic.slope_rad, SlopeLimit(capability)));
    const double relief_cost = SaturatingProduct(
        profile.relief_weight,
        NormalizedSquared(intrinsic.relief_m, ReliefLimit(capability)));
    double traversal_cost = SaturatingSum(slope_cost, relief_cost);
    if (profile.clearance_weight == 0.0 ||
        profile.preferred_clearance_m == 0.0) {
      return {.state = FineCellState::kFree,
              .traversal_cost = traversal_cost};
    }

    const double influence_m =
        hard_radius_m + profile.preferred_clearance_m;
    double nearest_hazard_m = DistanceToGeometryBoundary(center, geometry);
    const auto [clearance_minimum, clearance_maximum] =
        detail::CellBoundsForRadius(geometry, candidate, influence_m);
    VisitHazards(clearance_minimum, clearance_maximum, [&](GridIndex seed) {
      nearest_hazard_m = std::min(
          nearest_hazard_m, DistanceToCellArea(center, geometry, seed));
      return true;
    });
    const double remaining_clearance_m =
        std::max(0.0, nearest_hazard_m - hard_radius_m);
    const double clearance_penalty = std::clamp(
        1.0 - remaining_clearance_m / profile.preferred_clearance_m, 0.0,
        1.0);
    traversal_cost = SaturatingSum(
        traversal_cost,
        SaturatingProduct(profile.clearance_weight, clearance_penalty));
    return {.state = FineCellState::kFree,
            .traversal_cost = traversal_cost};
  }

  const ElevationRangeView& elevation;
  PlatformCapability capability;
  TraversabilityProfile profile;
  std::unique_ptr<const PlatformElevationEvaluator> intrinsic_evaluator;
  double hard_radius_m{};
  std::map<TileIndex, std::unique_ptr<IntrinsicCacheTile>> intrinsic_cache;
  TileIndex last_cache_tile_index;
  IntrinsicCacheTile* last_cache_tile{};
  std::size_t evaluated_elevation_cell_count{};
};

FineCellEvaluator::FineCellEvaluator(
    const ElevationRangeView& elevation,
    const PlatformCapability& capability,
    const TraversabilityProfile& profile)
    : impl_(std::make_unique<Impl>(elevation, capability, profile)) {}

FineCellEvaluator::~FineCellEvaluator() = default;
FineCellEvaluator::FineCellEvaluator(FineCellEvaluator&&) noexcept = default;
FineCellEvaluator& FineCellEvaluator::operator=(FineCellEvaluator&&) noexcept =
    default;

FineCellEvaluation FineCellEvaluator::Evaluate(const GridIndex index) {
  return impl_->Evaluate(index);
}

double FineCellEvaluator::hard_inflation_radius_m() const noexcept {
  return impl_->hard_radius_m;
}

std::size_t FineCellEvaluator::evaluated_elevation_cells() const noexcept {
  return impl_->evaluated_elevation_cell_count;
}

std::size_t FineCellEvaluator::cached_elevation_tiles() const noexcept {
  return impl_->intrinsic_cache.size();
}

}  // namespace lunar::incremental_navigation
