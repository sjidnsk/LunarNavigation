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
#include "lunar_incremental_navigation_core/terrain_batch.hpp"
#include "lunar_incremental_navigation_core/terrain_dependency.hpp"
#include "map/terrain_observation_estimator.hpp"

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

}  // namespace

[[nodiscard]] LocalTerrainMeasurements MeasureLocalTerrain(
    const ElevationRangeView& elevation, const GridIndex center) {
  if (const auto measured = elevation.TerrainMeasurementsAt(center)) {
    return *measured;
  }
  const std::optional<ElevationRange> center_range =
      elevation.ElevationRangeAt(center);
  if (!center_range) {
    return {};
  }

  const double center_mid =
      0.5 * (static_cast<double>(center_range->min_m) +
             static_cast<double>(center_range->max_m));
  double minimum = center_range->min_m;
  double maximum = center_range->max_m;
  double maximum_slope = 0.0;
  double maximum_positive_rise = 0.0;
  bool complete = true;
  const double resolution_m = elevation.geometry().resolution_m();
  for (std::int64_t dy = -1; dy <= 1; ++dy) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const std::optional<GridIndex> neighbor_index =
          detail::OffsetWithinGeometry(elevation.geometry(), center, dx, dy);
      const std::optional<ElevationRange> neighbor =
          neighbor_index ? elevation.ElevationRangeAt(*neighbor_index)
                         : std::nullopt;
      if (!neighbor_index || !neighbor) {
        complete = false;
        continue;
      }
      const double neighbor_mid =
          0.5 * (static_cast<double>(neighbor->min_m) +
                 static_cast<double>(neighbor->max_m));
      const double planar_distance_m =
          resolution_m * std::hypot(static_cast<double>(dx),
                                    static_cast<double>(dy));
      maximum_slope =
          std::max(maximum_slope,
                   std::atan2(std::abs(neighbor_mid - center_mid),
                              planar_distance_m));
      minimum = std::min(minimum, static_cast<double>(neighbor->min_m));
      maximum = std::max(maximum, static_cast<double>(neighbor->max_m));
      maximum_positive_rise = std::max(
          maximum_positive_rise,
          static_cast<double>(neighbor->max_m) - center_mid);
    }
  }
  return LocalTerrainMeasurements{
      .center_known = true,
      .neighborhood_complete = complete,
      .slope_rad = maximum_slope,
      .relief_m = maximum - minimum,
      .positive_rise_m = std::max(0.0, maximum_positive_rise),
  };
}

namespace {

template <typename Capability>
IntrinsicTraversalEvaluation EvaluateTerrainPatch(
    const ElevationRangeView& elevation, GridIndex center,
    const Capability& capability, double half_width_m,
    map::TerrainObservationEstimator* shared_observations = nullptr) {
  if (const auto measured = elevation.TerrainMeasurementsAt(center)) {
    if (!measured->center_known) return {};
    const double relief_limit = [&] {
      if constexpr (std::is_same_v<Capability, LeggedCapability>)
        return capability.maximum_step_height_m;
      else return capability.maximum_local_obstacle_relief_m;
    }();
    bool blocked = measured->slope_rad > capability.maximum_slope_rad ||
                   measured->relief_m > relief_limit;
    if constexpr (std::is_same_v<Capability, WheeledCapability>)
      blocked |= measured->positive_rise_m > capability.minimum_underbody_clearance_m;
    return {.state = blocked ? IntrinsicCellState::kBlocked :
                      (measured->neighborhood_complete ? IntrinsicCellState::kFree :
                                                        IntrinsicCellState::kUnknown),
            .slope_rad = measured->slope_rad,
            .relief_m = measured->relief_m,
            .confidence = blocked || measured->neighborhood_complete ? 1. : 0.};
  }
  map::TerrainObservationEstimator local_observations(elevation);
  auto& observations = shared_observations ? *shared_observations : local_observations;
  constexpr bool legged = std::is_same_v<Capability, LeggedCapability>;
  auto patch = map::MeasureTerrainPatch(elevation, center, half_width_m, observations);
  if (!patch) return {};
  auto& points = patch->points;
  const auto observed_count = patch->observed_count;
  const double slope = legged ? patch->median_slope_rad : patch->refined_slope_rad;
  const double low = patch->low, high = patch->high;
  const double relief = high - low;
  // Underbody rise needs a center reference; raw local relief does not.
  const double rise = patch->center_available
      ? std::max(0., high - patch->center_residual) : 0.;
  const double confidence = patch->confidence;
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
  if (discontinuity || (legged && relief > .0125 && !patch->continuous_ground)) {
    // Meaningful legged relief requires two supported surfaces even below the
    // step limit. An isolated tall/low return is UNKNOWN, never free landing.
    const double band = legged ? std::min(.0125, .25 * relief_limit)
                               : .25 * std::min(relief_limit, rise_limit);
    bool low_supported = false, high_supported = false;
    // A larger plane fit must not enlarge the local support certificate.
    std::array<std::size_t, 25> support{};
    std::size_t support_count = 0;
    for (std::size_t i = 0; i < observed_count; ++i)
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
  const bool blocked = discontinuity || slope > capability.maximum_slope_rad;
  // Missing returns cannot erase a supported obstacle. Completeness gates
  // FREE only, after the unchanged raw high/low support checks above.
  if (!blocked && (!patch->center_available || (legged && !patch->central_complete &&
          (!patch->continuous_ground || elevation.geometry().resolution_m() > .10 + 1e-12))))
    return {.state = IntrinsicCellState::kUnknown,
            .slope_rad = slope, .relief_m = relief};
  return {.state = blocked ? IntrinsicCellState::kBlocked : IntrinsicCellState::kFree,
          .slope_rad = slope,
          .relief_m = relief,
          .confidence = confidence};
}

class WheelElevationEvaluator final : public PlatformElevationEvaluator {
 public:
  explicit WheelElevationEvaluator(WheeledCapability capability, double half_width_m,
                                   map::TerrainObservationEstimator* observations = nullptr)
      : capability_(std::move(capability)), half_width_m_(half_width_m), observations_(observations) {
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
    return EvaluateTerrainPatch(elevation, index, capability_, half_width_m_, observations_);
  }

 private:
  WheeledCapability capability_;
  double half_width_m_{};
  map::TerrainObservationEstimator* observations_{};
};

class LeggedElevationEvaluator final : public PlatformElevationEvaluator {
 public:
  explicit LeggedElevationEvaluator(LeggedCapability capability)
      : capability_(std::move(capability)) {
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
    const LocalTerrainMeasurements measured =
        MeasureLocalTerrain(elevation, index);
    if (!measured.center_known) {
      return {};
    }
    const bool exceeds_physical_limit =
        measured.slope_rad > capability_.maximum_slope_rad ||
        measured.relief_m > capability_.maximum_step_height_m;
    if (exceeds_physical_limit) {
      return IntrinsicTraversalEvaluation{
          .state = IntrinsicCellState::kBlocked,
          .slope_rad = measured.slope_rad,
          .relief_m = measured.relief_m,
          .confidence = 1.,
      };
    }
    return IntrinsicTraversalEvaluation{
        .state = measured.neighborhood_complete
                     ? IntrinsicCellState::kFree
                     : IntrinsicCellState::kUnknown,
        .slope_rad = measured.slope_rad,
        .relief_m = measured.relief_m,
        .confidence = measured.neighborhood_complete ? 1. : 0.,
    };
  }

 private:
  LeggedCapability capability_;
};

}  // namespace

int TerrainBackgroundRadiusCells(double half_width_m, double resolution_m) {
  return map::ObservationBackgroundRadiusCells(half_width_m, resolution_m);
}

namespace {
std::unique_ptr<const PlatformElevationEvaluator>
MakeEvaluatorWithObservations(const PlatformCapability& capability,
                               const double background_half_width_m,
                               map::TerrainObservationEstimator* observations) {
  if (!FiniteNonnegative(background_half_width_m))
    throw std::invalid_argument("terrain background width must be finite and nonnegative");
  return std::visit(
      [background_half_width_m, observations](const auto& typed_capability)
          -> std::unique_ptr<const PlatformElevationEvaluator> {
        using Capability = std::decay_t<decltype(typed_capability)>;
        if constexpr (std::is_same_v<Capability, WheeledCapability>) {
          return std::make_unique<const WheelElevationEvaluator>(
              typed_capability, background_half_width_m, observations);
        } else if constexpr (std::is_same_v<Capability, LeggedCapability>) {
          return std::make_unique<const LeggedElevationEvaluator>(
              typed_capability);
        } else {
          throw std::invalid_argument(
              "fine traversability supports wheel or legged capability");
        }
      },
      capability);
}

}  // namespace

std::unique_ptr<const PlatformElevationEvaluator>
MakePlatformElevationEvaluator(const PlatformCapability& capability, double half_width_m) {
  return MakeEvaluatorWithObservations(capability, half_width_m, nullptr);
}

namespace {
// 有界 dense 查询缓存绑定不可变 snapshot；大域仍委托原 view。
class BatchElevationView final : public ElevationRangeView, public map::UniformPatchProvider {
 public:
  explicit BatchElevationView(const ElevationRangeView& source) : source_(source) {
    const auto& g=source.geometry();
    width_=g.width(); height_=g.height(); minimum_=g.min_inclusive();
    if (!width_ || !height_ || width_ > 4000000/height_) return;
    ranges_.resize(width_*height_);
    for (std::size_t y=0;y<height_;++y) for (std::size_t x=0;x<width_;++x) {
      auto value=source.ElevationRangeAt({minimum_.x+static_cast<std::int64_t>(x),minimum_.y+static_cast<std::int64_t>(y)});
      ranges_[y*width_+x]=value;
      if (!reference_ && value && value->min_m==value->max_m) reference_=value->min_m;
    }
    PreparePlaneCertificate();
    // 完整行/列逐值相同是重复窗口的精确证据，包含缺测与区间端点。
    repeated_rows_=true;repeated_columns_=true;
    for (std::size_t y=0;y<height_;++y) for (std::size_t x=0;x<width_;++x) {
      repeated_rows_ &= ranges_[y*width_+x]==ranges_[x];
      repeated_columns_ &= ranges_[y*width_+x]==ranges_[y*width_];
    }
    if (!reference_) return;
    prefix_.resize((width_+1)*(height_+1));
    for (std::size_t y=0;y<height_;++y) {
      std::uint32_t row=0;
      for (std::size_t x=0;x<width_;++x) {
        const auto& v=ranges_[y*width_+x];
        row+=!v || v->min_m!=*reference_ || v->max_m!=*reference_;
        prefix_[(y+1)*(width_+1)+x+1]=prefix_[y*(width_+1)+x+1]+row;
      }
    }
  }
  const SparseGridGeometry& geometry() const noexcept override {return source_.geometry();}
  std::optional<ElevationRange> ElevationRangeAt(GridIndex i) const noexcept override {
    if (ranges_.empty()) return source_.ElevationRangeAt(i);
    if (!geometry().Contains(i)) return {};
    return ranges_[static_cast<std::size_t>(i.y-minimum_.y)*width_+static_cast<std::size_t>(i.x-minimum_.x)];
  }
  std::optional<LocalTerrainMeasurements> TerrainMeasurementsAt(GridIndex i) const noexcept override {
    return source_.TerrainMeasurementsAt(i);
  }
  std::optional<map::UniformPatchKey> UniformPatch(GridIndex i,int r) const override {
    if (ranges_.empty()) return {};
    const auto x=i.x-minimum_.x,y=i.y-minimum_.y;
    const auto l=std::max<std::int64_t>(0,x-r),b=std::max<std::int64_t>(0,y-r);
    const auto h=std::min<std::int64_t>(width_,x+r+1),t=std::min<std::int64_t>(height_,y+r+1);
    if (l>=x || b>=y || h<=x+1 || t<=y+1) return {};
    const auto at=[&](std::int64_t xx,std::int64_t yy){return prefix_[yy*(width_+1)+xx];};
    int kind=0;std::int64_t identity=0;
    if (!prefix_.empty() && at(h,t)+at(l,b)==at(l,t)+at(h,b)) {
      identity=std::bit_cast<std::uint32_t>(*reference_);
    } else if (repeated_rows_) {kind=1;identity=x;}
    else if (repeated_columns_) {kind=2;identity=y;}
    else return {};
    return map::UniformPatchKey{static_cast<int>(l-x),static_cast<int>(h-x),
        static_cast<int>(b-y),static_cast<int>(t-y),kind,identity};
  }
  bool CertifiesFree(GridIndex i,const WheeledCapability& wheel) const {
    if (!plane_ || !geometry().Contains(i) || source_.TerrainMeasurementsAt(i)) return false;
    const auto x=i.x-minimum_.x,y=i.y-minimum_.y;
    if (x<1 || y<1 || x+1>=static_cast<std::int64_t>(width_) || y+1>=static_cast<std::int64_t>(height_)) return false;
    const double r=geometry().resolution_m(),delta=(*plane_)[2];
    const auto halo=TerrainIntrinsicDependencyRadiusCells(r);
    const double radius=halo-TerrainBackgroundRadiusCells(0.,r);
    const auto l=std::max<std::int64_t>(0,x-halo),b=std::max<std::int64_t>(0,y-halo);
    const auto h=std::min<std::int64_t>(width_,x+halo+1),t=std::min<std::int64_t>(height_,y+halo+1);
    const auto at=[&](std::int64_t xx,std::int64_t yy){return plane_bad_prefix_[yy*(width_+1)+xx];};
    if (at(h,t)+at(l,b)!=at(l,t)+at(h,b)) return false;
    // 证明见 docs/validation/2026-10-29-app-terrain-batch-proof.md。
    // 完整 clipped 矩形的 OLS 残差界同时保证原 IRLS weights 恒为1、
    // 所有inlier、无jump、center支持及confidence>=2/3；不修改原阈值。
    if ((2.+24.*radius)*delta >= .01) return false;
    const double slope_upper=std::atan(std::hypot((*plane_)[0],(*plane_)[1])+6.*std::sqrt(2.)*delta/r);
    return slope_upper < wheel.maximum_slope_rad &&
        36.*delta < wheel.maximum_local_obstacle_relief_m &&
        36.*delta < wheel.minimum_underbody_clearance_m;
  }
 private:
  void PreparePlaneCertificate() {
    // 有界证明域；其他分辨率、范围、缺测及非零宽度高程区间照旧scalar。
    const double r=geometry().resolution_m();
    if (r<.01 || r>.1 || width_<3 || height_<3 || width_>4096 || height_>4096) return;
    const auto valid=[](const auto& v){return v && v->min_m==v->max_m && std::isfinite(v->min_m) && std::abs(v->min_m)<=16.;};
    if (!valid(ranges_[0]) || !valid(ranges_[width_-1]) || !valid(ranges_[(height_-1)*width_])) return;
    const double z=ranges_[0]->min_m;
    const double a=(double(ranges_[width_-1]->min_m)-z)/((width_-1)*r);
    const double b=(double(ranges_[(height_-1)*width_]->min_m)-z)/((height_-1)*r);
    const double scale=std::max({1.,std::abs(z),std::abs(a*((width_-1)*r)),std::abs(b*((height_-1)*r))});
    // 逐端点验证本snapshot参考平面的误差包络；异常只否决依赖窗口，
    // 不能让远处一个孔洞/高返回使整幅正常坡面重新拟合。
    const double residual_bound=4e-6*scale;
    plane_bad_prefix_.resize((width_+1)*(height_+1));
    for (std::size_t y=0;y<height_;++y) {
      std::uint32_t bad=0;
      for (std::size_t x=0;x<width_;++x) {
        const auto& v=ranges_[y*width_+x];
        bad += !valid(v) || std::abs(double(v->min_m)-(z+a*(x*r)+b*(y*r)))>residual_bound;
        plane_bad_prefix_[(y+1)*(width_+1)+x+1]=plane_bad_prefix_[y*(width_+1)+x+1]+bad;
      }
    }
    // 4e-6是已验证端点残差上界，额外1e-6覆盖有界double算术；
    // 两者不是地形阈值，只决定能否证明原scalar必为FREE。
    const double delta=5e-6*scale;
    if (std::isfinite(delta) && std::isfinite(a) && std::isfinite(b)) plane_=std::array<double,3>{a,b,delta};
  }
  std::optional<std::array<double,3>> plane_;
  std::vector<std::uint32_t> plane_bad_prefix_;
  const ElevationRangeView& source_;
  std::size_t width_{},height_{}; GridIndex minimum_{};
  std::vector<std::optional<ElevationRange>> ranges_;
  std::optional<float> reference_;
  bool repeated_rows_{},repeated_columns_{};
  std::vector<std::uint32_t> prefix_;
};
class SnapshotElevationEvaluator final : public PlatformElevationEvaluator {
 public:
  SnapshotElevationEvaluator(const ElevationRangeView& source,const PlatformCapability& capability)
      :source_(source),view_(source),observations_(view_),
       evaluator_(MakeEvaluatorWithObservations(capability,0.,&observations_)) {}
  bool CertifiesFree(GridIndex i,const WheeledCapability& capability) const {return view_.CertifiesFree(i,capability);}
  IntrinsicTraversalEvaluation Evaluate(const ElevationRangeView& source,GridIndex i) const override {
    if (&source!=&source_) throw std::invalid_argument("batch evaluator snapshot identity mismatch");
    return evaluator_->Evaluate(view_,i);
  }
 private:
  const ElevationRangeView& source_;
  BatchElevationView view_;
  mutable map::TerrainObservationEstimator observations_;
  std::unique_ptr<const PlatformElevationEvaluator> evaluator_;
};
}
std::unique_ptr<const PlatformElevationEvaluator> MakeSnapshotElevationEvaluator(
    const ElevationRangeView& view,const PlatformCapability& capability) {
  return std::make_unique<SnapshotElevationEvaluator>(view,capability);
}

struct SnapshotTerrainClassifier::Impl final {
  Impl(const ElevationRangeView& source,const PlatformCapability& capability)
      : source(source),evaluator(source,capability) {
    if (const auto* value=std::get_if<WheeledCapability>(&capability)) wheel=*value;
  }
  const ElevationRangeView& source;
  SnapshotElevationEvaluator evaluator;
  std::optional<WheeledCapability> wheel;
  std::size_t certified{};
};
SnapshotTerrainClassifier::SnapshotTerrainClassifier(const ElevationRangeView& v,const PlatformCapability& c)
    :impl_(std::make_unique<Impl>(v,c)) {}
SnapshotTerrainClassifier::~SnapshotTerrainClassifier()=default;
IntrinsicCellState SnapshotTerrainClassifier::Classify(GridIndex i) {
  if (impl_->wheel && impl_->evaluator.CertifiesFree(i,*impl_->wheel)) {
    ++impl_->certified;return IntrinsicCellState::kFree;
  }
  return impl_->evaluator.Evaluate(impl_->source,i).state;
}
std::size_t SnapshotTerrainClassifier::certified_free_cells() const noexcept {return impl_->certified;}

struct FineCellEvaluator::Impl final {
  struct IntrinsicCacheTile final {
    std::array<IntrinsicTraversalEvaluation, kGridTileCellCount> evaluations;
    std::bitset<kGridTileCellCount> populated;
    // Lazy 16-cell row summaries; scoped to this immutable elevation view.
    std::array<std::uint16_t, kGridTileCellCount / 16> hazards{};
    std::array<std::uint16_t, kGridTileCellCount / 16> blocked{};
    std::array<std::uint16_t, kGridTileCellCount / 16> inferred{};
    std::bitset<kGridTileCellCount / 16> summarized;
  };

  Impl(const ElevationRangeView& elevation_value,
       PlatformCapability capability_value,
       TraversabilityProfile profile_value,
       std::unique_ptr<const PlatformElevationEvaluator> intrinsic,
       bool use_observations = false)
      : elevation(elevation_value),
        capability(std::move(capability_value)),
        profile(std::move(profile_value)),
        observations(elevation),
        intrinsic_evaluator(use_observations ?
            MakeEvaluatorWithObservations(
                capability, profile.terrain_background_half_width_m, &observations) :
            std::move(intrinsic)),
        hard_radius_m(CircumscribedRadius(profile.planar_envelope_xy_m)) {
    if (!intrinsic_evaluator ||
        !FiniteNonnegative(profile.preferred_clearance_m) ||
        !FiniteNonnegative(profile.slope_weight) ||
        !FiniteNonnegative(profile.relief_weight) ||
        !FiniteNonnegative(profile.clearance_weight) ||
        !FiniteNonnegative(profile.confidence_weight)) {
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
                    bool blocked_only = false, bool inferred_only = false) {
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
          std::uint16_t mask = 0, blocked = 0, inferred = 0;
          const auto base = x - static_cast<std::int64_t>(lane);
          for (int i = 0; i < 16; ++i) {
            const GridIndex cell{base + i, y};
            if (!elevation.geometry().Contains(cell)) continue;
            const auto& evaluation = IntrinsicAt(cell);
            const auto state = evaluation.state;
            if (state == IntrinsicCellState::kFree && evaluation.confidence < 1.)
              inferred |= std::uint16_t(1U << i);
            if (state != IntrinsicCellState::kFree)
              mask |= std::uint16_t(1U << i);
            if (state == IntrinsicCellState::kBlocked)
              blocked |= std::uint16_t(1U << i);
          }
          tile->hazards[block] = mask;
          tile->blocked[block] = blocked;
          tile->inferred[block] = inferred;
          tile->summarized.set(block);
        }
        const unsigned candidates = inferred_only ? tile->inferred[block] :
            (blocked_only ? tile->blocked[block] : tile->hazards[block]);
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
    if (!clear) return {.state = FineCellState::kBlocked, .confidence = 1.};
    if (!intersects_unknown) {
      intersects_unknown = !VisitHazards(minimum, maximum, [&](GridIndex seed) {
        return !CircleIntersectsCellArea(center, hard_radius_m, geometry, seed);
      });
    }
    if (intersects_unknown) {
      return {.state = FineCellState::kUnknown};
    }

    double confidence = 1.;
    VisitHazards(minimum, maximum, [&](GridIndex seed) {
      if (CircleIntersectsCellArea(center, hard_radius_m, geometry, seed))
        confidence = std::min(confidence, IntrinsicAt(seed).confidence);
      return true;
    }, false, true);
    const IntrinsicTraversalEvaluation& intrinsic = IntrinsicAt(candidate);
    const double slope_cost = SaturatingProduct(
        profile.slope_weight,
        NormalizedSquared(intrinsic.slope_rad, SlopeLimit(capability)));
    const double relief_cost = SaturatingProduct(
        profile.relief_weight,
        NormalizedSquared(intrinsic.relief_m, ReliefLimit(capability)));
    double traversal_cost = SaturatingSum(
        SaturatingSum(slope_cost, relief_cost),
        SaturatingProduct(profile.confidence_weight, 1. - confidence));
    if (profile.clearance_weight == 0.0 ||
        profile.preferred_clearance_m == 0.0) {
      return {.state = FineCellState::kFree,
              .traversal_cost = traversal_cost, .confidence = confidence};
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
            .traversal_cost = traversal_cost, .confidence = confidence};
  }

  const ElevationRangeView& elevation;
  PlatformCapability capability;
  TraversabilityProfile profile;
  map::TerrainObservationEstimator observations;
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
    : impl_(std::make_unique<Impl>(elevation, capability, profile, nullptr, true)) {}

FineCellEvaluator::FineCellEvaluator(
    const ElevationRangeView& elevation,
    const PlatformCapability& capability,
    const TraversabilityProfile& profile,
    std::unique_ptr<const PlatformElevationEvaluator> intrinsic)
    : impl_(std::make_unique<Impl>(elevation, capability, profile,
                                   std::move(intrinsic))) {}

FineCellEvaluator::~FineCellEvaluator() = default;
FineCellEvaluator::FineCellEvaluator(FineCellEvaluator&&) noexcept = default;
FineCellEvaluator& FineCellEvaluator::operator=(FineCellEvaluator&&) noexcept =
    default;

FineCellEvaluation FineCellEvaluator::Evaluate(const GridIndex index) {
  return impl_->Evaluate(index);
}

std::optional<float> FineCellEvaluator::GroundHeight(GridIndex index) {
  const auto& ground = impl_->observations.Estimate(index);
  return ground ? std::optional<float>(static_cast<float>(ground->surface_height_m)) : std::nullopt;
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

namespace lunar::incremental_navigation {
std::size_t FineCellEvaluator::evaluated_ground_cells() const noexcept {
  return impl_->observations.evaluated_cells();
}
std::size_t FineCellEvaluator::ground_cache_bytes() const noexcept {
  return impl_->observations.allocated_bytes();
}
}
