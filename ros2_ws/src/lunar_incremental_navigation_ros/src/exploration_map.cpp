#include "lunar_incremental_navigation_core/platform_terrain_window.hpp"
#include "lunar_incremental_navigation_core/convex_polygon.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include "lunar_incremental_navigation_ros/exploration_map.hpp"

namespace lunar::incremental_navigation_ros {
namespace core = lunar::incremental_navigation;
namespace {
using Polygon = std::vector<core::Point2>;
using Values = std::map<core::GridIndex, core::IntrinsicTraversalEvaluation>;
using core::convex_polygon::Cross;
using core::convex_polygon::Area;
using core::convex_polygon::Intersect;
using core::convex_polygon::Subtract;
Polygon Square(core::GridIndex i, double r) {
  const double x = i.x * r, y = i.y * r;
  return {{x, y}, {x + r, y}, {x + r, y + r}, {x, y + r}};
}
struct Fragment {
  Polygon polygon;
  core::IntrinsicTraversalEvaluation value;
  float height;
};
class SourceView final : public core::ElevationRangeView {
 public:
  explicit SourceView(const OwnedElevationEvidence& source)
      : source_(source),
        geometry_(source.geometry.frame_id, source.geometry.resolution_m,
                  source.geometry.origin_m, {0, 0},
                  {static_cast<std::int64_t>(source.geometry.width),
                   static_cast<std::int64_t>(source.geometry.height)}) {}
  const core::SparseGridGeometry& geometry() const noexcept override {
    return geometry_;
  }
  std::optional<core::ElevationRange> ElevationRangeAt(
      core::GridIndex i) const noexcept override {
    if (!geometry_.Contains(i)) return std::nullopt;
    const auto h = source_.elevation_m[i.y * source_.geometry.width + i.x];
    if (!std::isfinite(h)) return std::nullopt;
    return core::ElevationRange{h, h};
  }
  std::optional<core::LocalTerrainMeasurements> TerrainMeasurementsAt(
      core::GridIndex i) const noexcept override {
    if (!geometry_.Contains(i) || source_.terrain_measurements.empty())
      return std::nullopt;
    const auto m =
        source_.terrain_measurements[i.y * source_.geometry.width + i.x];
    return m.center_known ? std::optional(m) : std::nullopt;
  }

 private:
  const OwnedElevationEvidence& source_;
  core::SparseGridGeometry geometry_;
};
class ProjectedEvaluator final : public core::PlatformElevationEvaluator {
 public:
  explicit ProjectedEvaluator(std::shared_ptr<const Values> values)
      : values_(std::move(values)) {}
  core::IntrinsicTraversalEvaluation Evaluate(
      const core::ElevationRangeView& view, core::GridIndex i) const override {
    i.x += static_cast<std::int64_t>(std::llround(
        view.geometry().origin_m().x / view.geometry().resolution_m()));
    i.y += static_cast<std::int64_t>(std::llround(
        view.geometry().origin_m().y / view.geometry().resolution_m()));
    const auto it = values_->find(i);
    return it == values_->end() ? core::IntrinsicTraversalEvaluation{}
                                : it->second;
  }

 private:
  std::shared_ptr<const Values> values_;
};
core::Vec3 Transform(const core::RigidTransform& t, core::Vec3 p) {
  auto q = t.rotation;
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
  const core::Vec3 c{2 * (q.y * p.z - q.z * p.y), 2 * (q.z * p.x - q.x * p.z),
                     2 * (q.x * p.y - q.y * p.x)};
  return {p.x + q.w * c.x + q.y * c.z - q.z * c.y + t.translation_m.x,
          p.y + q.w * c.y + q.z * c.x - q.x * c.z + t.translation_m.y,
          p.z + q.w * c.z + q.x * c.y - q.y * c.x + t.translation_m.z};
}
bool Same(const core::IntrinsicTraversalEvaluation& a,
          const core::IntrinsicTraversalEvaluation& b) {
  return a.state == b.state && a.slope_rad == b.slope_rad &&
         a.relief_m == b.relief_m;
}
}  // namespace
struct ExplorationMap::Impl {
  core::PlatformCapability capability;
  core::TraversabilityProfile profile;
  double resolution, coarse;
  bool auto_resolution, auto_coarse;
  std::uint64_t geometry_revision{}, last_fine_revision{};
  std::string source_epoch, source_frame;
  std::uint64_t source_raw_revision{};
  double source_resolution{};
  std::size_t facts_aabb_checks{}, facts_overlap_candidates{};
  std::map<core::GridIndex, std::vector<Fragment>> history;
  std::shared_ptr<const Values> values = std::make_shared<const Values>();
  core::PersistentElevationMap elevation;
  std::shared_ptr<const core::FineTraversabilitySnapshot> fine;
  std::int64_t stamp{};
  std::deque<std::pair<std::uint64_t, std::vector<core::TileIndex>>> journal;
  Impl(core::PlatformCapability c, core::TraversabilityProfile p, double r,
       double coarse_value)
      : capability(std::move(c)),
        profile(std::move(p)),
        resolution(r),
        coarse(coarse_value),
        auto_resolution(r == 0.), auto_coarse(coarse_value == 0.) {
    if (!std::isfinite(r) || r < 0 || !std::isfinite(coarse) || coarse < 0)
      throw std::invalid_argument("invalid map resolution");
  }
};
ExplorationMap::ExplorationMap(core::PlatformCapability c,
                               core::TraversabilityProfile p, double r,
                               double coarse)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(p), r, coarse)) {}
ExplorationMap::~ExplorationMap() = default;

bool ExplorationMap::Apply(const OwnedElevationEvidence& source,
                           std::int64_t stamp) {
  auto& m = *impl_;
  const auto& g = source.geometry;
  // 原生 Apply 仅用于输入验证及非水平变换的保守物理辅助，绝不复用其源分辨率锁。
  core::PersistentElevationMap validation;
  auto geometry_validation = source;
  geometry_validation.terrain_measurements.clear();
  if (validation.Apply(geometry_validation.View()).status ==
      core::ElevationUpdateResult::Status::kRejected)
    return false;
  if (!source.terrain_measurements.empty()) {
    core::PersistentElevationMap measurements;
    auto native = source;
    native.map_from_source = {.parent_frame = g.frame_id,
                              .child_frame = g.frame_id};
    if (measurements.Apply(native.View()).status ==
        core::ElevationUpdateResult::Status::kRejected)
      return false;
  }
  if (m.fine &&
      source.map_from_source.parent_frame != m.fine->geometry().frame_id())
    return false;
  const double target_resolution = m.auto_resolution ? g.resolution_m : m.resolution;
  // 在修改历史格网之前完成目标坐标范围检查，拒绝输入不能破坏旧快照。
  const double coordinate_limit =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) - 4096.;
  const auto representable = [&](core::Point2 p) {
    return std::isfinite(p.x / target_resolution) &&
           std::isfinite(p.y / target_resolution) &&
           std::abs(p.x / target_resolution) < coordinate_limit &&
           std::abs(p.y / target_resolution) < coordinate_limit;
  };
  if (m.resolution != target_resolution)
    for (const auto& [index, fragments] : m.history)
      for (const auto& f : fragments)
        for (auto p : f.polygon)
          if (!representable(p)) return false;
  float min_height = std::numeric_limits<float>::infinity(), max_height = -min_height;
  for (auto h : source.elevation_m)
    if (std::isfinite(h)) {
      min_height = std::min(min_height, h); max_height = std::max(max_height, h);
    }
  if (std::isfinite(min_height))
    for (double x : {0., static_cast<double>(g.width)})
      for (double y : {0., static_cast<double>(g.height)})
        for (double h : {static_cast<double>(min_height), static_cast<double>(max_height)}) {
          auto p = Transform(source.map_from_source,
              {g.origin_m.x + x * g.resolution_m,
               g.origin_m.y + y * g.resolution_m, g.origin_m.z + h});
          if (!representable({p.x, p.y})) return false;
        }
  const bool migrated = m.resolution != target_resolution;
  if (migrated) {
    // 只裁分世界坐标证据足迹；历史分类仍属于实际观测的源格。
    std::map<core::GridIndex, std::vector<Fragment>> rebinned;
    const double epsilon = target_resolution * target_resolution * 1e-10;
    for (const auto& [index, fragments] : m.history)
      for (const auto& f : fragments) {
        double minx = f.polygon[0].x, maxx = minx;
        double miny = f.polygon[0].y, maxy = miny;
        for (auto p : f.polygon) {
          minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
          miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        }
        for (auto y = static_cast<std::int64_t>(std::floor(miny / target_resolution));
             y < static_cast<std::int64_t>(std::ceil(maxy / target_resolution)); ++y)
          for (auto x = static_cast<std::int64_t>(std::floor(minx / target_resolution));
               x < static_cast<std::int64_t>(std::ceil(maxx / target_resolution)); ++x) {
            auto polygon = Intersect(f.polygon, Square({x,y}, target_resolution));
            if (Area(polygon) > epsilon)
              rebinned[{x,y}].push_back({std::move(polygon), f.value, f.height});
          }
      }
    m.history = std::move(rebinned);
    m.resolution = target_resolution;
    m.values = std::make_shared<const Values>();
    m.elevation = core::PersistentElevationMap{};
    m.journal.clear();
  }
  if (m.auto_coarse) m.coarse = m.resolution;
  SourceView view(source);
  const auto evaluator = core::MakePlatformWindowElevationEvaluator(view,m.capability);
  const double cell_area = m.resolution * m.resolution,
               epsilon = cell_area * 1e-10;
  std::map<core::GridIndex, std::vector<Fragment>> incoming;
  const auto q = source.map_from_source.rotation;
  const bool upright = std::abs(q.x) + std::abs(q.y) < 1e-9;
  for (std::size_t y = 0; y < g.height; ++y)
    for (std::size_t x = 0; x < g.width; ++x) {
      const auto h = source.elevation_m[y * g.width + x];
      if (!std::isfinite(h)) continue;
      auto classified = evaluator->Evaluate(
          view, {static_cast<std::int64_t>(x), static_cast<std::int64_t>(y)});
      // 倾斜坐标系的源格标量不能冒充世界水平邻域；未知保持未知。
      if (!upright && classified.state == core::IntrinsicCellState::kFree)
        classified.state = core::IntrinsicCellState::kUnknown;
      Polygon polygon;
      float height = 0;
      for (const auto d :
           std::vector<core::Point2>{{0, 0}, {1, 0}, {1, 1}, {0, 1}}) {
        auto p = Transform(
            source.map_from_source,
            {g.origin_m.x + (x + d.x) * g.resolution_m,
             g.origin_m.y + (y + d.y) * g.resolution_m, g.origin_m.z + h});
        polygon.push_back({p.x, p.y});
        height += static_cast<float>(p.z * .25);
      }
      const auto area = Area(polygon);
      if (!std::isfinite(area) || area <= epsilon) continue;
      double signed_area = 0.;
      for (std::size_t i = 0; i < polygon.size(); ++i)
        signed_area +=
            Cross({0, 0}, polygon[i], polygon[(i + 1) % polygon.size()]);
      if (signed_area < 0) std::reverse(polygon.begin(), polygon.end());
      double minx = polygon[0].x, maxx = minx, miny = polygon[0].y, maxy = miny;
      for (auto p : polygon) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
      }
      const double coordinate_limit =
          static_cast<double>(std::numeric_limits<std::int64_t>::max()) - 4096.;
      if (!std::isfinite(minx / m.resolution) ||
          !std::isfinite(maxx / m.resolution) ||
          !std::isfinite(miny / m.resolution) ||
          !std::isfinite(maxy / m.resolution) ||
          std::max(
              {std::abs(minx / m.resolution), std::abs(maxx / m.resolution),
               std::abs(miny / m.resolution), std::abs(maxy / m.resolution)}) >=
              coordinate_limit)
        return false;
      for (auto iy = static_cast<std::int64_t>(std::floor(miny / m.resolution));
           iy < static_cast<std::int64_t>(std::ceil(maxy / m.resolution)); ++iy)
        for (auto ix =
                 static_cast<std::int64_t>(std::floor(minx / m.resolution));
             ix < static_cast<std::int64_t>(std::ceil(maxx / m.resolution));
             ++ix) {
          const core::GridIndex i{ix, iy};
          auto clipped = Intersect(polygon, Square(i, m.resolution));
          if (Area(clipped) > epsilon)
            incoming[i].push_back({std::move(clipped), classified, height});
        }
    }
  if (migrated)
    for (const auto& [index, fragments] : m.history) incoming.try_emplace(index);
  auto values = std::make_shared<Values>(*m.values);
  bool changed = false;
  std::set<core::TileIndex> dirty;
  std::map<core::GridIndex, float> heights;
  for (auto& [index, fragments] : incoming) {
    double incoming_area = 0.;
    for (const auto& f : fragments) incoming_area += Area(f.polygon);
    auto& old = m.history[index];
    // 完整覆盖替换旧源格足迹；部分覆盖只裁掉真正收到的有限观测。
    if (upright && incoming_area >= cell_area - epsilon)
      old.clear();
    else
      for (const auto& f : fragments) {
        std::vector<Fragment> retained;
        for (const auto& previous : old)
          for (auto polygon : Subtract(previous.polygon, f.polygon, epsilon))
            retained.push_back(
                {std::move(polygon), previous.value, previous.height});
        old = std::move(retained);
      }
    old.insert(old.end(), std::make_move_iterator(fragments.begin()),
               std::make_move_iterator(fragments.end()));
    core::IntrinsicTraversalEvaluation value{
        .state = core::IntrinsicCellState::kFree};
    double area = 0., height = 0.;
    for (const auto& f : old) {
      const auto a = Area(f.polygon);
      area += a;
      height += a * f.height;
      value.slope_rad = std::max(value.slope_rad, f.value.slope_rad);
      value.relief_m = std::max(value.relief_m, f.value.relief_m);
      if (f.value.state == core::IntrinsicCellState::kBlocked)
        value.state = core::IntrinsicCellState::kBlocked;
      else if (f.value.state == core::IntrinsicCellState::kUnknown &&
               value.state != core::IntrinsicCellState::kBlocked)
        value.state = core::IntrinsicCellState::kUnknown;
    }
    if (area < cell_area - epsilon &&
        value.state != core::IntrinsicCellState::kBlocked)
      value.state = core::IntrinsicCellState::kUnknown;
    const auto previous = values->find(index);
    if (previous == values->end() || !Same(previous->second, value)) {
      changed = true;
      dirty.insert(core::TileForCell(index));
    }
    (*values)[index] = value;
    if (area > epsilon) heights[index] = static_cast<float>(height / area);
  }
  // 高程仅作独立显示/传输字段；物理分类由上述源格证据提供。
  if (!heights.empty()) {
    auto minimum = heights.begin()->first, maximum = minimum;
    for (const auto& [i, h] : heights) {
      minimum.x = std::min(minimum.x, i.x);
      minimum.y = std::min(minimum.y, i.y);
      maximum.x = std::max(maximum.x, i.x);
      maximum.y = std::max(maximum.y, i.y);
    }
    const std::size_t width = maximum.x - minimum.x + 1,
                      height = maximum.y - minimum.y + 1;
    std::vector<float> h(width * height,
                         std::numeric_limits<float>::quiet_NaN());
    for (const auto& [i, v] : heights)
      h[(i.y - minimum.y) * width + i.x - minimum.x] = v;
    const core::GridGeometry canonical{
        .frame_id = source.map_from_source.parent_frame,
        .width = width,
        .height = height,
        .resolution_m = m.resolution,
        .origin_m = {minimum.x * m.resolution, minimum.y * m.resolution, 0.}};
    const auto result = m.elevation.Apply(
        {.geometry = canonical,
         .elevation_m = h,
         .map_from_source = {.parent_frame = canonical.frame_id,
                             .child_frame = canonical.frame_id}});
    if (result.status == core::ElevationUpdateResult::Status::kApplied)
      changed = true;
  }
  m.stamp = stamp;
  m.values = std::move(values);
  if (!changed) return true;
  auto next = core::FineTraversabilityBuilder{}.DeriveProjected(
      m.elevation.Snapshot(), m.capability, m.profile,
      std::make_unique<const ProjectedEvaluator>(m.values), migrated ? nullptr : m.fine);
  if (migrated && m.fine) {
    // 新格网有独立高程lineage，公共revision保持单调；旧revision不可请求delta。
    next = std::make_shared<const core::FineTraversabilitySnapshot>(
        next->geometry(), next->raw_elevation_revision(),
        m.fine->fine_traversability_revision() + 1,
        next->platform_profile_hash(), next->hard_inflation_radius_m(),
        next->preferred_clearance_m(), next->cost_weights(), next->elevation(),
        next->tile_directory(),
        std::vector<core::TileIndex>(next->changed_tiles().begin(), next->changed_tiles().end()),
        std::vector<core::TileIndex>(next->changed_halo_tiles().begin(), next->changed_halo_tiles().end()),
        next->metrics());
  }
  // 全量派生的脏tile并集可安全重放；不假设原始高程revision等同分类revision。
  dirty.clear();
  for (auto tile : next->tile_indices()) dirty.insert(tile);
  for (auto tile : next->elevation()->tile_indices()) dirty.insert(tile);
  if (m.fine)
    for (auto tile : m.fine->tile_indices()) dirty.insert(tile);
  m.fine = std::move(next);
  m.last_fine_revision = m.fine->fine_traversability_revision();
  if (migrated) m.geometry_revision = m.fine->fine_traversability_revision();
  m.journal.push_back(
      {m.fine->fine_traversability_revision(), {dirty.begin(), dirty.end()}});
  if (m.journal.size() > 128) m.journal.pop_front();
  return true;
}
bool ExplorationMap::ApplyFacts(const ExplorationMapFacts& facts, std::int64_t stamp) {
  const auto& previous = *impl_;
  if (!facts.complete_snapshot || facts.frame.empty() || facts.epoch.empty() ||
      !std::isfinite(facts.source_resolution_m) || facts.source_resolution_m <= 0.)
    return false;
  if (previous.source_epoch == facts.epoch) {
    if (facts.frame != previous.source_frame || facts.raw_revision < previous.source_raw_revision)
      return false;
    if (facts.raw_revision == previous.source_raw_revision) {
      if (facts.source_resolution_m != previous.source_resolution) return false;
      // 提交者保证epoch+raw_revision标识不可变的完整事实；重复只刷新处理时间。
      impl_->stamp = stamp;
      return true;
    }
  }
  auto replacement = std::make_unique<Impl>(previous.capability, previous.profile,
      previous.auto_resolution ? 0. : previous.resolution,
      previous.auto_coarse ? 0. : previous.coarse);
  auto& m = *replacement;
  m.resolution = m.auto_resolution ? facts.source_resolution_m : m.resolution;
  if (m.auto_coarse) m.coarse = m.resolution;
  const double cell_area = m.resolution * m.resolution;
  if (!std::isfinite(cell_area) || cell_area <= 0.) return false;
  const double epsilon = cell_area * 1e-10;
  const double limit = static_cast<double>(std::numeric_limits<std::int64_t>::max()) - 4096.;
  // 全量事实在临时实例验证和派生，发布前不触碰旧快照。
  for (const auto& observation : facts.observation_fragments) {
    auto polygon = observation.polygon;
    if (polygon.size() < 3 || !std::isfinite(observation.height_m) ||
        std::isinf(observation.slope_rad) || observation.slope_rad < 0. ||
        std::isinf(observation.relief_m) || observation.relief_m < 0. ||
        (observation.state != core::IntrinsicCellState::kUnknown &&
         observation.state != core::IntrinsicCellState::kFree &&
         observation.state != core::IntrinsicCellState::kBlocked)) return false;
    double minx = polygon[0].x, maxx = minx, miny = polygon[0].y, maxy = miny;
    for (auto p : polygon) {
      if (!std::isfinite(p.x / m.resolution) || !std::isfinite(p.y / m.resolution) ||
          std::abs(p.x / m.resolution) >= limit || std::abs(p.y / m.resolution) >= limit)
        return false;
      minx = std::min(minx,p.x); maxx = std::max(maxx,p.x);
      miny = std::min(miny,p.y); maxy = std::max(maxy,p.y);
    }
    double signed_area = 0.;
    for (std::size_t i=1; i+1<polygon.size(); ++i)
      signed_area += Cross(polygon[0],polygon[i],polygon[i+1]);
    if (!std::isfinite(signed_area) || Area(polygon) <= 0.) return false;
    if (signed_area < 0.) std::reverse(polygon.begin(),polygon.end());
    for (std::size_t i=0; i<polygon.size(); ++i)
      for (auto p : polygon)
        if (Cross(polygon[i],polygon[(i+1)%polygon.size()],p) < -epsilon) return false;
    const core::IntrinsicTraversalEvaluation value{.state=observation.state,
        .slope_rad=observation.slope_rad, .relief_m=observation.relief_m};
    for (auto y=static_cast<std::int64_t>(std::floor(miny/m.resolution));
         y<static_cast<std::int64_t>(std::ceil(maxy/m.resolution)); ++y)
      for (auto x=static_cast<std::int64_t>(std::floor(minx/m.resolution));
           x<static_cast<std::int64_t>(std::ceil(maxx/m.resolution)); ++x) {
        auto clipped=Intersect(polygon,Square({x,y},m.resolution));
        if (Area(clipped)<=epsilon) continue;
        auto& fragments=m.history[{x,y}];
        fragments.push_back({std::move(clipped),value,observation.height_m});
      }
  }
  // 每格按AABB左界排序：相邻边接触不可能有面积重叠，只有双轴开区间
  // 真正相交才调用公共凸多边形裁剪。扫线不改变最终重叠面积判据。
  struct Bounds { double minx,maxx,miny,maxy; std::size_t fragment; };
  for (const auto& [index,fragments]:m.history) {
    std::vector<Bounds> bounds; bounds.reserve(fragments.size());
    for (std::size_t i=0;i<fragments.size();++i) {
      const auto& polygon=fragments[i].polygon;
      Bounds b{polygon[0].x,polygon[0].x,polygon[0].y,polygon[0].y,i};
      for (auto p:polygon) {
        b.minx=std::min(b.minx,p.x); b.maxx=std::max(b.maxx,p.x);
        b.miny=std::min(b.miny,p.y); b.maxy=std::max(b.maxy,p.y);
      }
      bounds.push_back(b);
    }
    std::sort(bounds.begin(),bounds.end(),[](const auto& a,const auto& b){return a.minx<b.minx;});
    for (std::size_t i=0;i<bounds.size();++i)
      for (std::size_t j=i+1;j<bounds.size() && bounds[j].minx<bounds[i].maxx;++j) {
        ++m.facts_aabb_checks;
        if (bounds[j].maxy<=bounds[i].miny || bounds[i].maxy<=bounds[j].miny) continue;
        ++m.facts_overlap_candidates;
        if (Area(Intersect(fragments[bounds[i].fragment].polygon,
                           fragments[bounds[j].fragment].polygon))>epsilon) return false;
      }
  }
  auto values=std::make_shared<Values>();
  std::map<core::TileIndex,std::vector<std::pair<core::GridIndex,float>>> tiles;
  for (const auto& [index,fragments]:m.history) {
    core::IntrinsicTraversalEvaluation value{.state=core::IntrinsicCellState::kFree};
    double area=0., height=0.;
    for (const auto& f:fragments) {
      const auto a=Area(f.polygon); area+=a; height+=a*f.height;
      value.slope_rad=(std::isnan(value.slope_rad) || std::isnan(f.value.slope_rad))
          ? std::numeric_limits<double>::quiet_NaN() : std::max(value.slope_rad,f.value.slope_rad);
      value.relief_m=(std::isnan(value.relief_m) || std::isnan(f.value.relief_m))
          ? std::numeric_limits<double>::quiet_NaN() : std::max(value.relief_m,f.value.relief_m);
      if (f.value.state==core::IntrinsicCellState::kBlocked)
        value.state=core::IntrinsicCellState::kBlocked;
      else if (f.value.state==core::IntrinsicCellState::kUnknown &&
               value.state!=core::IntrinsicCellState::kBlocked)
        value.state=core::IntrinsicCellState::kUnknown;
    }
    if (area<cell_area-epsilon && value.state!=core::IntrinsicCellState::kBlocked)
      value.state=core::IntrinsicCellState::kUnknown;
    (*values)[index]=value;
    tiles[core::TileForCell(index)].push_back({index,static_cast<float>(height/area)});
  }
  m.values=std::move(values);
  // 按已有稀疏tile分批写入高程，避免远离的世界足迹分配巨型稠密矩形。
  for (const auto& [tile,cells]:tiles) {
    auto minimum=cells[0].first, maximum=minimum;
    for (const auto& [i,h]:cells) {
      minimum.x=std::min(minimum.x,i.x); minimum.y=std::min(minimum.y,i.y);
      maximum.x=std::max(maximum.x,i.x); maximum.y=std::max(maximum.y,i.y);
    }
    const std::size_t width=maximum.x-minimum.x+1, height=maximum.y-minimum.y+1;
    std::vector<float> heights(width*height,std::numeric_limits<float>::quiet_NaN());
    for (const auto& [i,h]:cells) heights[(i.y-minimum.y)*width+i.x-minimum.x]=h;
    const core::GridGeometry geometry{.frame_id=facts.frame, .width=width, .height=height,
        .resolution_m=m.resolution, .origin_m={minimum.x*m.resolution,minimum.y*m.resolution,0.}};
    if (m.elevation.Apply({.geometry=geometry, .elevation_m=heights,
        .map_from_source={.parent_frame=facts.frame,.child_frame=facts.frame}}).status==
        core::ElevationUpdateResult::Status::kRejected) return false;
  }
  const auto next_revision=previous.last_fine_revision+1;
  if (!m.history.empty()) {
    auto next=core::FineTraversabilityBuilder{}.DeriveProjected(m.elevation.Snapshot(),
        m.capability,m.profile,std::make_unique<const ProjectedEvaluator>(m.values));
    m.fine=std::make_shared<const core::FineTraversabilitySnapshot>(
        next->geometry(),next->raw_elevation_revision(),next_revision,
        next->platform_profile_hash(),next->hard_inflation_radius_m(),
        next->preferred_clearance_m(),next->cost_weights(),next->elevation(),next->tile_directory(),
        std::vector<core::TileIndex>(next->changed_tiles().begin(),next->changed_tiles().end()),
        std::vector<core::TileIndex>(next->changed_halo_tiles().begin(),next->changed_halo_tiles().end()),
        next->metrics());
  }
  m.last_fine_revision=next_revision;
  // 完整替换可以撤销任意旧tile，旧版本必须重新获取完整快照。
  m.geometry_revision=next_revision;
  m.source_epoch=facts.epoch; m.source_frame=facts.frame;
  m.source_raw_revision=facts.raw_revision; m.source_resolution=facts.source_resolution_m;
  m.stamp=stamp;
  impl_=std::move(replacement);
  return true;
}
std::string ExplorationMap::SourceEpoch() const { return impl_->source_epoch; }
std::uint64_t ExplorationMap::SourceRawRevision() const { return impl_->source_raw_revision; }
ObservedPolicyMapExportInput ExplorationMap::Capture(
    std::uint64_t since) const {
  const auto& m = *impl_;
  ObservedPolicyMapExportInput result{
      .map = {.fine = m.fine, .processed_stamp_ns = m.stamp},
      .intrinsic = std::make_shared<const ProjectedEvaluator>(m.values)};
  auto& out = result.map;
  if (!m.fine || since == 0 || since < m.geometry_revision ||
      since > m.fine->fine_traversability_revision())
    return result;
  if (since != m.fine->fine_traversability_revision() &&
      (m.journal.empty() || since + 1 < m.journal.front().first))
    return result;
  out.full_snapshot = false;
  out.base_revision = since;
  std::set<core::TileIndex> dirty;
  for (const auto& [revision, tiles] : m.journal)
    if (revision > since) dirty.insert(tiles.begin(), tiles.end());
  out.dirty_tiles = {dirty.begin(), dirty.end()};
  return result;
}
ExplorationMapProjection ExplorationMap::Project() const {
  const auto& m = *impl_;
  if (!m.fine) return {};
  const auto& g = m.fine->geometry();
  if (m.coarse == g.resolution_m())
    return ExplorationMapProjector{}.ProjectConservative(*m.fine, g);
  const auto o = g.origin_m();
  const core::SparseGridGeometry coarse(
      g.frame_id(), m.coarse, {},
      {static_cast<std::int64_t>(std::floor(
           (o.x + g.min_inclusive().x * g.resolution_m()) / m.coarse)),
       static_cast<std::int64_t>(std::floor(
           (o.y + g.min_inclusive().y * g.resolution_m()) / m.coarse))},
      {static_cast<std::int64_t>(std::ceil(
           (o.x + g.max_exclusive().x * g.resolution_m()) / m.coarse)),
       static_cast<std::int64_t>(std::ceil(
           (o.y + g.max_exclusive().y * g.resolution_m()) / m.coarse))});
  return ExplorationMapProjector{}.ProjectConservative(*m.fine, coarse);
}
ExplorationMapStorageStats ExplorationMap::StorageStats() const {
  ExplorationMapStorageStats result{.policy_cells = impl_->history.size()};
  result.facts_aabb_checks=impl_->facts_aabb_checks;
  result.facts_overlap_candidates=impl_->facts_overlap_candidates;
  for (const auto& [index, fragments] : impl_->history)
    result.fragments += fragments.size();
  return result;
}
std::uint64_t ExplorationMap::MinimumCompatibleRevision() const {
  return impl_->geometry_revision;
}
lunar_planning_msgs::srv::GetPolicyMap::Response SelectExplorationMapResponse(
    const PublishedExplorationMap& snapshot, std::uint64_t since_revision,
    std::int64_t minimum_map_stamp_ns) {
  auto response = snapshot.response;
  const auto stamp = static_cast<std::int64_t>(response.processed_stamp.sec) *
      1000000000LL + response.processed_stamp.nanosec;
  if (stamp < minimum_map_stamp_ns) {
    response.ready = false;
    response.reason_code = "MAP_STAMP_TOO_OLD";
    return response;
  }
  if (since_revision > 0 &&
      since_revision >= snapshot.minimum_compatible_revision &&
      since_revision <= response.fine_revision) {
    response.full_snapshot = false;
    response.base_revision = since_revision;
    if (since_revision == response.fine_revision) response.tiles.clear();
  }
  return response;
}
PublishedExplorationMap BuildExplorationMapPublication(
    const ObservedPolicyMapExportInput& input, const PublishedExplorationMap* previous,
    const PolicyMapExporter& exporter, std::uint64_t minimum_compatible_revision,
    const std::string& empty_frame) {
  if (!input.map.fine) {
    lunar_planning_msgs::srv::GetPolicyMap::Response response;
    response.ready=false; response.reason_code="INPUT_UNAVAILABLE";
    response.epoch=input.map.epoch; response.frame_id=empty_frame;
    response.fine_revision=minimum_compatible_revision;
    response.processed_stamp.sec=static_cast<std::int32_t>(input.map.processed_stamp_ns/1000000000LL);
    response.processed_stamp.nanosec=static_cast<std::uint32_t>(input.map.processed_stamp_ns%1000000000LL);
    response.full_snapshot=true;
    return {std::move(response),minimum_compatible_revision};
  }
  auto captured=input;
  const bool same_fine=previous && previous->response.epoch==input.map.epoch &&
      previous->response.frame_id==input.map.fine->geometry().frame_id() &&
      previous->response.fine_revision==input.map.fine->fine_traversability_revision();
  if (same_fine) {
    captured.map.full_snapshot=false;
    captured.map.base_revision=previous->response.fine_revision;
  }
  auto response=exporter.ExportObserved(captured,same_fine ? previous->response.fine_revision : 0);
  if (same_fine) response.tiles=previous->response.tiles;
  response.full_snapshot=true; response.base_revision=0;
  if (!captured.map.anchor) {response.ready=false;response.reason_code="INPUT_UNAVAILABLE";}
  return {std::move(response),minimum_compatible_revision};
}

}  // namespace lunar::incremental_navigation_ros
