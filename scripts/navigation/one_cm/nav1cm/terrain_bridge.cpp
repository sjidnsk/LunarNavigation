// 仅做 ABI、源栅格累计与结果行程合并；物理阈值与判定全部调用既有 core。
#include "terrain_bridge.h"
#include "lunar_incremental_navigation_core/convex_polygon.hpp"
#include "lunar_incremental_navigation_core/platform_terrain_window.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "lunar_incremental_navigation_core/elevation_map.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"

namespace nav = lunar::incremental_navigation;

namespace {
thread_local std::string last_error;
using Region = std::array<std::int64_t, 5>;
static_assert(sizeof(Region) == 5 * sizeof(std::int64_t));
constexpr std::size_t kSourceCacheCapacity = 8;

struct Lattice final {
  std::uint64_t source_id;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
  double yaw;
  nav::PersistentElevationMap elevation;
};

// 同分辨率、同朝向且整格平移的滚动窗口共享累计。其他分辨率、旋转或
// 亚格相位使用独立源栅格，不能偷偷投到旧分辨率后再计算坡度。
bool AlignedOffset(const Lattice& lattice, double resolution_m,
                   double origin_x, double origin_y, double yaw,
                   nav::GridIndex* offset) {
  if (lattice.resolution_m != resolution_m ||
      std::abs(std::remainder(lattice.yaw - yaw, 2 * std::numbers::pi)) > 1e-12) {
    return false;
  }
  const double c = std::cos(lattice.yaw), s = std::sin(lattice.yaw);
  const double dx = origin_x - lattice.origin_x_m;
  const double dy = origin_y - lattice.origin_y_m;
  const double x = (c * dx + s * dy) / resolution_m;
  const double y = (-s * dx + c * dy) / resolution_m;
  // 仅在舍入误差范围内复用。无法安全表达的巨大偏移另开源栅格。
  constexpr double bound = static_cast<double>(std::numeric_limits<std::int64_t>::max() / 4);
  if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > bound ||
      std::abs(y) > bound || std::abs(x - std::round(x)) > 1e-7 ||
      std::abs(y - std::round(y)) > 1e-7) {
    return false;
  }
  *offset = {static_cast<std::int64_t>(std::round(x)),
             static_cast<std::int64_t>(std::round(y))};
  return true;
}

struct Terrain final {
  using Lattices = std::list<std::unique_ptr<Lattice>>;
  explicit Terrain(double slope, double relief, double clearance) {
    nav::WheeledCapability wheel;
    wheel.maximum_slope_rad = slope;
    wheel.maximum_local_obstacle_relief_m = relief;
    wheel.minimum_underbody_clearance_m = clearance;
    // 使用 intrinsic evaluator，避免 FineCellEvaluator 的整车膨胀再进入
    // nav1cm 的矩形车体认证，造成双重膨胀。
    evaluator = nav::MakePlatformElevationEvaluator(wheel);
  }

  explicit Terrain(const nav::LeggedCapability& legged) {
    evaluator = nav::MakePlatformElevationEvaluator(legged);
  }

  Terrain(int kind,double length,double slope,double relief,double clearance)
      : Terrain(slope,relief,clearance) {
    if (!std::isfinite(length)||length<=0||kind<1||kind>2)
      throw std::invalid_argument("invalid platform terrain window");
    long_side_m=length;
    if(kind==1){
      nav::WheeledCapability wheel;wheel.body_extent_m={length,length,1.};
      wheel.maximum_slope_rad=slope;wheel.maximum_local_obstacle_relief_m=relief;
      wheel.minimum_underbody_clearance_m=clearance;platform=wheel;
    }else{
      nav::LeggedCapability legged;legged.body_extent_m={length,length,1.};
      legged.maximum_slope_rad=slope;legged.maximum_step_height_m=relief;
      legged.minimum_body_clearance_m=clearance;platform=legged;
    }
  }

  std::pair<Lattices::iterator, nav::GridIndex> FindLattice(
      double resolution, double x, double y, double yaw) {
    for (auto it = lattices.begin(); it != lattices.end(); ++it) {
      nav::GridIndex offset;
      if (AlignedOffset(**it, resolution, x, y, yaw, &offset)) {
        return {it, offset};
      }
    }
    return {lattices.end(), {0, 0}};
  }

  void MergeRows(const std::map<std::int64_t, std::map<std::int64_t, std::int64_t>>& rows) {
    using Key = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
    std::map<Key, std::size_t> active;
    for (const auto& [y, cells] : rows) {
      std::map<Key, std::size_t> next;
      auto start = cells.begin();
      while (start != cells.end()) {
        auto end = std::next(start);
        std::int64_t x1 = start->first + 1;
        while (end != cells.end() && end->first == x1 && end->second == start->second) {
          ++x1;
          ++end;
        }
        Key key{start->first, x1, start->second};
        const auto found = active.find(key);
        if (found != active.end() && regions[found->second][3] == y) {
          regions[found->second][3] = y + 1;
          next[key] = found->second;
        } else {
          next[key] = regions.size();
          regions.push_back({start->first, y, x1, y + 1, start->second});
        }
        start = end;
      }
      active = std::move(next);
    }
  }

  Nav1cmTerrainResultV2 Apply(const float* data, std::size_t height,
                             std::size_t width, double resolution,
                             double origin_x, double origin_y, double yaw,
                             double z_offset) {
    if (!data || height == 0 || width == 0 ||
        width > std::numeric_limits<std::size_t>::max() / height ||
        width > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max() / 4) ||
        height > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max() / 4) ||
        !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(yaw) || !std::isfinite(z_offset)) {
      throw std::invalid_argument("invalid native terrain source geometry");
    }
    std::vector<float> values(data, data + height * width);
    bool observed = false;
    for (float& value : values) {
      if (!std::isfinite(value)) continue;
      const double shifted = static_cast<double>(value) + z_offset;
      if (!std::isfinite(shifted) || std::abs(shifted) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("native elevation exceeds float32 range");
      }
      value = static_cast<float>(shifted);
      observed = true;
    }
    regions.clear();
    std::vector<Nav1cmTerrainMeasurementV3> next_measurements;
    yaw = std::remainder(yaw, 2 * std::numbers::pi);
    if (!observed) {
      measurements.swap(next_measurements);
      return {resolution, origin_x, origin_y, yaw, nullptr, 0, lattices.size(),
              0, 0, kSourceCacheCapacity};
    }
    auto [found, offset] = FindLattice(resolution, origin_x, origin_y, yaw);
    std::unique_ptr<Lattice> pending;
    if (found == lattices.end()) {
      if (next_source_id == 0) throw std::overflow_error("native terrain source identity exhausted");
      pending = std::make_unique<Lattice>();
      pending->source_id = next_source_id;
      pending->resolution_m = resolution;
      pending->origin_x_m = origin_x;
      pending->origin_y_m = origin_y;
      pending->yaw = yaw;
    }
    Lattice* lattice = pending ? pending.get() : found->get();
    const nav::ElevationEvidence evidence{
        .geometry = {.frame_id = "map", .width = width, .height = height,
                     .resolution_m = resolution,
                     .origin_m = {offset.x * resolution, offset.y * resolution, 0}},
        .elevation_m = values,
        .map_from_source = {.parent_frame = "map", .child_frame = "map"}};
    const auto applied = lattice->elevation.Apply(evidence);
    if (applied.status == nav::ElevationUpdateResult::Status::kRejected) {
      throw std::runtime_error("PersistentElevationMap rejected native source geometry");
    }
    const auto snapshot = lattice->elevation.Snapshot();
    std::set<nav::GridIndex> touched;
    // 每次都包括本次明确观测格。即便本源栅格是 duplicate，新的自由证据
    // 也必须能替换别的分辨率/相位先前投影出来的障碍。
    for (std::size_t y = 0; y < height; ++y) {
      for (std::size_t x = 0; x < width; ++x) {
        if (std::isfinite(values[y * width + x])) {
          touched.insert({offset.x + static_cast<std::int64_t>(x),
                          offset.y + static_cast<std::int64_t>(y)});
        }
      }
    }
    if (applied.status == nav::ElevationUpdateResult::Status::kApplied) {
      // 按行合并外扩影响域，避免每个changed cell枚举整个L窗口。
      const auto radius=platform ? nav::PlatformTerrainWindowRadiusCells(long_side_m,resolution) : 1;
      std::map<std::int64_t,std::pair<std::int64_t,std::int64_t>> changed_rows,expanded;
      auto include=[](auto& rows,std::int64_t y,std::int64_t lo,std::int64_t hi){
        auto [it,added]=rows.try_emplace(y,std::pair{lo,hi});
        if(!added){it->second.first=std::min(it->second.first,lo);it->second.second=std::max(it->second.second,hi);}
      };
      for(auto c:snapshot->changed_cells())include(changed_rows,c.y,c.x,c.x);
      for(auto [y,span]:changed_rows){
        if(y<std::numeric_limits<std::int64_t>::min()+radius||y>std::numeric_limits<std::int64_t>::max()-radius||
           span.first<std::numeric_limits<std::int64_t>::min()+radius||span.second>std::numeric_limits<std::int64_t>::max()-radius)
          throw std::invalid_argument("terrain dependency coordinate overflow");
        for(std::int64_t dy=-radius;dy<=radius;++dy)include(expanded,y+dy,span.first-radius,span.second+radius);
      }
      for(auto [y,span]:expanded)for(auto x=span.first;;++x){touched.insert({x,y});if(x==span.second)break;}
    }
    auto window_evaluator=platform ? nav::MakePlatformWindowElevationEvaluator(*snapshot,*platform) : nullptr;
    const auto* active_evaluator=window_evaluator ? window_evaluator.get() : evaluator.get();
    std::map<std::int64_t, std::map<std::int64_t, std::int64_t>> labels;
    for (const auto index : touched) {
      if (!snapshot->ElevationRangeAt(index)) continue;
      const auto evaluation = active_evaluator->Evaluate(*snapshot, index);
      const auto label = evaluation.state == nav::IntrinsicCellState::kBlocked ? 1 :
          evaluation.state == nav::IntrinsicCellState::kFree ? 0 : -1;
      labels[index.y][index.x] = label;
      const auto height = snapshot->ElevationAt(index);
      next_measurements.push_back({index.x, index.y,
          height ? static_cast<double>(*height) : std::numeric_limits<double>::quiet_NaN(),
          label, evaluation.slope_rad, evaluation.relief_m});
    }
    MergeRows(labels);
    std::uint64_t evicted_source_id = 0;
    if (pending) {
      // 成功分类后才提交新源及淘汰事件，失败不能让 Python 丢失清缓存通知。
      lattices.push_back(std::move(pending));
      ++next_source_id;
      if (lattices.size() > kSourceCacheCapacity) {
        evicted_source_id = lattices.front()->source_id;
        lattices.pop_front();  // 销毁 PersistentElevationMap，释放其高程与快照。
      }
    } else {
      // duplicate 有限观测和同源整格滚动也属于使用，不能按 FIFO 淘汰。
      lattices.splice(lattices.end(), lattices, found);
    }
    measurements.swap(next_measurements);
    return {resolution, lattice->origin_x_m, lattice->origin_y_m, lattice->yaw,
            regions.empty() ? nullptr : regions.front().data(), regions.size(), lattices.size(),
            lattice->source_id, evicted_source_id, kSourceCacheCapacity};
  }

  std::unique_ptr<const nav::PlatformElevationEvaluator> evaluator;
  std::optional<nav::PlatformCapability> platform;
  double long_side_m{};
  Lattices lattices;
  std::uint64_t next_source_id = 1;
  std::vector<Region> regions;
  std::vector<Nav1cmTerrainMeasurementV3> measurements;
};
namespace poly = nav::convex_polygon;
struct PolygonResult { std::vector<double> xy; std::vector<std::size_t> offsets{0}; };
poly::Polygon ReadPolygon(const double* xy, std::size_t count) {
  if (!xy || count < 3 || count > std::numeric_limits<std::size_t>::max() / 2)
    throw std::invalid_argument("invalid polygon buffer/count");
  poly::Polygon p;
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(xy[2*i]) || !std::isfinite(xy[2*i+1]))
      throw std::invalid_argument("nonfinite polygon coordinate");
    if (p.empty() || p.back().x != xy[2*i] || p.back().y != xy[2*i+1])
      p.push_back({xy[2*i], xy[2*i+1]});
  }
  if (p.size() > 1 && p.front().x == p.back().x && p.front().y == p.back().y)
    p.pop_back();
  count = p.size();
  if (count < 3) throw std::invalid_argument("degenerate polygon");
  double sign = 0.;
  for (std::size_t i = 0; i < count; ++i) {
    const auto a = p[i], b = p[(i+1)%count];
    for (std::size_t j = 0; j < count; ++j) {
      const double cross = poly::Cross(a, b, p[j]);
      if (!std::isfinite(cross)) throw std::invalid_argument("polygon overflow");
      if (cross != 0.) {
        if (sign != 0. && (cross > 0.) != (sign > 0.)) throw std::invalid_argument("nonconvex polygon");
        sign = cross;
      }
    }
  }
  if (sign == 0. || !std::isfinite(poly::Area(p)) || poly::Area(p) == 0.)
    throw std::invalid_argument("degenerate polygon");
  if (sign < 0.) std::reverse(p.begin(), p.end());
  return p;
}
// Clipping subtracts endpoint coordinates and signed edge distances. Reject
// pairs whose combined range cannot be represented before invoking the core.
void ValidatePair(const poly::Polygon& subject, const poly::Polygon& clipper) {
  for (const auto& a : subject) {
    for (const auto& b : clipper) {
      if (!std::isfinite(a.x - b.x) || !std::isfinite(a.y - b.y))
        throw std::invalid_argument("polygon coordinate range overflow");
    }
  }
  for (std::size_t i = 0; i < clipper.size(); ++i) {
    const auto a = clipper[i], b = clipper[(i + 1) % clipper.size()];
    double low = 0., high = 0.;
    for (const auto& point : subject) {
      const double distance = poly::Cross(a, b, point);
      if (!std::isfinite(distance)) throw std::invalid_argument("polygon cross overflow");
      low = std::min(low, distance); high = std::max(high, distance);
    }
    if (!std::isfinite(high - low)) throw std::invalid_argument("polygon distance range overflow");
  }
}
}  // namespace

extern "C" {
int nav1cm_polygon_boolean_v1(int op, const double* subject, std::size_t subject_count,
    const double* clipper, std::size_t clipper_count, double epsilon, void** output) {
  try {
    last_error.clear();
    if (!output) throw std::invalid_argument("missing polygon result");
    *output = nullptr;
    if ((op != 0 && op != 1) || !std::isfinite(epsilon) || epsilon < 0.)
      throw std::invalid_argument("invalid polygon operation/epsilon");
    auto a = ReadPolygon(subject, subject_count), b = ReadPolygon(clipper, clipper_count);
    ValidatePair(a, b);
    std::vector<poly::Polygon> pieces;
    if (op == 0) { auto p = poly::Intersect(a, b); if (poly::Area(p) > epsilon) pieces.push_back(std::move(p)); }
    else pieces = poly::Subtract(a, b, epsilon);
    auto result = std::make_unique<PolygonResult>();
    for (const auto& p : pieces) {
      for (auto point : p) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
          throw std::invalid_argument("nonfinite polygon output");
        result->xy.push_back(point.x); result->xy.push_back(point.y);
      }
      result->offsets.push_back(result->xy.size() / 2);
    }
    *output = result.release();
    return 0;
  } catch (const std::exception& error) { last_error = error.what(); }
    catch (...) { last_error = "unknown polygon failure"; }
  return 1;
}
int nav1cm_polygon_query_v1(const void* handle, const double** xy,
    const std::size_t** offsets, std::size_t* count) {
  try {
    last_error.clear();
    if (!handle || !xy || !offsets || !count) throw std::invalid_argument("missing polygon query argument");
    const auto& result = *static_cast<const PolygonResult*>(handle);
    *xy = result.xy.empty() ? nullptr : result.xy.data();
    *offsets = result.offsets.data(); *count = result.offsets.size() - 1;
    return 0;
  } catch (const std::exception& error) { last_error = error.what(); }
    catch (...) { last_error = "unknown polygon query failure"; }
  return 1;
}
void nav1cm_polygon_destroy_v1(void* handle) { delete static_cast<PolygonResult*>(handle); }

int nav1cm_terrain_abi_version(void) { return 2; }

int nav1cm_terrain_measurements_v3(void* handle,
    const Nav1cmTerrainMeasurementV3** measurements, std::size_t* count) {
  if (!handle || !measurements || !count) {
    last_error = "native terrain measurement handle/result missing";
    return 1;
  }
  const auto& values = static_cast<Terrain*>(handle)->measurements;
  *measurements = values.empty() ? nullptr : values.data();
  *count = values.size();
  return 0;
}



const char* nav1cm_terrain_last_error_v2(void) { return last_error.c_str(); }

void* nav1cm_terrain_create_v2(double slope, double relief, double clearance) {
  try {
    last_error.clear();
    return new Terrain(slope, relief, clearance);
  } catch (const std::exception& error) {
    last_error = error.what();
  } catch (...) {
    last_error = "unknown native terrain create failure";
  }
  return nullptr;
}

void* nav1cm_terrain_create_legged_v2(double slope, double step, double gap) {
  try {
    last_error.clear();
    nav::LeggedCapability legged;
    legged.maximum_slope_rad = slope;
    legged.maximum_step_height_m = step;
    legged.maximum_gap_width_m = gap;
    return new Terrain(legged);
  } catch (const std::exception& error) {
    last_error = error.what();
  } catch (...) {
    last_error = "unknown native legged terrain create failure";
  }
  return nullptr;
}

void* nav1cm_terrain_create_platform_window_v2(int kind,double length,double slope,double relief,double clearance) {
  try{last_error.clear();return new Terrain(kind,length,slope,relief,clearance);}
  catch(const std::exception& e){last_error=e.what();}
  catch(...){last_error="unknown platform terrain create failure";}
  return nullptr;
}

void* nav1cm_terrain_create_legged_window_v2(double length, double slope, double step,
                                             double gap, double clearance) {
  try {
    last_error.clear();
    nav::LeggedCapability legged;
    legged.body_extent_m = {length, length, 1.};
    legged.maximum_slope_rad = slope;
    legged.maximum_step_height_m = step;
    legged.maximum_gap_width_m = gap;
    legged.minimum_body_clearance_m = clearance;
    if (!std::isfinite(length) || length <= 0) throw std::invalid_argument("invalid legged terrain window");
    auto terrain = std::make_unique<Terrain>(legged);
    terrain->platform = legged;
    terrain->long_side_m = length;
    return terrain.release();
  } catch (const std::exception& error) { last_error = error.what(); }
  catch (...) { last_error = "unknown legged window create failure"; }
  return nullptr;
}

void nav1cm_terrain_destroy_v2(void* handle) { delete static_cast<Terrain*>(handle); }

int nav1cm_terrain_apply_v2(void* handle, const float* data, std::size_t height,
                           std::size_t width, double resolution, double origin_x,
                           double origin_y, double yaw, double z_offset,
                           Nav1cmTerrainResultV2* result) {
  try {
    last_error.clear();
    if (!handle || !result) throw std::invalid_argument("native terrain handle/result missing");
    *result = static_cast<Terrain*>(handle)->Apply(data, height, width, resolution,
                                                 origin_x, origin_y, yaw, z_offset);
    return 0;
  } catch (const std::exception& error) {
    last_error = error.what();
  } catch (...) {
    last_error = "unknown native terrain apply failure";
  }
  return 1;
}
}  // extern "C"
