// 仅做 ABI、源栅格累计与结果行程合并；物理阈值与判定全部调用既有 core。
#include "terrain_bridge.h"

#include <algorithm>
#include <atomic>
#include <chrono>
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
#include "lunar_incremental_navigation_core/terrain_dependency.hpp"
#include "lunar_incremental_navigation_core/terrain_batch.hpp"

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
    capability = wheel;
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
    CheckCancelled();
    profile={};
    const auto started = std::chrono::steady_clock::now();
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
    yaw = std::remainder(yaw, 2 * std::numbers::pi);
    if (!observed) {
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
    const auto accumulated = std::chrono::steady_clock::now();
    using Span = std::pair<std::int64_t,std::int64_t>;
    std::map<std::int64_t,std::vector<Span>> touched;
    const auto minimum=snapshot->geometry().min_inclusive();
    const auto maximum=snapshot->geometry().max_exclusive();
    const auto add=[&](std::int64_t y,std::int64_t left,std::int64_t right) {
      left=std::max(left,minimum.x);right=std::min(right,maximum.x);
      if (y>=minimum.y && y<maximum.y && left<right) touched[y].emplace_back(left,right);
    };
    // 直接观测即使 duplicate 仍需发布，来源权限不能被缓存替代。
    for (std::size_t y=0;y<height;++y) {
      CheckCancelled();
      for (std::size_t x=0;x<width;) {
        if (!std::isfinite(values[y*width+x])) {++x;continue;}
        const auto first=x++;
        while (x<width && std::isfinite(values[y*width+x])) ++x;
        add(offset.y+static_cast<std::int64_t>(y),offset.x+static_cast<std::int64_t>(first),offset.x+static_cast<std::int64_t>(x));
      }
    }
    if (applied.status==nav::ElevationUpdateResult::Status::kApplied) {
      const auto radius=nav::TerrainIntrinsicDependencyRadiusCells(resolution);
      // changed 先按行压成连续区间，再膨胀；完整帧从十亿重复插入降为
      // O(changed + 行程数 * radius)，最终只遍历每个合并区域一次。
      std::map<std::int64_t,std::vector<std::int64_t>> changed_rows;
      for (auto i:snapshot->changed_cells()) changed_rows[i.y].push_back(i.x);
      for (auto& [y,xs]:changed_rows) {
        CheckCancelled();std::sort(xs.begin(),xs.end());
        for (std::size_t n=0;n<xs.size();) {
          auto left=xs[n],right=left+1;++n;
          while (n<xs.size() && xs[n]<=right) {right=std::max(right,xs[n]+1);++n;}
          for (int dy=-radius;dy<=radius;++dy) add(y+dy,left-radius,right+radius);
        }
      }
    }
    for (auto& [y,spans]:touched) {
      std::sort(spans.begin(),spans.end());std::vector<Span> merged;
      for (auto span:spans) {
        if (!merged.empty() && span.first<=merged.back().second)
          merged.back().second=std::max(merged.back().second,span.second);
        else merged.push_back(span);
      }
      spans=std::move(merged);
    }
    const auto expanded=std::chrono::steady_clock::now();
    CheckCancelled();
    nav::SnapshotTerrainClassifier classifier(*snapshot,capability);
    const auto prepared=std::chrono::steady_clock::now();
    std::map<std::int64_t,std::map<std::int64_t,std::int64_t>> labels;
    for (const auto& [y,spans]:touched) for (const auto [left,right]:spans)
      for (auto x=left;x<right;++x) {
        CheckCancelled();const nav::GridIndex index{x,y};
        if (!snapshot->ElevationRangeAt(index)) continue;
        const auto state=classifier.Classify(index);
        labels[y][x]=state==nav::IntrinsicCellState::kBlocked?1:state==nav::IntrinsicCellState::kFree?0:-1;
      }
    const auto evaluated=std::chrono::steady_clock::now();
    MergeRows(labels);
    const auto finished=std::chrono::steady_clock::now();
    const auto seconds=[](auto a,auto b){return std::chrono::duration<double>(b-a).count();};
    profile={seconds(started,accumulated),seconds(accumulated,expanded),
             seconds(expanded,prepared),seconds(prepared,evaluated),seconds(evaluated,finished)};
    CheckCancelled();
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
    return {resolution, lattice->origin_x_m, lattice->origin_y_m, lattice->yaw,
            regions.empty() ? nullptr : regions.front().data(), regions.size(), lattices.size(),
            lattice->source_id, evicted_source_id, kSourceCacheCapacity};
  }

  void CheckCancelled() const {
    if (cancelled.load(std::memory_order_relaxed)) throw std::runtime_error("terrain cancelled for shutdown");
  }
  std::atomic<bool> cancelled{false};
  std::array<double,5> profile{};
  nav::PlatformCapability capability;
  Lattices lattices;
  std::uint64_t next_source_id = 1;
  std::vector<Region> regions;
};
}  // namespace

extern "C" {
int nav1cm_terrain_abi_version(void) { return 2; }
void nav1cm_terrain_cancel_v2(void* handle) {
  if (handle) static_cast<Terrain*>(handle)->cancelled.store(true,std::memory_order_relaxed);
}
int nav1cm_terrain_profile_v2(void* handle,double* phases) {
  if (!handle || !phases) return 1;
  const auto& p=static_cast<Terrain*>(handle)->profile;
  std::copy(p.begin(),p.end(),phases);return 0;
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
