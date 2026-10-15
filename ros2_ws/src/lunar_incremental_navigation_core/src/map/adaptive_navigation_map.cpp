#include "lunar_incremental_navigation_core/adaptive_navigation_map.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
namespace lunar::incremental_navigation {
struct AdaptiveNavigationTile final {
  std::shared_ptr<const FineTraversabilityTile> source;
  std::array<std::int64_t, 4> clip{};
  std::vector<AdaptiveNavigationRegion> nodes;
  std::size_t regions{};
};
namespace {
using Region = AdaptiveNavigationRegion;
struct Interrupted final {};
struct Control final {
  const NowFn &now;
  SearchDeadline deadline;
  const StopToken &stop;
  mutable std::uint64_t ticks{};
  void Check() const {
    if (stop.stop_requested() || now() >= deadline)
      throw Interrupted{};
  }
  void Tick() const {
    if ((++ticks & 255U) == 0U)
      Check();
  }
};
std::array<std::int64_t, 4> Clip(const SparseGridGeometry &g, TileIndex t) {
  const auto x = t.x * kGridTileWidthCells, y = t.y * kGridTileWidthCells;
  // Inclusive last-cell coordinates avoid overflow at the integer boundary.
  return {std::max(x, g.min_inclusive().x), std::max(y, g.min_inclusive().y),
          std::min(x + 255, g.max_exclusive().x - 1),
          std::min(y + 255, g.max_exclusive().y - 1)};
}
std::int32_t Build(AdaptiveNavigationTile &out, std::int64_t x, std::int64_t y,
                   int size, const Control &c, double maximum_cost_spread) {
  c.Tick();
  const auto id = static_cast<std::int32_t>(out.nodes.size());
  out.nodes.push_back(Region{.x = x, .y = y, .size = size});
  if (x > out.clip[2] || y > out.clip[3] || x + size - 1 < out.clip[0] ||
      y + size - 1 < out.clip[1])
    return id;
  if (size == 1) {
    auto &node = out.nodes[id];
    auto offset = TileCellOffset({x, y});
    node.free = out.source->State(offset) == FineCellState::kFree;
    node.unknown = out.source->State(offset) == FineCellState::kUnknown;
    if (node.free)
      node.minimum_cost = node.cost = out.source->TraversalCost(offset);
    return id;
  }
  const int half = size / 2;
  std::array<std::int32_t, 4> children;
  children[0] = Build(out, x, y, half, c, maximum_cost_spread);
  children[1] = Build(out, x + half, y, half, c, maximum_cost_spread);
  children[2] = Build(out, x, y + half, half, c, maximum_cost_spread);
  children[3] = Build(out, x + half, y + half, half, c, maximum_cost_spread);
  bool same = true;
  double minimum = INFINITY, maximum = 0.;
  const auto first = out.nodes[children[0]];
  for (auto child : children) {
    const auto &q = out.nodes[child];
    same &= q.leaf && q.free == first.free &&
            (q.free || q.unknown == first.unknown);
    minimum = std::min(minimum, q.minimum_cost);
    maximum = std::max(maximum, q.cost);
  }
  same &= !first.free || maximum - minimum <= maximum_cost_spread;
  if (same) {
    out.nodes.resize(static_cast<std::size_t>(id) + 1);
    out.nodes[id].unknown = first.unknown;
    out.nodes[id].free = first.free;
    out.nodes[id].cost = maximum;
    out.nodes[id].minimum_cost = minimum;
  } else {
    out.nodes[id].leaf = false;
    out.nodes[id].children = children;
  }
  return id;
}

} // namespace
const AdaptiveNavigationRegion *
AdaptiveNavigationSnapshot::RegionAt(GridIndex cell) const {
  if (!fine_->geometry().Contains(cell))
    return nullptr;
  auto it = tiles_.find(TileForCell(cell));
  if (it == tiles_.end())
    return nullptr;
  const auto &nodes = it->second->nodes;
  const auto *node = &nodes[0];
  while (!node->leaf) {
    int half = node->size / 2;
    int index =
        (cell.x >= node->x + half ? 1 : 0) + (cell.y >= node->y + half ? 2 : 0);
    node = &nodes[node->children[index]];
  }
  return node;
}
struct AdaptiveNavigationMapBuilder::Impl final {
  NowFn now;
  double maximum_cost_spread{};
  std::map<TileIndex, std::shared_ptr<const AdaptiveNavigationTile>> tiles;
  std::optional<SparseGridGeometry> geometry;
  std::string profile;
};
AdaptiveNavigationMapBuilder::AdaptiveNavigationMapBuilder(
    NowFn now, double maximum_cost_spread)
    : impl_(std::make_unique<Impl>()) {
  if (!now || !std::isfinite(maximum_cost_spread) || maximum_cost_spread < 0.)
    throw std::invalid_argument(
        "map builder requires a clock and nonnegative cost spread");
  impl_->now = std::move(now);
  impl_->maximum_cost_spread = maximum_cost_spread;
}
AdaptiveNavigationMapBuilder::~AdaptiveNavigationMapBuilder() = default;
AdaptiveMapBuildResult AdaptiveNavigationMapBuilder::Update(
    std::shared_ptr<const FineTraversabilitySnapshot> fine,
    SearchDeadline deadline, const StopToken &stop) {
  if (!fine)
    throw std::invalid_argument("map builder requires a fine snapshot");
  AdaptiveMapBuildResult result;
  const auto begin = impl_->now();
  const Control c{impl_->now, deadline, stop};
  try {
    c.Check();
    const auto &g = fine->geometry();
    if (!impl_->geometry || impl_->geometry->frame_id() != g.frame_id() ||
        impl_->geometry->resolution_m() != g.resolution_m() ||
        impl_->geometry->origin_m() != g.origin_m() ||
        impl_->profile != fine->platform_profile_hash())
      impl_->tiles.clear();
    impl_->geometry = g;
    impl_->profile = fine->platform_profile_hash();
    auto snapshot = std::make_shared<AdaptiveNavigationSnapshot>();
    snapshot->fine_ = fine;
    const auto indices = fine->tile_indices();
    c.Check();
    std::set<TileIndex> present;
    for (const auto &index : indices) {
      c.Tick();
      present.insert(index);
    }
    for (auto i = impl_->tiles.begin(); i != impl_->tiles.end();) {
      c.Tick();
      if (!present.contains(i->first))
        i = impl_->tiles.erase(i);
      else
        ++i;
    }
    for (auto index : indices) {
      c.Check();
      auto source = fine->FindTile(index);
      auto clip = Clip(g, index);
      auto it = impl_->tiles.find(index);
      if (it != impl_->tiles.end() && it->second->source == source &&
          it->second->clip == clip)
        ++result.metrics.reused_tiles;
      else {
        auto tile = std::make_shared<AdaptiveNavigationTile>();
        tile->source = source;
        tile->clip = clip;
        Build(*tile, index.x * 256, index.y * 256, 256, c,
              impl_->maximum_cost_spread);
        c.Check();
        tile->regions =
            std::count_if(tile->nodes.begin(), tile->nodes.end(),
                          [](const auto &n) { return n.leaf && n.free; });
        tile->nodes.shrink_to_fit();
        impl_->tiles.insert_or_assign(index, std::move(tile));
        ++result.metrics.built_tiles;
      }
      snapshot->tiles_.emplace(index, impl_->tiles.at(index));
    }
    for (const auto &[index, tile] : snapshot->tiles_) {
      c.Tick();
      result.metrics.regions += tile->regions;
      result.metrics.cached_bytes += sizeof(AdaptiveNavigationTile) +
                                     tile->nodes.capacity() * sizeof(Region);
    }
    c.Check();
    result.status = AdaptiveMapBuildResult::Status::kReady;
    result.metrics.build_ms =
        std::chrono::duration<double, std::milli>(impl_->now() - begin).count();
    snapshot->metrics_ = result.metrics;
    result.snapshot = std::move(snapshot);
  } catch (const Interrupted &) {
    result.status = stop.stop_requested()
                        ? AdaptiveMapBuildResult::Status::kCanceled
                        : AdaptiveMapBuildResult::Status::kTimeout;
    result.metrics.build_ms =
        std::chrono::duration<double, std::milli>(impl_->now() - begin).count();
  }
  return result;
}
} // namespace lunar::incremental_navigation
