#pragma once
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "map/grid_bounds.hpp"
namespace lunar::incremental_navigation::detail {
struct InfluenceStats { std::size_t source_runs{}; std::size_t generated_spans{}; };
inline Point2 InfluenceCellCenter(const SparseGridGeometry& g, GridIndex p) {
  return {g.origin_m().x + (static_cast<double>(p.x) + 0.5) * g.resolution_m(),
          g.origin_m().y + (static_cast<double>(p.y) + 0.5) * g.resolution_m()};
}
using InfluenceRows = std::map<std::int64_t,
    std::vector<std::pair<std::int64_t, std::int64_t>>>;
inline std::set<GridIndex> MergeInfluenceRows(InfluenceRows& rows) {
  std::set<GridIndex> result;
  for (auto& [y, spans] : rows) {
    std::sort(spans.begin(), spans.end());
    auto left = spans.front().first, right = spans.front().second;
    const auto emit = [&](std::int64_t begin, std::int64_t end) {
      for (auto x = begin; x < end; ++x) result.insert({x, y});
    };
    for (std::size_t i = 1; i < spans.size(); ++i) {
      if (spans[i].first <= right) right = std::max(right, spans[i].second);
      else {
        emit(left, right);
        left = spans[i].first;
        right = spans[i].second;
      }
    }
    emit(left, right);
  }
  return result;
}
[[nodiscard]] inline std::set<GridIndex> LegacyInfluenceCells(
    const SparseGridGeometry& geometry,
    const std::set<GridIndex>& changed_raw_cells,
    const double influence_m, InfluenceStats* stats = nullptr) {
  if (stats) *stats = {};
  // The intersection of a circle's cell-area halo with one grid row is
  // contiguous. Keep the original endpoint predicate, then union row spans
  // before materializing cells instead of repeatedly inserting each overlap.
  std::map<std::int64_t, std::vector<std::pair<std::int64_t, std::int64_t>>> rows;
  for (const GridIndex changed : changed_raw_cells) {
    if (stats) ++stats->source_runs;
    const auto [minimum, maximum] =
        detail::CellBoundsForRadius(geometry, changed, influence_m);
    for (auto y = minimum.y; y < maximum.y; ++y) {
      auto left = minimum.x, right = maximum.x;
      const auto intersects = [&](std::int64_t x) {
        return CircleIntersectsCellArea(InfluenceCellCenter(geometry, {x, y}),
                                        influence_m, geometry, changed);
      };
      while (left < right && !intersects(left)) ++left;
      while (left < right && !intersects(right - 1)) --right;
      if (left < right) { rows[y].emplace_back(left, right); if (stats) ++stats->generated_spans; }
    }
  }
  return MergeInfluenceRows(rows);
}


// Only merge runs when every computed x center stays strictly inside its own
// cell. This conservative roundoff bound covers conversion, multiply/add,
// origin subtraction and division. Large/ill-conditioned coordinates use the
// original per-cell algorithm with exactly its endpoint predicates.
inline bool CanMergeInfluenceRuns(const SparseGridGeometry& geometry) {
  if (!geometry.valid()) return false;
  const long double index_scale = std::max(
      std::abs(static_cast<long double>(geometry.min_inclusive().x)),
      std::abs(static_cast<long double>(geometry.max_exclusive().x)));
  const long double scale = index_scale + 2.0L +
      std::abs(static_cast<long double>(geometry.origin_m().x)) /
          geometry.resolution_m();
  return index_scale < 0x1p51L &&
      std::isfinite(scale) &&
      16.0L * std::numeric_limits<double>::epsilon() * scale < 0.25L;
}

[[nodiscard]] inline std::set<GridIndex> InfluenceCells(
    const SparseGridGeometry& geometry,
    const std::set<GridIndex>& changed_raw_cells,
    const double influence_m, InfluenceStats* stats = nullptr) {
  if (!CanMergeInfluenceRuns(geometry))
    return LegacyInfluenceCells(geometry, changed_raw_cells, influence_m, stats);
  if (stats) *stats = {};

  // GridIndex ordering is x then y, so each row receives increasing x values.
  // Runs use inclusive endpoints; checked addition also handles extreme input.
  InfluenceRows source_rows;
  for (const auto cell : changed_raw_cells) {
    if (!geometry.Contains(cell)) continue;
    auto& runs = source_rows[cell.y];
    if (!runs.empty() && CheckedAdd(runs.back().second, 1) == cell.x)
      runs.back().second = cell.x;
    else {
      runs.emplace_back(cell.x, cell.x);
      if (stats) ++stats->source_runs;
    }
  }

  InfluenceRows affected_rows;
  for (const auto& [source_y, runs] : source_rows) {
    for (const auto& [first, last] : runs) {
      const auto [minimum, first_maximum] =
          CellBoundsForRadius(geometry, {first, source_y}, influence_m);
      const auto last_maximum =
          CellBoundsForRadius(geometry, {last, source_y}, influence_m).second;
      for (auto y = minimum.y; y < first_maximum.y; ++y) {
        const auto intersects = [&](std::int64_t x) {
          // Nearest cell of this contiguous run, not a changed collision model.
          const GridIndex nearest{std::clamp(x, first, last), source_y};
          return CircleIntersectsCellArea(InfluenceCellCenter(geometry, {x, y}),
                                          influence_m, geometry, nearest);
        };
        // At an interior x, horizontal distance is zero. If this fails the
        // whole row is outside the halo; avoid scanning a long empty run.
        if (!intersects(first)) continue;
        auto left = minimum.x, right = last_maximum.x;
        while (left < right && !intersects(left)) ++left;
        while (left < right && !intersects(right - 1)) --right;
        if (left < right) {
          affected_rows[y].emplace_back(left, right);
          if (stats) ++stats->generated_spans;
        }
      }
    }
  }
  return MergeInfluenceRows(affected_rows);
}
}  // namespace lunar::incremental_navigation::detail
