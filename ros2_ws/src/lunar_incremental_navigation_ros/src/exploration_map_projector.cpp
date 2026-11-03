#include "lunar_incremental_navigation_ros/exploration_map_projector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <type_traits>
#include <stdexcept>

namespace lunar::incremental_navigation_ros {

namespace {

namespace core = lunar::incremental_navigation;

[[nodiscard]] double CellMinimum(const double origin,
                                 const double resolution,
                                 const std::int64_t index) {
  return std::fma(static_cast<double>(index), resolution, origin);
}

[[nodiscard]] double OverlapLength(const double first_min,
                                   const double first_max,
                                   const double second_min,
                                   const double second_max) {
  return std::max(0.0,
                  std::min(first_max, second_max) -
                      std::max(first_min, second_min));
}

// 旧出口只需找到任意 FREE；保留对内外两层细格遍历的提前退出。
struct AnyFreeAggregation {
  bool has_free{};
  double blocked_area{};

  void Add(const core::FineCellState state, const double area) {
    if (state == core::FineCellState::kFree) {
      has_free = true;
    } else if (state == core::FineCellState::kBlocked) {
      blocked_area += area;
    }
  }
  [[nodiscard]] bool Done() const { return has_free; }
  [[nodiscard]] std::int8_t Occupancy(const double coarse_area,
                                    const double tolerance) const {
    if (has_free) return 0;
    return blocked_area + tolerance >= coarse_area ? 100 : -1;
  }
};

// 保守出口累计全部交叠面积，只有完整自由覆盖才可发布 FREE。
struct ConservativeAggregation {
  double free_area{};
  double blocked_area{};

  void Add(const core::FineCellState state, const double area) {
    if (state == core::FineCellState::kFree) {
      free_area += area;
    } else if (state == core::FineCellState::kBlocked) {
      blocked_area += area;
    }
  }
  [[nodiscard]] bool Done() const { return false; }
  [[nodiscard]] std::int8_t Occupancy(const double coarse_area,
                                    const double tolerance) const {
    if (blocked_area > tolerance) return 100;
    return free_area + tolerance >= coarse_area ? 0 : -1;
  }
};

// 严格核对实际浮点格边界，不用米制或相对容差把错位格当成同格。
// 返回目标首格对应的源索引；目标必须完整落在源几何内。
[[nodiscard]] std::optional<std::int64_t> MatchingAxisBegin(
    const double fine_origin, const double coarse_origin,
    const double resolution, const std::int64_t fine_begin,
    const std::int64_t fine_end, const std::int64_t coarse_begin,
    const std::int64_t coarse_end) {
  std::int64_t begin = coarse_begin;
  if (fine_origin != coarse_origin) {
    const double first = CellMinimum(coarse_origin, resolution, coarse_begin);
    // 单调边界二分查找避免浮点商转 int64 的溢出或整数偏移溢出。
    std::int64_t low = fine_begin;
    std::int64_t high = fine_end;
    while (low < high) {
      const std::int64_t mid = std::midpoint(low, high);
      if (CellMinimum(fine_origin, resolution, mid) < first) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }
    begin = low;
  }
  if (begin < fine_begin || begin >= fine_end) return std::nullopt;
  std::int64_t fine_index = begin;
  for (std::int64_t coarse_index = coarse_begin; coarse_index < coarse_end;
       ++coarse_index, ++fine_index) {
    if (fine_index >= fine_end) return std::nullopt;
    const double fine_min = CellMinimum(fine_origin, resolution, fine_index);
    const double coarse_min = CellMinimum(coarse_origin, resolution, coarse_index);
    if (fine_min != coarse_min ||
        fine_min + resolution != coarse_min + resolution ||
        !(fine_min + resolution > fine_min)) {
      return std::nullopt;
    }
  }
  return begin;
}

// 几何验证、索引裁剪和交叠遍历只维护此处；聚合器只决定状态语义。
template <typename Aggregation>
ExplorationMapProjection ProjectImpl(
    const core::FineTraversabilitySnapshot& fine,
    const core::SparseGridGeometry& coarse_geometry) {
  const core::SparseGridGeometry& fine_geometry = fine.geometry();
  if (!coarse_geometry.valid() ||
      coarse_geometry.frame_id() != fine_geometry.frame_id()) {
    throw std::invalid_argument(
        "exploration projection requires compatible map geometry");
  }

  ExplorationMapProjection result{
      .geometry = coarse_geometry,
      .source_fine_traversability_revision =
          fine.fine_traversability_revision(),
      .data = std::vector<std::int8_t>(coarse_geometry.CellCount(), -1)};
  const core::Vec3 fine_origin = fine_geometry.origin_m();
  const core::Vec3 coarse_origin = coarse_geometry.origin_m();
  const double fine_resolution = fine_geometry.resolution_m();
  const double coarse_resolution = coarse_geometry.resolution_m();
  const double coarse_area = coarse_resolution * coarse_resolution;
  const double area_tolerance = coarse_area * 1.0e-9;
  const core::GridIndex coarse_min = coarse_geometry.min_inclusive();
  const core::GridIndex coarse_max = coarse_geometry.max_exclusive();

  // 仅保守出口允许逐状态导出，legacy AnyFree 仍使用原面积聚合。
  if constexpr (std::is_same_v<Aggregation, ConservativeAggregation>) {
    if (fine_resolution == coarse_resolution) {
      const auto fine_x_begin = MatchingAxisBegin(
          fine_origin.x, coarse_origin.x, fine_resolution,
          fine_geometry.min_inclusive().x, fine_geometry.max_exclusive().x,
          coarse_min.x, coarse_max.x);
      const auto fine_y_begin = MatchingAxisBegin(
          fine_origin.y, coarse_origin.y, fine_resolution,
          fine_geometry.min_inclusive().y, fine_geometry.max_exclusive().y,
          coarse_min.y, coarse_max.y);
      if (fine_x_begin && fine_y_begin) {
        std::size_t output = 0U;
        std::int64_t fine_y = *fine_y_begin;
        for (std::int64_t y = coarse_min.y; y < coarse_max.y; ++y, ++fine_y) {
          std::int64_t fine_x = *fine_x_begin;
          for (std::int64_t x = coarse_min.x; x < coarse_max.x; ++x, ++fine_x) {
            const auto state = fine.State({.x = fine_x, .y = fine_y});
            result.data[output++] = state == core::FineCellState::kBlocked ? 100
                : state == core::FineCellState::kFree ? 0 : -1;
          }
        }
        return result;
      }
    }
  }

  std::size_t output_index = 0U;
  for (std::int64_t coarse_y = coarse_min.y; coarse_y < coarse_max.y;
       ++coarse_y) {
    const double coarse_y_min =
        CellMinimum(coarse_origin.y, coarse_resolution, coarse_y);
    const double coarse_y_max = coarse_y_min + coarse_resolution;
    const std::int64_t fine_y_begin = std::max(
        fine_geometry.min_inclusive().y,
        static_cast<std::int64_t>(
            std::floor((coarse_y_min - fine_origin.y) / fine_resolution)));
    const std::int64_t fine_y_end = std::min(
        fine_geometry.max_exclusive().y,
        static_cast<std::int64_t>(
            std::ceil((coarse_y_max - fine_origin.y) / fine_resolution)));

    for (std::int64_t coarse_x = coarse_min.x; coarse_x < coarse_max.x;
         ++coarse_x, ++output_index) {
      const double coarse_x_min =
          CellMinimum(coarse_origin.x, coarse_resolution, coarse_x);
      const double coarse_x_max = coarse_x_min + coarse_resolution;
      const std::int64_t fine_x_begin = std::max(
          fine_geometry.min_inclusive().x,
          static_cast<std::int64_t>(
              std::floor((coarse_x_min - fine_origin.x) / fine_resolution)));
      const std::int64_t fine_x_end = std::min(
          fine_geometry.max_exclusive().x,
          static_cast<std::int64_t>(
              std::ceil((coarse_x_max - fine_origin.x) / fine_resolution)));

      Aggregation aggregation;
      for (std::int64_t fine_y = fine_y_begin;
           fine_y < fine_y_end && !aggregation.Done(); ++fine_y) {
        const double fine_y_min =
            CellMinimum(fine_origin.y, fine_resolution, fine_y);
        const double overlap_y = OverlapLength(
            coarse_y_min, coarse_y_max, fine_y_min,
            fine_y_min + fine_resolution);
        if (overlap_y <= 0.0) {
          continue;
        }
        for (std::int64_t fine_x = fine_x_begin; fine_x < fine_x_end;
             ++fine_x) {
          const double fine_x_min =
              CellMinimum(fine_origin.x, fine_resolution, fine_x);
          const double overlap_x = OverlapLength(
              coarse_x_min, coarse_x_max, fine_x_min,
              fine_x_min + fine_resolution);
          if (overlap_x <= 0.0) {
            continue;
          }
          aggregation.Add(fine.State({.x = fine_x, .y = fine_y}),
                          overlap_x * overlap_y);
          if (aggregation.Done()) {
            break;
          }
        }
      }
      result.data[output_index] =
          aggregation.Occupancy(coarse_area, area_tolerance);
    }
  }
  return result;
}

}  // namespace

ExplorationMapProjection ExplorationMapProjector::Project(
    const core::FineTraversabilitySnapshot& fine,
    const core::SparseGridGeometry& coarse_geometry) const {
  return ProjectImpl<AnyFreeAggregation>(fine, coarse_geometry);
}

ExplorationMapProjection ExplorationMapProjector::ProjectConservative(
    const core::FineTraversabilitySnapshot& fine,
    const core::SparseGridGeometry& coarse_geometry) const {
  return ProjectImpl<ConservativeAggregation>(fine, coarse_geometry);
}

}  // namespace lunar::incremental_navigation_ros
