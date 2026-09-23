#pragma once

#include <array>
#include <memory>
#include <vector>

#include "lunar_incremental_navigation_core/local_planning.hpp"

namespace lunar::incremental_navigation::local {

// A cache belongs to exactly one immutable request view, including its start
// overrides. Allocate only pages touched by this request, not a full raster.
class RequestCellCache final {
public:
  explicit RequestCellCache(const RequestLocalPlanningView &view)
      : view_(view), elevation_(view.base()->elevation()),
        lo_(view.geometry().min_inclusive()),
        hi_(view.geometry().max_exclusive()), width_(view.geometry().width()),
        pages_((view.geometry().CellCount() + kPageSize - 1) / kPageSize) {}

  const SparseGridGeometry &geometry() const noexcept {
    return view_.geometry();
  }

  LocalCellSource Source(GridIndex cell) {
    auto *entry = EntryAt(cell);
    if (!entry)
      return LocalCellSource::kUnknown;
    if (!(entry->ready & kSource)) {
      entry->source = view_.Source(cell);
      entry->ready |= kSource;
    }
    return entry->source;
  }

  double TraversalCost(GridIndex cell) {
    auto *entry = EntryAt(cell);
    if (!entry)
      return 0.;
    if (!(entry->ready & kCost)) {
      entry->cost = view_.TraversalCost(cell);
      entry->ready |= kCost;
    }
    return entry->cost;
  }

  std::optional<ElevationRange> ElevationRangeAt(GridIndex cell) {
    auto *entry = EntryAt(cell);
    // Elevation has the base snapshot's extent, not the request window's.
    if (!entry)
      return elevation_->ElevationRangeAt(cell);
    if (!(entry->ready & kElevation)) {
      const auto range = elevation_->ElevationRangeAt(cell);
      if (range) {
        entry->range = *range;
        entry->ready |= kKnownElevation;
      }
      entry->ready |= kElevation;
    }
    return entry->ready & kKnownElevation
               ? std::optional<ElevationRange>(entry->range)
               : std::nullopt;
  }

  std::optional<StartPhase> AdvancePhase(StartPhase phase, GridIndex cell) {
    switch (Source(cell)) {
    case LocalCellSource::kEvidenceFree:
      return StartPhase::kNormal;
    case LocalCellSource::kStartAssumedFree:
      return phase == StartPhase::kStartPrefix
                 ? std::optional<StartPhase>(phase)
                 : std::nullopt;
    case LocalCellSource::kUnknown:
    case LocalCellSource::kBlocked:
      return std::nullopt;
    }
    return std::nullopt;
  }

private:
  static constexpr std::size_t kPageSize = 256;
  static constexpr unsigned char kSource = 1, kCost = 2, kElevation = 4,
                                 kKnownElevation = 8;
  struct Entry {
    double cost{};
    ElevationRange range{};
    LocalCellSource source{};
    unsigned char ready{};
  };
  using Page = std::array<Entry, kPageSize>;
  Entry *EntryAt(GridIndex cell) {
    if (cell.x < lo_.x || cell.x >= hi_.x || cell.y < lo_.y || cell.y >= hi_.y)
      return nullptr;
    const auto offset = static_cast<std::size_t>(cell.y - lo_.y) * width_ +
                        static_cast<std::size_t>(cell.x - lo_.x);
    auto &page = pages_[offset / kPageSize];
    if (!page)
      page = std::make_unique<Page>();
    return &(*page)[offset % kPageSize];
  }
  const RequestLocalPlanningView &view_;
  std::shared_ptr<const ElevationSnapshot> elevation_;
  GridIndex lo_, hi_;
  std::size_t width_;
  std::vector<std::unique_ptr<Page>> pages_;
};
} // namespace lunar::incremental_navigation::local
