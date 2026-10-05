#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "lunar_incremental_navigation_core/search_control.hpp"

namespace lunar::incremental_navigation::local {
struct BoundaryPoint {
  long double x, y;
};
struct BoundarySegment {
  BoundaryPoint a, b;
};

// Exact nearest boundary for segments already certified disjoint by supercover.
// All arithmetic remains in the simplifier's long-double cell coordinates.
class BoundaryDistanceIndex final {
public:
  bool Build(std::vector<BoundarySegment> edges, const SearchControl &control) {
    edges_ = std::move(edges);
    nodes_.clear();
    ready_ = false;
    if (Interrupted(control))
      return false;
    if (!edges_.empty()) {
      nodes_.reserve(edges_.size() * 2);
      if (!BuildNode(0, edges_.size(), control))
        return false;
    }
    ready_ = true;
    return true;
  }

  std::optional<long double> Query(BoundarySegment line,
                                   const SearchControl &control) const {
    if (!ready_ || Interrupted(control))
      return std::nullopt;
    long double best = std::numeric_limits<long double>::infinity();
    if (!nodes_.empty() && !Visit(0, line, Bounds(line), best, control))
      return std::nullopt;
    return best;
  }

private:
  struct Box {
    long double xmin, ymin, xmax, ymax;
  };
  struct Node {
    Box box;
    std::size_t begin, end, left{}, right{};
  };
  static bool Interrupted(const SearchControl &c) {
    return c.canceled() || c.expired();
  }
  static Box Bounds(BoundarySegment s) {
    return {std::min(s.a.x, s.b.x), std::min(s.a.y, s.b.y),
            std::max(s.a.x, s.b.x), std::max(s.a.y, s.b.y)};
  }
  static Box Union(Box a, Box b) {
    return {std::min(a.xmin, b.xmin), std::min(a.ymin, b.ymin),
            std::max(a.xmax, b.xmax), std::max(a.ymax, b.ymax)};
  }
  static long double LowerBound(Box a, Box b) {
    const auto dx = std::max({0.L, a.xmin - b.xmax, b.xmin - a.xmax});
    const auto dy = std::max({0.L, a.ymin - b.ymax, b.ymin - a.ymax});
    return dx * dx + dy * dy;
  }
  static bool CanImprove(long double bound, long double best) {
    // Keep roundoff-near ties for the original exact metric to decide.
    return bound <= best + 64 * std::numeric_limits<long double>::epsilon() *
                               std::max(1.L, best);
  }
  static long double PointDistanceSquared(BoundaryPoint p, BoundarySegment s) {
    const long double dx = s.b.x - s.a.x, dy = s.b.y - s.a.y,
                      length = dx * dx + dy * dy;
    const long double t =
        length > 0
            ? std::clamp(((p.x - s.a.x) * dx + (p.y - s.a.y) * dy) / length,
                         0.L, 1.L)
            : 0;
    const long double ex = p.x - s.a.x - t * dx, ey = p.y - s.a.y - t * dy;
    return ex * ex + ey * ey;
  }
  static long double DistanceSquared(BoundarySegment a, BoundarySegment b) {
    return std::min({PointDistanceSquared(a.a, b), PointDistanceSquared(a.b, b),
                     PointDistanceSquared(b.a, a),
                     PointDistanceSquared(b.b, a)});
  }
  bool BuildNode(std::size_t begin, std::size_t end, const SearchControl &c) {
    if (Interrupted(c))
      return false;
    Box box = Bounds(edges_[begin]);
    for (auto i = begin + 1; i < end; ++i) {
      if ((i - begin) % 256 == 0 && Interrupted(c))
        return false;
      box = Union(box, Bounds(edges_[i]));
    }
    const auto index = nodes_.size();
    nodes_.push_back({box, begin, end});
    if (end - begin <= 8)
      return true;
    const bool x = (box.xmax - box.xmin) >= (box.ymax - box.ymin);
    const auto mid = begin + (end - begin) / 2;
    std::nth_element(edges_.begin() + begin, edges_.begin() + mid,
                     edges_.begin() + end,
                     [x](BoundarySegment a, BoundarySegment b) {
                       return x ? a.a.x + a.b.x < b.a.x + b.b.x
                                : a.a.y + a.b.y < b.a.y + b.b.y;
                     });
    nodes_[index].left = nodes_.size();
    if (!BuildNode(begin, mid, c))
      return false;
    nodes_[index].right = nodes_.size();
    return BuildNode(mid, end, c);
  }
  bool Visit(std::size_t index, BoundarySegment line, Box box,
             long double &best, const SearchControl &c) const {
    if (Interrupted(c))
      return false;
    const auto &node = nodes_[index];
    if (!CanImprove(LowerBound(box, node.box), best))
      return true;
    if (node.end - node.begin <= 8) {
      for (auto i = node.begin; i < node.end; ++i)
        if (CanImprove(LowerBound(box, Bounds(edges_[i])), best))
          best = std::min(best, DistanceSquared(line, edges_[i]));
      return true;
    }
    auto first = node.left, second = node.right;
    if (LowerBound(box, nodes_[second].box) <
        LowerBound(box, nodes_[first].box))
      std::swap(first, second);
    return Visit(first, line, box, best, c) &&
           Visit(second, line, box, best, c);
  }
  bool ready_{};
  std::vector<BoundarySegment> edges_;
  std::vector<Node> nodes_;
};
} // namespace lunar::incremental_navigation::local
