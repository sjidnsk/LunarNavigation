#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "lunar_incremental_navigation_core/types/platform_capability.hpp"

namespace lunar::incremental_navigation::convex_polygon {
using Polygon = std::vector<Point2>;
inline double Cross(Point2 a, Point2 b, Point2 p) {
  // 两项乘积各自舍入，避免ARM的FMA收缩把Cross(a,b,b)变成微小负数。
  // 这不是面积容差；保持原有零阈值和跨平台一致的乘积差语义。
  volatile double left = (b.x - a.x) * (p.y - a.y);
  volatile double right = (b.y - a.y) * (p.x - a.x);
  return left - right;
}
inline double Area(const Polygon& p) {
  if (p.size() < 3) return 0.;
  double sum = 0.;
  for (std::size_t i = 1; i + 1 < p.size(); ++i)
    sum += Cross(p[0], p[i], p[i + 1]);
  return std::abs(sum) * .5;
}
inline Polygon Clip(const Polygon& p, Point2 a, Point2 b,
             bool inside = true) {
  Polygon result;
  if (p.empty()) return result;
  auto previous = p.back();
  double d0 = Cross(a, b, previous) * (inside ? 1. : -1.);
  for (const auto current : p) {
    double d1 = Cross(a, b, current) * (inside ? 1. : -1.);
    if ((d0 >= 0.) != (d1 >= 0.)) {
      const double t = d0 / (d0 - d1);
      result.push_back({previous.x + t * (current.x - previous.x),
                        previous.y + t * (current.y - previous.y)});
    }
    if (d1 >= 0.) result.push_back(current);
    previous = current;
    d0 = d1;
  }
  return result;
}
inline Polygon Intersect(Polygon a, const Polygon& b) {
  for (std::size_t i = 0; i < b.size(); ++i)
    a = Clip(a, b[i], b[(i + 1) % b.size()]);
  return a;
}
inline std::vector<Polygon> Subtract(Polygon a, const Polygon& b, double epsilon) {
  std::vector<Polygon> result;
  for (std::size_t i = 0; i < b.size() && !a.empty(); ++i) {
    auto outside = Clip(a, b[i], b[(i + 1) % b.size()], false);
    if (Area(outside) > epsilon) result.push_back(std::move(outside));
    a = Clip(a, b[i], b[(i + 1) % b.size()]);
  }
  return result;
}
}  // namespace lunar::incremental_navigation::convex_polygon
