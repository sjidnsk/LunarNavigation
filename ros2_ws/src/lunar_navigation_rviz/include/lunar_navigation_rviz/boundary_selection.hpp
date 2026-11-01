#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>
namespace lunar_navigation_rviz {
// 仅负责选区几何；不参与导航或探索决策。
class BoundarySelection {
public:
  using Point = std::pair<double, double>;
  bool add(const std::string & frame, double x, double y) {
    if (closed_ || frame != "map" || !std::isfinite(x) || !std::isfinite(y) ||
        std::abs(x) > std::numeric_limits<float>::max() ||
        std::abs(y) > std::numeric_limits<float>::max()) return false;
    // 按实际geometry_msgs/Point32精度校验，避免序列化后顶点重合。
    x = static_cast<float>(x); y = static_cast<float>(y);
    Point point{x, y};
    for (const auto & p : points_) {
      if (std::hypot(p.first - x, p.second - y) < 1e-6) return false;
    }
    points_.push_back(point);
    return true;
  }
  bool close() {
    if (points_.size() < 3 || area() < 1e-6) return false;
    const auto n = points_.size();
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        if (j == i + 1 || (i == 0 && j == n - 1)) continue;
        if (intersects(points_[i], points_[(i + 1) % n], points_[j], points_[(j + 1) % n])) return false;
      }
    }
    closed_ = true;
    return true;
  }
  void clear() { points_.clear(); closed_ = false; }
  void undo() { if (!points_.empty()) points_.pop_back(); closed_ = false; }
  bool closed() const { return closed_; }
  double area() const {
    if (points_.size() < 3) return 0.;
    double sum = 0.;
    // 平移后计算，降低远离原点时的消减误差。
    const auto origin = points_.front();
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const auto & a = points_[i]; const auto & b = points_[(i + 1) % points_.size()];
      sum += (a.first - origin.first) * (b.second - origin.second) -
        (b.first - origin.first) * (a.second - origin.second);
    }
    return std::abs(sum) * 0.5;
  }
  const std::vector<Point> & points() const { return points_; }
private:
  static double cross(Point a, Point b, Point c) {
    return (b.first-a.first)*(c.second-a.second)-(b.second-a.second)*(c.first-a.first);
  }
  static bool onSegment(Point a, Point b, Point c) {
    return std::abs(cross(a,b,c)) < 1e-9 &&
      c.first >= std::min(a.first,b.first)-1e-9 && c.first <= std::max(a.first,b.first)+1e-9 &&
      c.second >= std::min(a.second,b.second)-1e-9 && c.second <= std::max(a.second,b.second)+1e-9;
  }
  static bool intersects(Point a, Point b, Point c, Point d) {
    const auto ab_c=cross(a,b,c), ab_d=cross(a,b,d), cd_a=cross(c,d,a), cd_b=cross(c,d,b);
    return ((ab_c > 0) != (ab_d > 0) && (cd_a > 0) != (cd_b > 0)) ||
      onSegment(a,b,c) || onSegment(a,b,d) || onSegment(c,d,a) || onSegment(c,d,b);
  }
  std::vector<Point> points_;
  bool closed_{false};
};
}
