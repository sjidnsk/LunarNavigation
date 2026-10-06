#include "map/terrain_surface_fit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <numbers>
#include <optional>
#include <tuple>
#include <vector>

namespace lunar::incremental_navigation::map {
namespace {
using Indices = std::vector<std::size_t>;

struct Plane {
  double c{}, a{}, b{};
  double At(const SurfaceSample& p) const { return c + a * p.x + b * p.y; }
};

std::optional<Plane> FitPlane(const std::vector<SurfaceSample>& points,
                              const Indices& indices) {
  if (indices.size() < 3) return {};
  double mx = 0., my = 0., mz = 0.;
  for (auto i : indices) {
    mx += points[i].x;
    my += points[i].y;
    mz += points[i].z;
  }
  mx /= indices.size();
  my /= indices.size();
  mz /= indices.size();
  double xx = 0., xy = 0., yy = 0., xz = 0., yz = 0.;
  for (auto i : indices) {
    const double x = points[i].x - mx;
    const double y = points[i].y - my;
    const double z = points[i].z - mz;
    xx += x * x;
    xy += x * y;
    yy += y * y;
    xz += x * z;
    yz += y * z;
  }
  const double trace = xx + yy;
  const double det = xx * yy - xy * xy;
  if (!(trace > 0.) || det <= 1e-4 * trace * trace) return {};
  const double a = (xz * yy - yz * xy) / det;
  const double b = (yz * xx - xz * xy) / det;
  const Plane fit{mz - a * mx - b * my, a, b};
  if (!std::isfinite(fit.c) || !std::isfinite(a) || !std::isfinite(b)) return {};
  return fit;
}

std::optional<Plane> Hypothesis(const SurfaceSample& p, const SurfaceSample& q,
                                const SurfaceSample& r) {
  const double qx = q.x - p.x, qy = q.y - p.y;
  const double rx = r.x - p.x, ry = r.y - p.y;
  const double det = qx * ry - rx * qy;
  const double scale = std::max(qx * qx + qy * qy, rx * rx + ry * ry);
  if (!(scale > 0.) || std::abs(det) <= 1e-4 * scale) return {};
  const double a = ((q.z - p.z) * ry - (r.z - p.z) * qy) / det;
  const double b = ((r.z - p.z) * qx - (q.z - p.z) * rx) / det;
  const Plane plane{p.z - a * p.x - b * p.y, a, b};
  if (!std::isfinite(plane.c) || !std::isfinite(a) || !std::isfinite(b)) return {};
  return plane;
}

Indices PlaneInliers(const std::vector<SurfaceSample>& points, const Indices& active,
                     const Plane& plane, double limit) {
  Indices inliers;
  for (auto i : active)
    if (std::abs(points[i].z - plane.At(points[i])) <= limit) inliers.push_back(i);
  return inliers;
}

// Fixed-seed hypotheses and coordinate sorting make fits reproducible and
// independent of the order in which the map supplied its measurements.
std::optional<std::pair<Plane, Indices>> ExtractPlane(
    const std::vector<SurfaceSample>& points, const Indices& active, double limit,
    std::uint64_t salt) {
  if (active.size() < 4) return {};
  std::optional<Plane> best;
  Indices best_indices;
  double best_error = std::numeric_limits<double>::infinity();
  const auto Consider = [&](const std::optional<Plane>& candidate) {
    if (!candidate) return;
    Indices inliers = PlaneInliers(points, active, *candidate, limit);
    double error = 0.;
    for (auto i : inliers) {
      const double residual = points[i].z - candidate->At(points[i]);
      error += residual * residual;
    }
    if (inliers.size() > best_indices.size() ||
        (inliers.size() == best_indices.size() && error < best_error)) {
      best = candidate;
      best_indices = std::move(inliers);
      best_error = error;
    }
  };
  Consider(FitPlane(points, active));
  // Least squares already minimizes squared error. If every observation lies
  // within the residual threshold, no other hypothesis can improve consensus.
  if (best && best_indices.size() == active.size())
    return std::pair{*best, std::move(best_indices)};
  // Small patches enumerate every triple. Large patches use a bounded 128
  // hypotheses per extracted surface, rather than a data-dependent runtime.
  if (active.size() <= 10) {
    for (std::size_t i = 0; i < active.size(); ++i)
      for (std::size_t j = i + 1; j < active.size(); ++j)
        for (std::size_t k = j + 1; k < active.size(); ++k)
          Consider(Hypothesis(points[active[i]], points[active[j]], points[active[k]]));
  } else {
    std::uint64_t state = 0x6a09e667f3bcc909ULL ^ salt ^ active.size();
    const auto Pick = [&]() {
      state = state * 6364136223846793005ULL + 1442695040888963407ULL;
      return static_cast<std::size_t>((state >> 32) % active.size());
    };
    for (int iteration = 0; iteration < 128; ++iteration) {
      const auto i = Pick(), j = Pick(), k = Pick();
      if (i == j || i == k || j == k) continue;
      Consider(Hypothesis(points[active[i]], points[active[j]], points[active[k]]));
    }
  }
  if (!best || best_indices.size() < 4) return {};
  for (int iteration = 0; iteration < 2; ++iteration) {
    auto refined = FitPlane(points, best_indices);
    if (!refined) return {};
    best = refined;
    best_indices = PlaneInliers(points, active, *best, limit);
    if (best_indices.size() < 4) return {};
  }
  best = FitPlane(points, best_indices);
  if (!best) return {};
  return std::pair{*best, std::move(best_indices)};
}

std::vector<Indices> SpatialComponents(const std::vector<SurfaceSample>& points,
                                      const Indices& indices, double link_distance) {
  const double limit_squared = link_distance * link_distance;
  std::vector<bool> visited(indices.size(), false);
  std::vector<Indices> groups;
  for (std::size_t start = 0; start < indices.size(); ++start) {
    if (visited[start]) continue;
    visited[start] = true;
    std::vector<std::size_t> queue{start};
    Indices group;
    for (std::size_t offset = 0; offset < queue.size(); ++offset) {
      const auto current = queue[offset];
      group.push_back(indices[current]);
      for (std::size_t next = 0; next < indices.size(); ++next) {
        if (visited[next]) continue;
        const double dx = points[indices[current]].x - points[indices[next]].x;
        const double dy = points[indices[current]].y - points[indices[next]].y;
        if (dx * dx + dy * dy <= limit_squared + 1e-12) {
          visited[next] = true;
          queue.push_back(next);
        }
      }
    }
    std::sort(group.begin(), group.end());
    groups.push_back(std::move(group));
  }
  return groups;
}

bool SurroundsOrigin(const std::vector<SurfaceSample>& points, const Indices& group,
                     double maximum_nearest_distance) {
  std::vector<double> angles;
  double nearest = std::numeric_limits<double>::infinity();
  for (auto i : group) {
    nearest = std::min(nearest, std::hypot(points[i].x, points[i].y));
    if (std::hypot(points[i].x, points[i].y) > 1e-9)
      angles.push_back(std::atan2(points[i].y, points[i].x));
  }
  if (nearest > maximum_nearest_distance || angles.size() < 3) return false;
  std::sort(angles.begin(), angles.end());
  double maximum_gap = angles.front() + 2. * std::numbers::pi - angles.back();
  for (std::size_t i = 1; i < angles.size(); ++i)
    maximum_gap = std::max(maximum_gap, angles[i] - angles[i - 1]);
  // Strictly interior support. Even a measured origin does not license
  // extending a one-sided component into unobserved terrain.
  return maximum_gap < std::numbers::pi - 1e-9;
}

struct Candidate {
  Plane plane;
  Indices indices;
  double local_weight{};
};
}  // namespace

std::optional<SurfaceFit> FitGroupedRansac(const std::vector<SurfaceSample>& points,
                                         const SurfaceFitOptions& options) {
  if (points.size() < 4 || !std::isfinite(options.support_distance_m) ||
      !std::isfinite(options.resolution_m) || !std::isfinite(options.residual_limit_m) ||
      options.support_distance_m <= 0. || options.resolution_m <= 0. ||
      options.residual_limit_m <= 0.) return {};
  for (const auto& p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(p.low) || !std::isfinite(p.high) || p.low > p.high) return {};
  }
  Indices active(points.size());
  std::iota(active.begin(), active.end(), 0);
  std::sort(active.begin(), active.end(), [&](auto lhs, auto rhs) {
    const auto& a = points[lhs];
    const auto& b = points[rhs];
    return std::tie(a.x, a.y, a.z, a.low, a.high) < std::tie(b.x, b.y, b.z, b.low, b.high);
  });
  const Indices all_indices = active;
  std::vector<Candidate> candidates;
  const double link_distance = std::max(2.1 * options.resolution_m,
                                         .8 * options.support_distance_m);
  for (int surface = 0; surface < 4 && active.size() >= 4; ++surface) {
    auto extracted = ExtractPlane(points, active, options.residual_limit_m,
                                  static_cast<std::uint64_t>(surface));
    if (!extracted) break;
    const auto& plane_inliers = extracted->second;
    for (auto& group : SpatialComponents(points, plane_inliers, link_distance)) {
      if (group.size() < 4) continue;
      auto local_plane = FitPlane(points, group);
      if (!local_plane) continue;
      // A broad RANSAC hypothesis can gather two banks while omitting their
      // intervening near-origin measurements. Refit each connected fragment,
      // then recruit its agreeing raw measurements across all extraction
      // rounds. This restores one actual plane split by extraction order;
      // the second connectivity split still preserves unsupported gaps.
      const Indices recruited = PlaneInliers(points, all_indices, *local_plane,
                                             options.residual_limit_m);
      for (auto& local_group : SpatialComponents(points, recruited, link_distance)) {
        if (local_group.size() < 4) continue;
        auto model = FitPlane(points, local_group);
        if (!model) continue;
        for (int iteration = 0; iteration < 2; ++iteration) {
          local_group = PlaneInliers(points, local_group, *model,
                                     options.residual_limit_m);
          model = FitPlane(points, local_group);
          if (!model) break;
        }
        if (!model || local_group.size() < 4 ||
            !SurroundsOrigin(points, local_group, options.support_distance_m)) continue;
        const bool duplicate = std::any_of(candidates.begin(), candidates.end(),
            [&](const Candidate& candidate) { return candidate.indices == local_group; });
        if (duplicate) continue;
        double local_weight = 0.;
        for (auto i : local_group) {
          const double range_squared = points[i].x * points[i].x + points[i].y * points[i].y;
          local_weight += 1. / (1. + range_squared /
                                       (options.support_distance_m * options.support_distance_m));
        }
        candidates.push_back({*model, std::move(local_group), local_weight});
      }
    }
    std::vector<bool> removed(points.size(), false);
    for (auto i : plane_inliers) removed[i] = true;
    active.erase(std::remove_if(active.begin(), active.end(),
                                [&](auto i) { return removed[i]; }), active.end());
  }
  if (candidates.empty()) return {};
  const auto selected = std::max_element(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) { return a.local_weight < b.local_weight; });
  // Overlapping independently supported layers are ambiguous at the target.
  // Picking the most numerous layer would silently suppress the other layer.
  for (const auto& candidate : candidates) {
    if (std::abs(candidate.plane.c - selected->plane.c) > options.residual_limit_m)
      return {};
  }
  SurfaceFit result;
  result.coefficients = {selected->plane.c, selected->plane.a, selected->plane.b, 0., 0., 0.};
  result.inliers.assign(points.size(), false);
  for (auto i : selected->indices) result.inliers[i] = true;
  return result;
}
}  // namespace lunar::incremental_navigation::map
