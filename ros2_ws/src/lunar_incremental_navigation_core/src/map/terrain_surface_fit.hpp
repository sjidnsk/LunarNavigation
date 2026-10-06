#pragma once
#include <array>
#include <optional>
#include <vector>
#include "lunar_incremental_navigation_core/types/geometry.hpp"
#include "lunar_incremental_navigation_core/types/terrain_fit_method.hpp"

namespace lunar::incremental_navigation::map {
struct SurfaceSample { double x{}, y{}, low{}, high{}, z{}; };
struct SurfaceFitOptions {
  double support_distance_m{.20};
  double resolution_m{.05};
  double residual_limit_m{.03};
};
// Coefficients use metres: c + ax + by + qxx*x*x + qxy*x*y + qyy*y*y.
// GP uses this polynomial as its mean, plus an RBF residual model.
struct SurfaceFit {
  std::array<double, 6> coefficients{};
  std::vector<SurfaceSample> gp_samples;
  std::vector<double> gp_alpha;
  double gp_length_m{.12};
  double gp_signal_variance{.0004};
  double predictive_variance_m2{};
  bool used_fallback{};
  std::vector<bool> inliers;
  double Height(double x, double y) const;
  Vec2 Gradient(double x, double y) const;
};
std::optional<SurfaceFit> FitWeightedPlane(const std::vector<SurfaceSample>&, const SurfaceFitOptions&);
std::optional<SurfaceFit> FitMovingLeastSquares(const std::vector<SurfaceSample>&, const SurfaceFitOptions&);
std::optional<SurfaceFit> FitGroupedRansac(const std::vector<SurfaceSample>&, const SurfaceFitOptions&);
std::optional<SurfaceFit> FitGaussianProcess(const std::vector<SurfaceSample>&, const SurfaceFitOptions&);
std::optional<SurfaceFit> FitTerrainSurface(TerrainFitMethod, const std::vector<SurfaceSample>&, const SurfaceFitOptions&);
}  // namespace lunar::incremental_navigation::map
