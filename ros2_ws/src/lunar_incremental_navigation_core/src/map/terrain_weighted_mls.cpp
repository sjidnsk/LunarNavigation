#include "terrain_surface_fit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lunar::incremental_navigation::map {
namespace {
bool ValidInput(const std::vector<SurfaceSample>& samples,
                const SurfaceFitOptions& options) {
  if (samples.size() < 4 || !std::isfinite(options.support_distance_m) ||
      options.support_distance_m <= 0. || !std::isfinite(options.resolution_m) ||
      options.resolution_m <= 0. || !std::isfinite(options.residual_limit_m) ||
      options.residual_limit_m <= 0.) return false;
  for (const auto& sample : samples) {
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) ||
        !std::isfinite(sample.z) || !std::isfinite(sample.low) ||
        !std::isfinite(sample.high) || sample.high < sample.low) return false;
  }
  return true;
}

std::array<double, 6> Features(const SurfaceSample& sample, double scale) {
  const double x = sample.x / scale, y = sample.y / scale;
  return {1., x, y, x*x, x*y, y*y};
}

std::vector<double> EvidenceWeights(const std::vector<SurfaceSample>& samples,
                                    const SurfaceFitOptions& options) {
  std::vector<double> weights;
  weights.reserve(samples.size());
  for (const auto& sample : samples) {
    const double x = sample.x / options.support_distance_m;
    const double y = sample.y / options.support_distance_m;
    // The raw interval width is a quality proxy, not a calibrated sensor
    // variance. All methods retain the raw extrema in the common hazard guards.
    const double relative_width = (sample.high - sample.low) /
                                  (2. * options.residual_limit_m);
    weights.push_back(std::exp(-.5 * (x*x + y*y)) /
                      (1. + relative_width*relative_width));
  }
  return weights;
}

template <std::size_t N>
std::optional<std::array<double, 6>> WeightedLeastSquares(
    const std::vector<SurfaceSample>& samples, const std::vector<double>& weights,
    double scale) {
  std::array<std::array<double, N>, N> normal{};
  std::array<double, N> right{};
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto basis = Features(samples[i], scale);
    for (std::size_t row = 0; row < N; ++row) {
      right[row] += weights[i] * basis[row] * samples[i].z;
      for (std::size_t col = 0; col <= row; ++col)
        normal[row][col] += weights[i] * basis[row] * basis[col];
    }
  }
  for (std::size_t row = 0; row < N; ++row)
    for (std::size_t col = 0; col < row; ++col) normal[col][row] = normal[row][col];

  // Normalized coordinates keep metres and squared metres from creating a
  // purely numerical rank deficiency. Reject unresolved coefficients instead
  // of inventing curvature with a ridge penalty.
  double maximum_diagonal = 0.;
  for (std::size_t i = 0; i < N; ++i)
    maximum_diagonal = std::max(maximum_diagonal, normal[i][i]);
  if (!std::isfinite(maximum_diagonal) || maximum_diagonal <= 0.) return {};
  std::array<std::array<double, N>, N> lower{};
  for (std::size_t row = 0; row < N; ++row) {
    for (std::size_t col = 0; col <= row; ++col) {
      double remainder = normal[row][col];
      for (std::size_t k = 0; k < col; ++k)
        remainder -= lower[row][k] * lower[col][k];
      if (row == col) {
        if (!std::isfinite(remainder) || remainder <= 1e-8 * maximum_diagonal)
          return {};
        lower[row][col] = std::sqrt(remainder);
      } else {
        lower[row][col] = remainder / lower[col][col];
      }
    }
  }
  std::array<double, N> intermediate{};
  for (std::size_t row = 0; row < N; ++row) {
    double remainder = right[row];
    for (std::size_t col = 0; col < row; ++col)
      remainder -= lower[row][col] * intermediate[col];
    intermediate[row] = remainder / lower[row][row];
  }
  std::array<double, 6> coefficients{};
  for (std::size_t row = N; row-- > 0;) {
    double remainder = intermediate[row];
    for (std::size_t col = row + 1; col < N; ++col)
      remainder -= lower[col][row] * coefficients[col];
    coefficients[row] = remainder / lower[row][row];
    if (!std::isfinite(coefficients[row])) return {};
  }
  coefficients[1] /= scale;
  coefficients[2] /= scale;
  for (std::size_t i = 3; i < N; ++i) coefficients[i] /= scale*scale;
  for (double coefficient : coefficients) if (!std::isfinite(coefficient)) return {};
  return coefficients;
}

template <std::size_t N>
std::optional<SurfaceFit> RobustFit(const std::vector<SurfaceSample>& samples,
                                  const SurfaceFitOptions& options) {
  const auto base_weights = EvidenceWeights(samples, options);
  auto weights = base_weights;
  auto coefficients = WeightedLeastSquares<N>(samples, weights, options.support_distance_m);
  if (!coefficients) return {};
  SurfaceFit result;
  result.coefficients = *coefficients;
  // Same robust transition scale as the current baseline, with fixed iteration
  // count. Distance and interval quality weights remain present at every step.
  const double huber_transition = .5 * options.residual_limit_m;
  for (int iteration = 0; iteration < 6; ++iteration) {
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const double residual = std::abs(samples[i].z - result.Height(samples[i].x, samples[i].y));
      weights[i] = base_weights[i] *
                   std::min(1., huber_transition / std::max(1e-12, residual));
    }
    coefficients = WeightedLeastSquares<N>(samples, weights, options.support_distance_m);
    if (!coefficients) return {};
    result.coefficients = *coefficients;
  }
  result.inliers.reserve(samples.size());
  for (const auto& sample : samples)
    result.inliers.push_back(std::abs(sample.z - result.Height(sample.x, sample.y)) <=
                             options.residual_limit_m);
  return result;
}
}  // namespace

std::optional<SurfaceFit> FitWeightedPlane(const std::vector<SurfaceSample>& samples,
                                         const SurfaceFitOptions& options) {
  if (!ValidInput(samples, options)) return {};
  return RobustFit<3>(samples, options);
}

std::optional<SurfaceFit> FitMovingLeastSquares(const std::vector<SurfaceSample>& samples,
                                              const SurfaceFitOptions& options) {
  if (!ValidInput(samples, options)) return {};
  if (samples.size() >= 6) {
    if (auto quadratic = RobustFit<6>(samples, options)) return quadratic;
  }
  auto plane = FitWeightedPlane(samples, options);
  if (plane) plane->used_fallback = true;
  return plane;
}
}  // namespace lunar::incremental_navigation::map
