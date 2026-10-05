#pragma once
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string_view>

namespace lunar::incremental_navigation {
enum class TerrainFitMethod : std::uint8_t {
  kBaseline, kWeightedPlane, kMovingLeastSquares, kGroupedRansac, kGaussianProcess
};
inline constexpr std::string_view TerrainFitMethodName(TerrainFitMethod method) {
  switch (method) {
    case TerrainFitMethod::kBaseline: return "baseline";
    case TerrainFitMethod::kWeightedPlane: return "weighted_plane";
    case TerrainFitMethod::kMovingLeastSquares: return "mls";
    case TerrainFitMethod::kGroupedRansac: return "grouped_ransac";
    case TerrainFitMethod::kGaussianProcess: return "gaussian_process";
  }
  return "invalid";
}
inline TerrainFitMethod ParseTerrainFitMethod(std::string_view name) {
  for (auto method : {TerrainFitMethod::kBaseline, TerrainFitMethod::kWeightedPlane,
       TerrainFitMethod::kMovingLeastSquares, TerrainFitMethod::kGroupedRansac,
       TerrainFitMethod::kGaussianProcess})
    if (TerrainFitMethodName(method) == name) return method;
  throw std::invalid_argument("unknown terrain fitting method");
}
}  // namespace lunar::incremental_navigation
