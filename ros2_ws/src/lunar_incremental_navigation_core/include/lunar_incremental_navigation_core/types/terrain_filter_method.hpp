#pragma once
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string_view>

namespace lunar::incremental_navigation {
enum class TerrainFilterMethod : std::uint8_t { kNone, kMedian, kBilateral };
inline constexpr std::string_view TerrainFilterMethodName(TerrainFilterMethod method) {
  switch(method) {
    case TerrainFilterMethod::kNone: return "none";
    case TerrainFilterMethod::kMedian: return "median";
    case TerrainFilterMethod::kBilateral: return "bilateral";
  }
  return "invalid";
}
inline TerrainFilterMethod ParseTerrainFilterMethod(std::string_view name) {
  for(auto method:{TerrainFilterMethod::kNone,TerrainFilterMethod::kMedian,
                   TerrainFilterMethod::kBilateral})
    if(TerrainFilterMethodName(method)==name) return method;
  throw std::invalid_argument("unknown terrain sample filter");
}
}  // namespace lunar::incremental_navigation
