#include "map/terrain_sample_filter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lunar::incremental_navigation::map {
namespace {
constexpr double kNeighbourRadiusM=.10;
constexpr double kSpatialSigmaM=.075;
constexpr double kHeightSigmaM=.03;
constexpr std::size_t kMinimumNeighbours=3;
}  // namespace

std::vector<SurfaceSample> FilterTerrainSamples(
    const std::vector<SurfaceSample>& raw, TerrainFilterMethod method) {
  if(TerrainFilterMethodName(method)=="invalid")
    throw std::invalid_argument("invalid terrain sample filter");
  auto filtered=raw;
  // Keep none an exact copy, including signed zero and NaN representations.
  if(method==TerrainFilterMethod::kNone) return filtered;
  for(const auto& p:raw)
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       !std::isfinite(p.low)||!std::isfinite(p.high)||p.high<p.low)
      throw std::invalid_argument("invalid terrain filter sample");
  std::vector<double> heights;
  heights.reserve(raw.size());
  for(std::size_t i=0;i<raw.size();++i) {
    const auto& center=raw[i];
    heights.clear();
    double weighted_height=0.,weight_sum=0.;
    for(const auto& p:raw) {
      const double dx=p.x-center.x,dy=p.y-center.y;
      // The small tolerance includes exact lattice-boundary neighbours despite
      // floating-point subtraction, without extending the physical radius.
      if(std::abs(dx)>kNeighbourRadiusM+1e-12||std::abs(dy)>kNeighbourRadiusM+1e-12)
        continue;
      const double distance2=dx*dx+dy*dy;
      if(distance2>kNeighbourRadiusM*kNeighbourRadiusM+1e-12) continue;
      heights.push_back(p.z);
      if(method==TerrainFilterMethod::kBilateral) {
        const double dz=p.z-center.z;
        const double weight=std::exp(-.5*distance2/(kSpatialSigmaM*kSpatialSigmaM)-
            .5*dz*dz/(kHeightSigmaM*kHeightSigmaM));
        weighted_height+=weight*p.z;
        weight_sum+=weight;
      }
    }
    if(heights.size()<kMinimumNeighbours) continue;
    if(method==TerrainFilterMethod::kMedian) {
      std::sort(heights.begin(),heights.end());
      const std::size_t middle=heights.size()/2;
      filtered[i].z=heights.size()%2?heights[middle]:
          .5*heights[middle-1]+.5*heights[middle];
    } else {
      // Self always has weight 1, so no synthetic or unsupported sample is
      // required to make the denominator positive.
      filtered[i].z=weighted_height/weight_sum;
    }
    if(!std::isfinite(filtered[i].z))
      throw std::invalid_argument("non-finite filtered terrain height");
  }
  return filtered;
}
}  // namespace lunar::incremental_navigation::map
