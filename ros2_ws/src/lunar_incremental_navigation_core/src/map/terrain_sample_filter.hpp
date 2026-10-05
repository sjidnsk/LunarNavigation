#pragma once
#include <vector>
#include "lunar_incremental_navigation_core/types/terrain_filter_method.hpp"
#include "map/terrain_surface_fit.hpp"

namespace lunar::incremental_navigation::map {
// One pass over the original candidate window, with a 10 cm Euclidean
// neighbourhood (including self), at least 3 samples, and no added samples.
// Bilateral sigmas are 7.5 cm in XY and 3 cm in Z. Only z changes; x/y/low/high
// remain raw. Consumers must evaluate all terrain evidence against raw samples.
std::vector<SurfaceSample> FilterTerrainSamples(
    const std::vector<SurfaceSample>& raw, TerrainFilterMethod method);
}  // namespace lunar::incremental_navigation::map
