#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
namespace lunar::incremental_navigation {
// 仅定义测量尺度，不包含通行阈值和车体膨胀。
double PlatformTerrainLongSide(const PlatformCapability& capability);
std::int64_t PlatformTerrainWindowRadiusCells(double long_side_m,double resolution_m,bool local=false);
struct PlatformTerrainMeasurementBatch {
 GridIndex min_inclusive{};
 std::size_t width{},height{};
 std::vector<LocalTerrainMeasurements> measurements;
 const LocalTerrainMeasurements* At(GridIndex index) const noexcept;
};
PlatformTerrainMeasurementBatch MeasurePlatformTerrainBatch(const ElevationRangeView& view,double long_side_m,GridIndex min_inclusive,GridIndex max_exclusive);
LocalTerrainMeasurements MeasurePlatformTerrain(const ElevationRangeView& view,GridIndex index,double long_side_m);
std::unique_ptr<const PlatformElevationEvaluator> MakePlatformWindowElevationEvaluator(const ElevationRangeView& view,const PlatformCapability& capability);
}
