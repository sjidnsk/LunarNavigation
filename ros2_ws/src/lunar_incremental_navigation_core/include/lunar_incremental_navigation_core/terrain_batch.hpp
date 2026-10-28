#pragma once
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
namespace lunar::incremental_navigation {
// 批处理只绑定一个不可变 view；旧标量接口与类型布局不变。
// 返回 evaluator 只能用原 view 调用；view 必须比 evaluator 活得更久。
std::unique_ptr<const PlatformElevationEvaluator> MakeSnapshotElevationEvaluator(
    const ElevationRangeView& view, const PlatformCapability& capability);
}
