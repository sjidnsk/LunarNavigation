#pragma once

namespace lunar::incremental_navigation {
// 原始高程改变时，覆盖背景测量和缺测推断的保守 intrinsic 重算半径。
// 与默认地面观测策略共用实现；不改变现有公开类型或 ABI 布局。
int TerrainIntrinsicDependencyRadiusCells(double resolution_m, double background_half_width_m = 0.0);
}  // namespace lunar::incremental_navigation
