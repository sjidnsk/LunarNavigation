#pragma once
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
namespace lunar::incremental_navigation {
// 批处理只绑定一个不可变 view；旧标量接口与类型布局不变。
// 返回 evaluator 只能用原 view 调用；view 必须比 evaluator 活得更久。
std::unique_ptr<const PlatformElevationEvaluator> MakeSnapshotElevationEvaluator(
    const ElevationRangeView& view, const PlatformCapability& capability);
// 只导出 intrinsic 三态；充分条件快证书不会伪造旧 Evaluate 的浮点字段。
class SnapshotTerrainClassifier final {
 public:
  SnapshotTerrainClassifier(const ElevationRangeView&, const PlatformCapability&);
  ~SnapshotTerrainClassifier();
  IntrinsicCellState Classify(GridIndex);
  std::size_t certified_free_cells() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
