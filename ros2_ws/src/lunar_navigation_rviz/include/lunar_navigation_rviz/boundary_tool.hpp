#pragma once
#include <rviz_common/tool.hpp>
namespace lunar_navigation_rviz {
class BoundaryTool : public rviz_common::Tool {
  Q_OBJECT
public:
  BoundaryTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
Q_SIGNALS:
  void pointSelected(const QString & frame, double x, double y);
};
}
