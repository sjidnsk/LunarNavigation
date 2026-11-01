#include "lunar_navigation_rviz/boundary_tool.hpp"
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreViewport.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace lunar_navigation_rviz {
BoundaryTool::BoundaryTool() { shortcut_key_ = 'b'; }
void BoundaryTool::onInitialize() { setName("探索区域顶点"); }
void BoundaryTool::activate() { setStatus("请先在探索面板点击选取区域，再逐点单击 map 地面，最后点击完成区域。"); }
void BoundaryTool::deactivate() {}
int BoundaryTool::processMouseEvent(rviz_common::ViewportMouseEvent & event) {
  if (!event.leftDown() || context_->getFixedFrame() != "map") return Render;
  if (!event.panel || !event.panel->getRenderWindow()) return Render;
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  if (!viewport || viewport->getActualWidth() == 0 || viewport->getActualHeight() == 0) return Render;
  auto * camera = viewport->getCamera();
  if (!camera) return Render;
  const auto ray = camera->getCameraToViewportRay(
    static_cast<float>(event.x) / viewport->getActualWidth(),
    static_cast<float>(event.y) / viewport->getActualHeight());
  const auto intersection = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0));
  if (!intersection.first || intersection.second < 0) return Render;
  const auto point = ray.getPoint(intersection.second);
  // RViz鼠标事件和面板均位于GUI线程，点选立即交付，不经DDS异步回环。
  Q_EMIT pointSelected("map", point.x, point.y);
  return Render;
}
}
PLUGINLIB_EXPORT_CLASS(lunar_navigation_rviz::BoundaryTool, rviz_common::Tool)
