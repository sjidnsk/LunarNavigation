#pragma once
#include <memory>
#include <deque>
#include <mutex>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <lunar_pure_exploration_msgs/msg/pure_exploration_task.hpp>
#include <lunar_pure_exploration_msgs/msg/pure_exploration_status.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "lunar_navigation_rviz/boundary_selection.hpp"
class QLabel;
class QPushButton;
class QTimer;
namespace lunar_navigation_rviz {
class ExplorationPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit ExplorationPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
private:
  using Task = lunar_pure_exploration_msgs::msg::PureExplorationTask;
  using Status = lunar_pure_exploration_msgs::msg::PureExplorationStatus;
  struct Inbox {
    std::mutex mutex;
    std::deque<Status::ConstSharedPtr> statuses;

  };
  void bindInterfaces();
  void receiveUpdates();
  void selectRegion();
  void renderRegion();
  void refreshButtons();
  void sendCommand(uint8_t command);
  std::shared_ptr<Inbox> inbox_{std::make_shared<Inbox>()};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Task>::SharedPtr task_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  QMetaObject::Connection tool_connection_;
  BoundarySelection region_;
  bool collecting_{false};
  bool status_received_{false};
  uint8_t state_{Status::IDLE};
  uint8_t pending_{0};
  std::string task_id_;
  std::string task_topic_{"/Car/T4/exploration/task"};
  std::string status_topic_{"/Car/T4/exploration/status"};
  QLabel * state_label_;
  QLabel * details_;
  QLabel * region_label_;
  QLabel * notice_;
  QPushButton * start_;
  QPushButton * pause_;
  QPushButton * resume_;
  QPushButton * cancel_;
  QTimer * timer_;
};
}
