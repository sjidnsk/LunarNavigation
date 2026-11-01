#include <gtest/gtest.h>
#include <QApplication>
#include <QAbstractButton>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <chrono>
#include <thread>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include "lunar_navigation_rviz/exploration_panel.hpp"
#include "lunar_navigation_rviz/boundary_tool.hpp"
namespace {
using Task = lunar_pure_exploration_msgs::msg::PureExplorationTask;
using Status = lunar_pure_exploration_msgs::msg::PureExplorationStatus;
// 只替代RViz图形宿主；所有ROS消息、publisher、subscriber和executor均为真实对象。
class NodeAbstraction : public rviz_common::ros_integration::RosNodeAbstractionIface {
public:
  explicit NodeAbstraction(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}
  std::string get_node_name() const override { return node_->get_name(); }
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const override { return node_->get_topic_names_and_types(); }
  rclcpp::Node::SharedPtr get_raw_node() override { return node_; }
private:
  rclcpp::Node::SharedPtr node_;
};
class Context : public rviz_common::DisplayContext {
public:
  explicit Context(rclcpp::Node::SharedPtr node) : node_(node), abstraction_(std::make_shared<NodeAbstraction>(node)), tools_(this) {}
  Ogre::SceneManager * getSceneManager() const override { return nullptr; }
  rviz_common::WindowManagerInterface * getWindowManager() const override { return nullptr; }
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface> getSelectionManager() const override { return {}; }
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface> getHandlerManager() const override { return {}; }
  std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker() const override { return {}; }
  rviz_common::FrameManagerIface * getFrameManager() const override { return nullptr; }
  QString getFixedFrame() const override { return "map"; }
  uint64_t getFrameCount() const override { return 0; }
  rviz_common::DisplayFactory * getDisplayFactory() const override { return nullptr; }
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr getRosNodeAbstraction() const override { return abstraction_; }
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager * getToolManager() const override { return const_cast<rviz_common::ToolManager *>(&tools_); }
  rviz_common::ViewManager * getViewManager() const override { return nullptr; }
  rviz_common::transformation::TransformationManager * getTransformationManager() override { return nullptr; }
  rviz_common::DisplayGroup * getRootDisplayGroup() const override { return nullptr; }
  uint32_t getDefaultVisibilityBit() const override { return 0; }
  rviz_common::BitAllocator * visibilityBits() override { return nullptr; }
  void setStatus(const QString &) override {}
  QString getHelpPath() const override { return {}; }
  std::shared_ptr<rclcpp::Clock> getClock() override { return node_->get_clock(); }
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<NodeAbstraction> abstraction_;
  rviz_common::ToolManager tools_;
};
}
TEST(ExplorationPanelDds, PublishesValidatedCommandsWithConfirmationAndBackendAcknowledgements) {
  int argc = 1; char name[] = "panel_dds_test"; char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  struct Shutdown { ~Shutdown() { rclcpp::shutdown(); } } shutdown;
  auto node = std::make_shared<rclcpp::Node>("rviz_panel_test");
  auto peer = std::make_shared<rclcpp::Node>("rviz_panel_peer");
  Context context(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(peer);
  std::vector<Task> tasks;
  auto task_sub = peer->create_subscription<Task>("/panel_test/exploration/task", 10,
    [&](Task::ConstSharedPtr task) { tasks.push_back(*task); });
  auto status_pub = peer->create_publisher<Status>("/panel_test/exploration/status", rclcpp::QoS(10).reliable().transient_local());

  auto panel = std::make_unique<lunar_navigation_rviz::ExplorationPanel>();
  rviz_common::Config config;
  config.mapSetValue("Task Topic","/panel_test/exploration/task");
  config.mapSetValue("Status Topic","/panel_test/exploration/status");
  panel->load(config);  // 配置先于initialize。
  panel->initialize(&context);
  rviz_common::Config saved; panel->save(saved);
  QString saved_task, saved_status;
  ASSERT_TRUE(saved.mapGetString("Task Topic",&saved_task)); EXPECT_EQ(saved_task,"/panel_test/exploration/task");
  ASSERT_TRUE(saved.mapGetString("Status Topic",&saved_status)); EXPECT_EQ(saved_status,"/panel_test/exploration/status");
  auto wait = [&](auto predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some(); QApplication::processEvents();
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  };
  auto pump = [&] {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < end) { executor.spin_some(); QApplication::processEvents(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  };
  auto button = [&](const char * id) { return panel->findChild<QPushButton *>(id); };
  auto confirm = [&](QMessageBox::StandardButton answer, const char * id) {
    QTimer::singleShot(0, [&] { auto * dialog = qobject_cast<QMessageBox *>(QApplication::activeModalWidget()); ASSERT_NE(dialog, nullptr); ASSERT_NE(dialog->button(answer), nullptr); dialog->button(answer)->click(); });
    button(id)->click();
  };
  ASSERT_TRUE(wait([&] { return status_pub->get_subscription_count() && task_sub->get_publisher_count(); }));
  Status status; status.state = Status::IDLE; status.reason_code = "IDLE"; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return panel->findChild<QLabel *>("state")->text().contains("IDLE"); }));
  EXPECT_FALSE(button("start")->isEnabled()); button("start")->click(); pump(); EXPECT_TRUE(tasks.empty());
  button("select")->click();
  auto * tool = dynamic_cast<lunar_navigation_rviz::BoundaryTool *>(context.getToolManager()->getCurrentTool());
  ASSERT_NE(tool,nullptr);
  auto point = [&](const QString & frame, double x, double y) {
    ASSERT_TRUE(QMetaObject::invokeMethod(tool,"pointSelected",Qt::DirectConnection,
      Q_ARG(QString,frame),Q_ARG(double,x),Q_ARG(double,y)));
  };
  point("odom",99,99);
  EXPECT_TRUE(panel->findChild<QLabel *>("region")->text().contains("0 点"));
  point("map",0,0); point("map",2,0); point("map",2,2); point("map",0,2);
  // 最后一点与完成操作之间不泵Qt/ROS队列，必须同步形成完整四边形。
  EXPECT_TRUE(panel->findChild<QLabel *>("region")->text().contains("4 点"));
  button("finish")->click(); ASSERT_TRUE(button("start")->isEnabled());
  confirm(QMessageBox::No, "start"); pump(); EXPECT_TRUE(tasks.empty());
  confirm(QMessageBox::Yes, "start"); ASSERT_TRUE(wait([&] { return tasks.size()==1; }));
  EXPECT_EQ(tasks[0].command, Task::START); EXPECT_EQ(tasks[0].header.frame_id,"map");
  ASSERT_EQ(tasks[0].boundary.points.size(),4u); EXPECT_FLOAT_EQ(tasks[0].boundary.points[2].x,2.f); EXPECT_FLOAT_EQ(tasks[0].boundary.points[2].y,2.f);
  const auto id = tasks[0].task_id; EXPECT_FALSE(id.empty());
  button("start")->click(); status_pub->publish(status); pump(); EXPECT_EQ(tasks.size(),1u); EXPECT_FALSE(button("start")->isEnabled());
  status.task_id=id; status.state=Status::EXECUTING; status.reason_code="PLAN_FOUND"; status.coverage_ratio=.75; status.current_goal.position.x=3.; status.current_goal.position.y=5.; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return button("pause")->isEnabled(); }));
  EXPECT_TRUE(panel->findChild<QLabel *>("details")->text().contains("75.0%"));
  EXPECT_TRUE(panel->findChild<QLabel *>("details")->text().contains("PLAN_FOUND"));
  button("pause")->click(); button("pause")->click(); ASSERT_TRUE(wait([&] { return tasks.size()==2; }));
  EXPECT_EQ(tasks[1].command,Task::PAUSE); EXPECT_EQ(tasks[1].task_id,id);
  status.state=Status::PAUSED; status.reason_code="MANUAL_GOAL_REPLACED"; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return button("resume")->isEnabled(); }));
  confirm(QMessageBox::No,"resume"); pump(); EXPECT_EQ(tasks.size(),2u);
  confirm(QMessageBox::Yes,"resume"); button("resume")->click(); ASSERT_TRUE(wait([&] { return tasks.size()==3; }));
  EXPECT_EQ(tasks[2].command,Task::RESUME); EXPECT_EQ(tasks[2].task_id,id);
  // 两个真实DDS回执先全部进入ROS callback，期间不处理Qt timer事件。
  status.state=Status::WAITING_FOR_INPUT; status.reason_code="RESUMED"; status_pub->publish(status);
  status.state=Status::PAUSED; status.reason_code="MANUAL_GOAL_REPLACED"; status_pub->publish(status);
  const auto receive_end=std::chrono::steady_clock::now()+std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now()<receive_end) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  ASSERT_TRUE(wait([&] { return button("resume")->isEnabled(); })) << "不得吞掉RESUMED中间回执并永久锁住恢复";
  status.state=Status::WAITING_FOR_INPUT; status.reason_code="RESUMED"; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return button("pause")->isEnabled(); }));
  button("cancel")->click(); button("cancel")->click(); ASSERT_TRUE(wait([&] { return tasks.size()==4; }));
  EXPECT_EQ(tasks[3].command,Task::CANCEL); EXPECT_EQ(tasks[3].task_id,id);
  status.state=Status::IDLE; status.task_id.clear(); status.reason_code="CANCELED"; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return button("start")->isEnabled(); }));
  confirm(QMessageBox::Yes,"start"); ASSERT_TRUE(wait([&] { return tasks.size()==5; }));
  status.task_id=tasks.back().task_id; status.state=Status::EXECUTING; status_pub->publish(status);
  ASSERT_TRUE(wait([&] { return button("pause")->isEnabled(); }));
  // 也支持宿主先initialize后load；默认端点须退订，旧回执不能污染新接口。
  auto reloaded_panel = std::make_unique<lunar_navigation_rviz::ExplorationPanel>();
  reloaded_panel->initialize(&context);
  auto old_status_pub = peer->create_publisher<Status>("/Car/T4/exploration/status", rclcpp::QoS(10).reliable().transient_local());
  Status old_status; old_status.state=Status::ERROR; old_status.reason_code="OLD_ENDPOINT";
  old_status_pub->publish(old_status);
  executor.spin_some();
  reloaded_panel->load(config);
  ASSERT_TRUE(wait([&] { return reloaded_panel->findChild<QLabel *>("state")->text().contains("EXECUTING"); }));
  EXPECT_FALSE(reloaded_panel->findChild<QLabel *>("details")->text().contains("OLD_ENDPOINT"));
  reloaded_panel.reset();
  panel.reset(); pump(); EXPECT_EQ(tasks.size(),5u) << "关闭面板不得自动发送CANCEL";
}
