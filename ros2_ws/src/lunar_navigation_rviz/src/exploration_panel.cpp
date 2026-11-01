#include "lunar_navigation_rviz/exploration_panel.hpp"
#include "lunar_navigation_rviz/boundary_tool.hpp"
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace lunar_navigation_rviz {
ExplorationPanel::ExplorationPanel(QWidget * parent) : rviz_common::Panel(parent) {
  auto * layout = new QVBoxLayout(this);
  auto * help = new QLabel("导航：使用工具栏 2D Goal Pose 点选位置并拖拽朝向。\n探索：选取区域 → 逐点单击地面 → 完成区域 → 开始探索。", this);
  help->setWordWrap(true); layout->addWidget(help);
  auto * buttons = new QGridLayout;
  auto make_button = [this, buttons](const char * name, const QString & text, int row, int col) {
    auto * button = new QPushButton(text, this); button->setObjectName(name);
    buttons->addWidget(button, row, col); return button;
  };
  auto * select = make_button("select", "选取区域", 0, 0);
  auto * finish = make_button("finish", "完成区域", 0, 1);
  auto * undo = make_button("undo", "撤销一点", 1, 0);
  auto * clear = make_button("clear", "清空区域", 1, 1);
  start_ = make_button("start", "开始探索", 2, 0);
  pause_ = make_button("pause", "暂停探索", 2, 1);
  resume_ = make_button("resume", "恢复探索", 3, 0);
  cancel_ = make_button("cancel", "结束探索", 3, 1);
  layout->addLayout(buttons);
  region_label_ = new QLabel(this); region_label_->setObjectName("region");
  state_label_ = new QLabel("探索：尚未收到后端状态", this); state_label_->setObjectName("state");
  details_ = new QLabel("覆盖率 / 目标 / 原因：等待后端状态", this); details_->setObjectName("details");
  notice_ = new QLabel("区域点使用 map 坐标系的 z=0 平面。", this); notice_->setObjectName("notice");
  for (auto * label : {region_label_, state_label_, details_, notice_}) {
    label->setTextFormat(Qt::PlainText); label->setWordWrap(true); layout->addWidget(label);
  }
  layout->addStretch();
  connect(select, &QPushButton::clicked, this, &ExplorationPanel::selectRegion);
  connect(finish, &QPushButton::clicked, this, [this] {
    receiveUpdates();
    if (!region_.close()) { notice_->setText("区域无效：至少三个不同顶点，不能退化或自交。请撤销或清空后重选。"); return; }
    collecting_ = false;
    if (getDisplayContext()) {
      auto * manager = getDisplayContext()->getToolManager();
      manager->setCurrentTool(manager->getDefaultTool());
    }
    notice_->setText("区域已闭合；开始探索前会确认替换当前导航目标。"); renderRegion();
  });
  connect(undo, &QPushButton::clicked, this, [this] { region_.undo(); renderRegion(); });
  connect(clear, &QPushButton::clicked, this, [this] {
    collecting_ = false; region_.clear();

    renderRegion();
  });
  connect(start_, &QPushButton::clicked, this, [this] { sendCommand(Task::START); });
  connect(pause_, &QPushButton::clicked, this, [this] { sendCommand(Task::PAUSE); });
  connect(resume_, &QPushButton::clicked, this, [this] { sendCommand(Task::RESUME); });
  connect(cancel_, &QPushButton::clicked, this, [this] { sendCommand(Task::CANCEL); });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &ExplorationPanel::receiveUpdates);
  timer_->start(100);
  renderRegion();
}
void ExplorationPanel::onInitialize() {
  if (!getDisplayContext()) return;
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) { notice_->setText("RViz ROS 节点不可用。"); return; }
  node_ = abstraction->get_raw_node();
  bindInterfaces();
}
void ExplorationPanel::load(const rviz_common::Config & config) {
  rviz_common::Panel::load(config);
  QString task, status;
  const auto old_task = task_topic_, old_status = status_topic_;
  if (config.mapGetString("Task Topic", &task) && !task.isEmpty()) task_topic_ = task.toStdString();
  if (config.mapGetString("Status Topic", &status) && !status.isEmpty()) status_topic_ = status.toStdString();
  if (node_ && (old_task != task_topic_ || old_status != status_topic_)) bindInterfaces();
}
void ExplorationPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Task Topic", QString::fromStdString(task_topic_));
  config.mapSetValue("Status Topic", QString::fromStdString(status_topic_));
}
void ExplorationPanel::bindInterfaces() {
  status_sub_.reset(); task_pub_.reset();
  // 旧订阅的在途回调仅持有旧Inbox，不得污染新接口状态。
  inbox_ = std::make_shared<Inbox>();
  status_received_ = false; state_ = Status::IDLE; pending_ = 0; task_id_.clear();
  state_label_->setText("探索：尚未收到后端状态");
  details_->setText("状态话题：" + QString::fromStdString(status_topic_));
  task_pub_ = node_->create_publisher<Task>(task_topic_, rclcpp::QoS(10).reliable());
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/lunar_rviz/boundary", rclcpp::QoS(1).reliable().transient_local());
  const auto inbox = inbox_;
  status_sub_ = node_->create_subscription<Status>(status_topic_,
    rclcpp::QoS(10).reliable().transient_local(), [inbox](Status::ConstSharedPtr status) {
      std::lock_guard<std::mutex> lock(inbox->mutex); inbox->statuses.push_back(std::move(status));
    });
  renderRegion();
}
void ExplorationPanel::selectRegion() {
  if (!getDisplayContext() || !node_) { notice_->setText("RViz ROS 节点尚未初始化。"); return; }
  if (getDisplayContext()->getFixedFrame() != "map") { notice_->setText("请先将 RViz Fixed Frame 设置为 map，选区不能使用其他坐标系。"); return; }
  if (region_.closed()) { notice_->setText("当前区域已闭合；请先清空区域或撤销一点再编辑。"); return; }
  auto * manager = getDisplayContext()->getToolManager();
  rviz_common::Tool * tool = nullptr;
  for (int i = 0; i < manager->numTools(); ++i) {
    if (manager->getTool(i)->getClassId() == "lunar_navigation_rviz/BoundaryTool") tool = manager->getTool(i);
  }
  if (!tool) tool = manager->addTool("lunar_navigation_rviz/BoundaryTool");
  if (!tool) { notice_->setText("区域工具加载失败，请检查插件安装。"); return; }
  auto * boundary_tool = dynamic_cast<BoundaryTool *>(tool);
  if (!boundary_tool) { notice_->setText("区域工具类型不匹配，请检查插件安装。"); return; }
  QObject::disconnect(tool_connection_);
  tool_connection_ = connect(boundary_tool, &BoundaryTool::pointSelected, this,
    [this](const QString & frame, double x, double y) {
      if (!collecting_) return;
      if (!region_.add(frame.toStdString(), x, y)) {
        notice_->setText("忽略无效选区点：坐标系必须为 map，坐标须有限，顶点不能重复。");
      }
      renderRegion();
    }, Qt::DirectConnection);
  collecting_ = true; manager->setCurrentTool(tool);
  notice_->setText("正在选区：在地图地面逐点单击，顶点数正确后点击完成区域。");
}
void ExplorationPanel::receiveUpdates() {
  std::deque<Status::ConstSharedPtr> statuses;
  { std::lock_guard<std::mutex> lock(inbox_->mutex); statuses.swap(inbox_->statuses); }
  for (const auto & status : statuses) {
  const bool invalid_start = status->reason_code.rfind("INVALID_START", 0) == 0;
  if (pending_ == Task::START && status->task_id != task_id_ && !invalid_start) continue;
  status_received_ = true; state_ = status->state; task_id_ = status->task_id;
  const bool terminal = state_ == Status::IDLE || state_ == Status::COMPLETED || state_ == Status::ERROR;
  if ((pending_ == Task::START && (invalid_start || !task_id_.empty())) ||
      (pending_ == Task::PAUSE && (state_ == Status::PAUSED || terminal)) ||
      (pending_ == Task::RESUME && state_ != Status::PAUSED) ||
      (pending_ == Task::CANCEL && state_ == Status::IDLE)) pending_ = 0;
  static const char * names[] = {"IDLE 空闲", "WAITING_FOR_INPUT 等待输入", "SELECTING_FRONTIER 选择前沿",
    "PLANNING 规划中", "EXECUTING 执行中", "REPLANNING 重规划", "PAUSED 已暂停", "COMPLETED 已完成", "ERROR 错误"};
  state_label_->setText(QString("探索：%1").arg(state_ < 9 ? names[state_] : "未知状态"));
  details_->setText(QString("任务：%1\n覆盖率：%2%\n后端目标位置：(%3, %4)\n已完成探索目标：%5\n原因：%6")
    .arg(QString::fromStdString(task_id_)).arg(status->coverage_ratio * 100., 0, 'f', 1)
    .arg(status->current_goal.position.x, 0, 'f', 2).arg(status->current_goal.position.y, 0, 'f', 2)
    .arg(status->completed_goal_count).arg(QString::fromStdString(status->reason_code)));
  if (!pending_) notice_->setText("以上为后端探索状态；导航到达需查看现有导航结果。关闭面板不会结束任务。");
  refreshButtons();
  }
}
void ExplorationPanel::renderRegion() {
  region_label_->setText(QString("选区：%1 点，%2，面积 %3 m²").arg(region_.points().size())
    .arg(region_.closed() ? "已闭合" : "编辑中").arg(region_.area(), 0, 'f', 2));
  if (marker_pub_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; marker.header.stamp = node_->now();
    marker.ns = "rviz_boundary_draft"; marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = region_.points().empty() ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.; marker.scale.x = 0.07;
    marker.color.r = 0.2f; marker.color.g = 0.9f; marker.color.b = 1.f; marker.color.a = 1.f;
    for (const auto & point : region_.points()) {
      geometry_msgs::msg::Point p; p.x = point.first; p.y = point.second; p.z = 0.03; marker.points.push_back(p);
    }
    if (region_.closed() && !marker.points.empty()) marker.points.push_back(marker.points.front());
    marker_pub_->publish(marker);
  }
  refreshButtons();
}
void ExplorationPanel::refreshButtons() {
  const bool connected = task_pub_ && status_received_;
  const bool terminal = state_ == Status::IDLE || state_ == Status::COMPLETED || state_ == Status::ERROR;
  start_->setEnabled(connected && !pending_ && terminal && region_.closed());
  pause_->setEnabled(connected && !pending_ && !task_id_.empty() && !terminal && state_ != Status::PAUSED);
  resume_->setEnabled(connected && !pending_ && !task_id_.empty() && state_ == Status::PAUSED);
  // 等待开始/暂停回执时仍允许结束，重复结束不会重复发送。
  cancel_->setEnabled(connected && pending_ != Task::CANCEL && !task_id_.empty() && (state_ != Status::IDLE || pending_));
}
void ExplorationPanel::sendCommand(uint8_t command) {
  auto * button = command == Task::START ? start_ : command == Task::PAUSE ? pause_ : command == Task::RESUME ? resume_ : cancel_;
  if (!task_pub_ || !button->isEnabled()) return;
  if (command == Task::START || command == Task::RESUME) {
    if (QMessageBox::question(this, "确认替换导航目标",
      "开始或恢复探索将由现有导航入口替换当前导航目标（包括手动目标）。是否继续？",
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    // 模态确认期间后端状态仍可更新，重新检查操作是否仍然适用。
    if (!button->isEnabled()) return;
  }
  Task task; task.header.frame_id = "map"; task.header.stamp = node_->now(); task.command = command;
  if (command == Task::START) {
    if (!region_.closed()) return;
    task_id_ = "rviz-" + QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    for (const auto & point : region_.points()) {
      geometry_msgs::msg::Point32 p; p.x = static_cast<float>(point.first); p.y = static_cast<float>(point.second); p.z = 0.;
      task.boundary.points.push_back(p);
    }
  }
  task.task_id = task_id_;
  pending_ = command; refreshButtons(); task_pub_->publish(task);
  notice_->setText("命令已发送，等待后端状态回执；可使用结束探索取消待处理的探索。");
}
}
PLUGINLIB_EXPORT_CLASS(lunar_navigation_rviz::ExplorationPanel, rviz_common::Panel)
