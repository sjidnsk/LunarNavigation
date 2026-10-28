#include "lunar_incremental_navigation_ros/exploration_map_node.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/input_store.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"
#include "lunar_incremental_navigation_ros/state_adapter.hpp"
namespace lunar::incremental_navigation_ros {
namespace core=lunar::incremental_navigation;
using PolicyService=lunar_planning_msgs::srv::GetPolicyMap;
struct ExplorationMapNode::Impl {
  ExplorationMapNode& node;
  std::string map_frame,odom_frame,base_frame,epoch;
  double window;
  std::unique_ptr<InputStore> inputs;
  std::unique_ptr<ExplorationMap> map;
  std::unique_ptr<PolicyMapExporter> exporter;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription,static_subscription;
  rclcpp::Service<PolicyService>::SharedPtr service;
  std::mutex mutex;
  std::condition_variable condition;
  bool stop=false,pending=false;
  std::thread worker;
  std::shared_ptr<const PolicyService::Response> published;
  explicit Impl(ExplorationMapNode& n):node(n) {
    auto text=[&](const char* key,const std::string& value){return node.declare_parameter<std::string>(key,value);};
    map_frame=text("map_frame","map");odom_frame=text("odom_frame","odom");base_frame=text("base_frame","base_link");
    const auto path=text("platform_config",ament_index_cpp::get_package_share_directory("lunar_incremental_navigation_ros")+"/config/wheel.yaml");
    auto loaded=LoadPlatformConfig(path,"wheel");
    if(!loaded.capability) throw std::runtime_error(loaded.error_detail);
    auto capability=*loaded.capability;
    const auto& wheel=std::get<core::WheeledCapability>(capability);
    const double radius=core::CircumscribedRadius(wheel.footprint_xy_m);
    if(radius<=0.) throw std::invalid_argument("wheel footprint must have positive radius");
    core::TraversabilityProfile profile{.planar_envelope_xy_m=wheel.footprint_xy_m,
      .preferred_clearance_m=wheel.minimum_clearance_m,.slope_weight=1.,.relief_weight=1.,.clearance_weight=1.,
      .goal_position_tolerance_m=node.declare_parameter<double>("goal_position_tolerance_m",.1),
      .goal_yaw_tolerance_rad=node.declare_parameter<double>("goal_yaw_tolerance_rad",.05)};
    // 硬包络真实覆盖车体外接圆加选定能力净空，不把离地高度当导航净空。
    for(auto& p:profile.planar_envelope_xy_m) {p.x*=1.+wheel.minimum_clearance_m/radius;p.y*=1.+wheel.minimum_clearance_m/radius;}
    window=node.declare_parameter<double>("local_window_size_m",64.);
    if(!std::isfinite(window)||window<=0||!std::isfinite(profile.goal_position_tolerance_m)||profile.goal_position_tolerance_m<0||
       !std::isfinite(profile.goal_yaw_tolerance_rad)||profile.goal_yaw_tolerance_rad<0) throw std::invalid_argument("invalid map query profile");
    map=std::make_unique<ExplorationMap>(capability,profile,node.declare_parameter<double>("policy_resolution_m",.2),
        node.declare_parameter<double>("coarse_resolution_m",1.));
    exporter=std::make_unique<PolicyMapExporter>(capability,profile);
    inputs=std::make_unique<InputStore>(map_frame,odom_frame);
    epoch="observed-projection-v1-"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    publisher=node.create_publisher<nav_msgs::msg::OccupancyGrid>(text("exploration_map_topic","/Car/T4/mapping/exploration_map"),rclcpp::QoS(1).reliable().transient_local());
    map_subscription=node.create_subscription<grid_map_msgs::msg::GridMap>(text("map_topic","/Car/T3/mapping/grid_map"),rclcpp::QoS(1).best_effort(),
        [this](grid_map_msgs::msg::GridMap::ConstSharedPtr message){inputs->UpdateLocal(std::move(message));Wake();});
    odom_subscription=node.create_subscription<nav_msgs::msg::Odometry>(text("odometry_topic","/Car/T3/localization/odometry"),rclcpp::QoS(1).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message){inputs->UpdateOdometry(std::move(message));Wake();});
    auto tf_callback=[this](tf2_msgs::msg::TFMessage::ConstSharedPtr message){inputs->UpdateTf(*message);Wake();};
    tf_subscription=node.create_subscription<tf2_msgs::msg::TFMessage>(text("tf_topic","/tf"),rclcpp::QoS(100).best_effort(),tf_callback);
    static_subscription=node.create_subscription<tf2_msgs::msg::TFMessage>(text("tf_static_topic","/tf_static"),rclcpp::QoS(100).reliable().transient_local(),tf_callback);
    service=node.create_service<PolicyService>(text("policy_map_service","/Car/T4/mapping/get_policy_map"),
      [this](std::shared_ptr<PolicyService::Request> request,std::shared_ptr<PolicyService::Response> response) {
        const auto snapshot=std::atomic_load(&published);
        if(!snapshot) {response->ready=false;response->reason_code="INPUT_UNAVAILABLE";return;}
        *response=*snapshot;
        const auto stamp=static_cast<std::int64_t>(response->processed_stamp.sec)*1000000000LL+response->processed_stamp.nanosec;
        const auto requested=request->minimum_map_stamp_ns;
        if(stamp<requested) {response->ready=false;response->reason_code="MAP_STAMP_TOO_OLD";return;}
        // 旧版本可用全部当前tile形成保守delta；同版本不重复传输tile。
        if(request->since_revision>0&&request->since_revision<=response->fine_revision) {
          response->full_snapshot=false;response->base_revision=request->since_revision;
          if(request->since_revision==response->fine_revision) response->tiles.clear();
        }
      });
    worker=std::thread([this]{Run();});
  }
  ~Impl() {
    {std::lock_guard lock(mutex);stop=true;}condition.notify_all();
    if(worker.joinable()) worker.join();
  }
  void Wake() {{std::lock_guard lock(mutex);pending=true;}condition.notify_one();}
  void Run() {
    std::uint64_t local_sequence=0,tf_sequence=0;
    while(true) {
      {std::unique_lock lock(mutex);condition.wait(lock,[this]{return stop||pending;});if(stop) return;pending=false;}
      try {
        auto input=inputs->Capture();
        if(input.local_map && (local_sequence!=input.local_sequence||tf_sequence!=input.tf_sequence)) {
          // 同map坐标输入不需要虚构TF；odom坐标仍使用最新/静态TF，不检查stamp。
          auto map_input=input;
          if(input.local_map->header.frame_id==map_frame) {
            geometry_msgs::msg::TransformStamped identity;
            identity.header.frame_id=map_frame;identity.child_frame_id=map_frame;identity.transform.rotation.w=1.;
            map_input.map_from_odom=identity;
          }
          const auto evidence=AdaptExplorationElevation(map_input);
          if(evidence.value) {
            const auto stamp=static_cast<std::int64_t>(input.local_map->header.stamp.sec)*1000000000LL+input.local_map->header.stamp.nanosec;
            if(map->Apply(*evidence.value,stamp)) {local_sequence=input.local_sequence;tf_sequence=input.tf_sequence;}
          }
        }
        auto capture=map->Capture();
        if(!capture.map.fine) continue;
        capture.map.epoch=epoch;capture.map.local_window_size_m=window;
        if(input.odometry&&input.map_from_odom) {
          const auto transform=AdaptDirectMapFromOdom(*input.map_from_odom,map_frame,odom_frame);
          if(transform.value) {
            const auto state=AdaptStateInput(*transform.value,*input.odometry,map_frame,odom_frame,base_frame);
            if(state.value) capture.map.anchor=state.value->base_link_pose;
          }
        }
        auto response=exporter->ExportObserved(capture,0);
        if(!capture.map.anchor) {response.ready=false;response.reason_code="INPUT_UNAVAILABLE";}
        std::atomic_store(&published,std::make_shared<const PolicyService::Response>(std::move(response)));
        const auto projection=map->Project();
        const auto& g=projection.geometry;
        nav_msgs::msg::OccupancyGrid message;message.header.frame_id=g.frame_id();message.header.stamp=node.now();
        message.info.resolution=g.resolution_m();message.info.width=g.width();message.info.height=g.height();
        message.info.origin.position.x=g.origin_m().x+g.min_inclusive().x*g.resolution_m();
        message.info.origin.position.y=g.origin_m().y+g.min_inclusive().y*g.resolution_m();message.info.origin.orientation.w=1.;
        message.data=projection.data;publisher->publish(std::move(message));
      } catch(const std::exception& error) {RCLCPP_ERROR(node.get_logger(),"地图更新失败: %s",error.what());}
    }
  }
};
ExplorationMapNode::ExplorationMapNode(const rclcpp::NodeOptions& options):rclcpp::Node("lunar_exploration_map_node",options),impl_(std::make_unique<Impl>(*this)) {}
ExplorationMapNode::~ExplorationMapNode()=default;
}  // namespace lunar::incremental_navigation_ros
