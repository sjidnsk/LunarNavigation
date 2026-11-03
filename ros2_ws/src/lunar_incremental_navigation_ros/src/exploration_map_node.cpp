#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <thread>
#include <std_msgs/msg/string.hpp>
#include <utility>

#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/exploration_facts_codec.hpp"
#include "lunar_incremental_navigation_ros/exploration_map_node.hpp"
#include "lunar_incremental_navigation_ros/input_store.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"
#include "lunar_incremental_navigation_ros/state_adapter.hpp"
#include "lunar_incremental_navigation_ros/traversability_qos.hpp"
namespace lunar::incremental_navigation_ros {
namespace core = lunar::incremental_navigation;
using PolicyService = lunar_planning_msgs::srv::GetPolicyMap;
namespace {
bool EquivalentTransform(const geometry_msgs::msg::Transform& a,
                         const geometry_msgs::msg::Transform& b) {
  if (a.translation != b.translation) return false;
  auto normalize = [](const auto& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return std::array<double, 4>{q.w / n, q.x / n, q.y / n, q.z / n};
  };
  const auto qa = normalize(a.rotation), qb = normalize(b.rotation);
  double same = 0., opposite = 0.;
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(qa[i]) || !std::isfinite(qb[i])) return false;
    same = std::max(same, std::abs(qa[i] - qb[i]));
    opposite = std::max(opposite, std::abs(qa[i] + qb[i]));
  }
  return std::min(same, opposite) <= 1e-12;
}
}  // namespace

struct ExplorationMapNode::Impl {
  ExplorationMapNode& node;
  std::string map_frame, odom_frame, base_frame, epoch, base_map_facts_topic;
  double window;
  std::unique_ptr<InputStore> inputs;
  std::unique_ptr<ExplorationMap> map;
  std::function<std::unique_ptr<ExplorationMap>()> make_map;
  std::unique_ptr<PolicyMapExporter> exporter;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr facts_subscription;
  std_msgs::msg::String::ConstSharedPtr latest_facts;
  std::optional<nav_msgs::msg::OccupancyGrid> last_published_grid;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription,
      static_subscription;
  rclcpp::Service<PolicyService>::SharedPtr service;
  std::mutex mutex;
  std::condition_variable condition;
  bool stop = false, pending = false;
  std::thread worker;
  std::shared_ptr<const PublishedExplorationMap> published;
  explicit Impl(ExplorationMapNode& n) : node(n) {
    auto text = [&](const char* key, const std::string& value) {
      return node.declare_parameter<std::string>(key, value);
    };
    map_frame = text("map_frame", "map");
    odom_frame = text("odom_frame", "odom");
    base_frame = text("base_frame", "base_link");
    const auto path =
        text("platform_config", ament_index_cpp::get_package_share_directory(
                                    "lunar_incremental_navigation_ros") +
                                    "/config/wheel.yaml");
    auto loaded = LoadPlatformConfig(path, "wheel");
    if (!loaded.capability) throw std::runtime_error(loaded.error_detail);
    auto capability = *loaded.capability;
    const auto& wheel = std::get<core::WheeledCapability>(capability);
    const double radius = core::CircumscribedRadius(wheel.footprint_xy_m);
    if (radius <= 0.)
      throw std::invalid_argument("wheel footprint must have positive radius");
    core::TraversabilityProfile profile{
        .planar_envelope_xy_m = wheel.footprint_xy_m,
        .preferred_clearance_m = wheel.minimum_clearance_m,
        .slope_weight = 1.,
        .relief_weight = 1.,
        .clearance_weight = 1.,
        .goal_position_tolerance_m =
            node.declare_parameter<double>("goal_position_tolerance_m", .1),
        .goal_yaw_tolerance_rad =
            node.declare_parameter<double>("goal_yaw_tolerance_rad", .05)};
    // 硬包络真实覆盖车体外接圆加选定能力净空，不把离地高度当导航净空。
    for (auto& p : profile.planar_envelope_xy_m) {
      p.x *= 1. + wheel.minimum_clearance_m / radius;
      p.y *= 1. + wheel.minimum_clearance_m / radius;
    }
    window = node.declare_parameter<double>("local_window_size_m", 64.);
    if (!std::isfinite(window) || window <= 0 ||
        !std::isfinite(profile.goal_position_tolerance_m) ||
        profile.goal_position_tolerance_m < 0 ||
        !std::isfinite(profile.goal_yaw_tolerance_rad) ||
        profile.goal_yaw_tolerance_rad < 0)
      throw std::invalid_argument("invalid map query profile");
    const double policy_resolution = node.declare_parameter<double>("policy_resolution_m", 0.);
    const double coarse_resolution = node.declare_parameter<double>("coarse_resolution_m", 0.);
    if (!std::isfinite(policy_resolution) || policy_resolution < 0. ||
        !std::isfinite(coarse_resolution) || coarse_resolution < 0.)
      throw std::invalid_argument("invalid map resolution (0 follows source)");
    make_map = [capability, profile, policy_resolution, coarse_resolution] {
      return std::make_unique<ExplorationMap>(capability, profile, policy_resolution, coarse_resolution);
    };
    exporter = std::make_unique<PolicyMapExporter>(capability, profile);
    inputs = std::make_unique<InputStore>(map_frame, odom_frame);
    epoch = "observed-projection-v1-" +
            std::to_string(
                std::chrono::system_clock::now().time_since_epoch().count());
    publisher = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
        text("exploration_map_topic", "/Car/T4/mapping/exploration_map"),
        rclcpp::QoS(1).reliable().transient_local());
    const auto map_qos =
        MakeTraversabilityInputQos(text("map_qos_reliability", "reliable"),
                                   text("map_qos_durability", "volatile"));
    if (!map_qos) throw std::invalid_argument("invalid map input QoS");
    base_map_facts_topic = text("base_map_facts_topic", "");
    if (!base_map_facts_topic.empty()) {
      facts_subscription = node.create_subscription<std_msgs::msg::String>(
          base_map_facts_topic, rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::String::ConstSharedPtr message) {
            {
              std::lock_guard lock(mutex);
              latest_facts = std::move(message);
            }
            Wake();
          });
    } else {
      map_subscription = node.create_subscription<grid_map_msgs::msg::GridMap>(
          text("map_topic", "/Car/T3/mapping/grid_map"), *map_qos,
          [this](grid_map_msgs::msg::GridMap::ConstSharedPtr message) {
            RCLCPP_DEBUG(
                node.get_logger(), "地图接收 stamp=%d res=%.6f cells=%zu",
                message->header.stamp.sec, message->info.resolution,
                message->data.empty() ? 0UL : message->data.front().data.size());
            inputs->UpdateLocal(std::move(message));
            Wake();
          });
    }
    odom_subscription = node.create_subscription<nav_msgs::msg::Odometry>(
        text("odometry_topic", "/Car/T3/localization/odometry"),
        rclcpp::QoS(1).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          inputs->UpdateOdometry(std::move(message));
          Wake();
        });
    auto tf_callback =
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          inputs->UpdateTf(*message);
          Wake();
        };
    tf_subscription = node.create_subscription<tf2_msgs::msg::TFMessage>(
        text("tf_topic", "/tf"), rclcpp::QoS(100).best_effort(), tf_callback);
    static_subscription = node.create_subscription<tf2_msgs::msg::TFMessage>(
        text("tf_static_topic", "/tf_static"),
        rclcpp::QoS(100).reliable().transient_local(), tf_callback);
    service = node.create_service<PolicyService>(
        text("policy_map_service", "/Car/T4/mapping/get_policy_map"),
        [this](std::shared_ptr<PolicyService::Request> request,
               std::shared_ptr<PolicyService::Response> response) {
          const auto snapshot = std::atomic_load(&published);
          if (!snapshot) {
            response->ready = false;
            response->reason_code = "INPUT_UNAVAILABLE";
            return;
          }
          *response = SelectExplorationMapResponse(*snapshot, request->since_revision,
                                                   request->minimum_map_stamp_ns);
        });
    worker = std::thread([this] { Run(); });
  }
  ~Impl() {
    {
      std::lock_guard lock(mutex);
      stop = true;
    }
    condition.notify_all();
    if (worker.joinable()) worker.join();
  }
  void Wake() {
    {
      std::lock_guard lock(mutex);
      pending = true;
    }
    condition.notify_one();
  }
  void Run() {
    std::uint64_t local_sequence = 0;
    std::optional<geometry_msgs::msg::Transform> applied_transform;
    while (true) {
      std_msgs::msg::String::ConstSharedPtr facts_message;
      {
        std::unique_lock lock(mutex);
        condition.wait(lock, [this] { return stop || pending; });
        if (stop) return;
        pending = false;
        facts_message = std::exchange(latest_facts, {});
      }
      try {
        auto input = inputs->Capture();
        if (facts_message) {
          auto [facts, stamp] = ParseExplorationFacts(facts_message->data);
          if (facts.frame != map_frame)
            throw std::invalid_argument("base map facts frame does not match configured map_frame");
          auto candidate = map ? nullptr : make_map();
          auto* active_map = map ? map.get() : candidate.get();
          if (!active_map->ApplyFacts(facts, stamp))
            throw std::invalid_argument("base map facts snapshot rejected");
          if (candidate) map = std::move(candidate);
          RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 10000,
              "共享事实 epoch=%s raw_revision=%lu source_resolution=%.9f",
              map->SourceEpoch().c_str(), map->SourceRawRevision(), facts.source_resolution_m);
        }
        const bool transform_changed =
            input.local_map && input.local_map->header.frame_id != map_frame &&
            input.map_from_odom &&
            (!applied_transform ||
             !EquivalentTransform(*applied_transform,
                                  input.map_from_odom->transform));
        if (base_map_facts_topic.empty() && input.local_map &&
            (local_sequence != input.local_sequence || transform_changed)) {
          // 同map坐标输入不需要虚构TF；odom坐标仍使用最新/静态TF，不检查stamp。
          auto map_input = input;
          if (input.local_map->header.frame_id == map_frame) {
            geometry_msgs::msg::TransformStamped identity;
            identity.header.frame_id = map_frame;
            identity.child_frame_id = map_frame;
            identity.transform.rotation.w = 1.;
            map_input.map_from_odom = identity;
          }
          const auto started = std::chrono::steady_clock::now();
          const auto evidence = AdaptExplorationElevation(map_input);
          const auto adapted_at = std::chrono::steady_clock::now();
          if (evidence.value) {
            const auto stamp =
                static_cast<std::int64_t>(input.local_map->header.stamp.sec) *
                    1000000000LL +
                input.local_map->header.stamp.nanosec;
            // 首次有效源证据才安装地图实例；无图时服务仍保持not ready。
            auto candidate = map ? nullptr : make_map();
            auto* active_map = map ? map.get() : candidate.get();
            if (active_map->Apply(*evidence.value, stamp)) {
              if (candidate) map = std::move(candidate);
              const auto effective = map->Capture().map.fine;
              if (effective) {
                RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 10000,
                    "地图分辨率 source=%.9f policy=%.9f coarse=%.9f (0参数为auto)",
                    evidence.value->geometry.resolution_m,
                    effective->geometry().resolution_m(),
                    node.get_parameter("coarse_resolution_m").as_double() == 0.
                        ? effective->geometry().resolution_m()
                        : node.get_parameter("coarse_resolution_m").as_double());
              }
              const auto storage = map->StorageStats();
              RCLCPP_DEBUG(node.get_logger(),
                           "地图历史 policy_cells=%zu fragments=%zu",
                           storage.policy_cells, storage.fragments);
              local_sequence = input.local_sequence;
              if (input.map_from_odom)
                applied_transform = input.map_from_odom->transform;
              RCLCPP_DEBUG(
                  node.get_logger(),
                  "地图派生 seq=%lu stamp=%lld adapt_ms=%.3f apply_ms=%.3f",
                  local_sequence, stamp,
                  std::chrono::duration<double, std::milli>(adapted_at -
                                                            started)
                      .count(),
                  std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - adapted_at)
                      .count());
            } else
              RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                                   "地图源几何或测量被拒绝");
          } else
            RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                                 "地图适配未就绪: %s",
                                 evidence.reason_code.c_str());
        }
        if (!map) continue;
        auto capture = map->Capture();
        capture.map.epoch = base_map_facts_topic.empty() ? epoch : map->SourceEpoch();
        if (!capture.map.fine) {
          const auto previous = std::atomic_load(&published);
          auto unavailable = BuildExplorationMapPublication(capture, previous.get(), *exporter,
              map->MinimumCompatibleRevision(), map_frame);
          std::atomic_store(&published,
              std::make_shared<const PublishedExplorationMap>(std::move(unavailable)));
          // 空完整快照直接撤销旧已知图，不等待超时；沿用旧图几何清除同一空间。
          if (last_published_grid && std::any_of(last_published_grid->data.begin(),
              last_published_grid->data.end(), [](auto state) { return state != -1; })) {
            auto cleared = *last_published_grid;
            std::fill(cleared.data.begin(), cleared.data.end(), -1);
            cleared.header.stamp = node.now();
            publisher->publish(cleared);
            last_published_grid = std::move(cleared);
          }
          continue;
        }
        capture.map.local_window_size_m = window;
        if (input.odometry && input.map_from_odom) {
          const auto transform = AdaptDirectMapFromOdom(*input.map_from_odom,
                                                        map_frame, odom_frame);
          if (transform.value) {
            const auto state =
                AdaptStateInput(*transform.value, *input.odometry, map_frame,
                                odom_frame, base_frame);
            if (state.value) capture.map.anchor = state.value->base_link_pose;
          }
        }
        const auto previous = std::atomic_load(&published);
        const bool same_fine = previous && previous->response.epoch == capture.map.epoch &&
            previous->response.fine_revision == capture.map.fine->fine_traversability_revision();
        const auto export_started = std::chrono::steady_clock::now();
        auto publication = BuildExplorationMapPublication(capture, previous.get(), *exporter,
            map->MinimumCompatibleRevision(), map_frame);
        RCLCPP_DEBUG(node.get_logger(), "地图导出 revision=%lu export_ms=%.3f",
            publication.response.fine_revision,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - export_started).count());
        std::atomic_store(&published,
            std::make_shared<const PublishedExplorationMap>(std::move(publication)));
        // 位姿/等价TF只刷新当前anchor与支撑；源receipt仍通过Apply更新processed_stamp。
        if (same_fine) continue;
        const auto projection = map->Project();
        const auto& g = projection.geometry;
        nav_msgs::msg::OccupancyGrid message;
        message.header.frame_id = g.frame_id();
        message.header.stamp = node.now();
        message.info.resolution = g.resolution_m();
        message.info.width = g.width();
        message.info.height = g.height();
        message.info.origin.position.x =
            g.origin_m().x + g.min_inclusive().x * g.resolution_m();
        message.info.origin.position.y =
            g.origin_m().y + g.min_inclusive().y * g.resolution_m();
        message.info.origin.orientation.w = 1.;
        message.data = projection.data;
        publisher->publish(message);
        last_published_grid = std::move(message);
      } catch (const std::exception& error) {
        RCLCPP_ERROR(node.get_logger(), "地图更新失败: %s", error.what());
      }
    }
  }
};
ExplorationMapNode::ExplorationMapNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lunar_exploration_map_node", options),
      impl_(std::make_unique<Impl>(*this)) {}
ExplorationMapNode::~ExplorationMapNode() = default;
}  // namespace lunar::incremental_navigation_ros
