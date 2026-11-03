#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include "lunar_incremental_navigation_ros/exploration_map_node.hpp"

namespace lunar::incremental_navigation_ros {
namespace {
class ExplorationMapConsumer : public ::testing::Test {
 protected:
  void SetUp() override { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
  void Consume(const std::string& config, const std::string& selector = "") {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params{
        {"platform_config", std::string(PLATFORM_CONFIG_DIR) + "/" + config},
        {"base_map_facts_topic", "/platform_consumer/facts"},
        {"exploration_map_topic", "/platform_consumer/map"}};
    if (!selector.empty()) params.emplace_back("platform_selector", selector);
    options.parameter_overrides(params);
    auto map = std::make_shared<ExplorationMapNode>(options);
    auto probe = std::make_shared<rclcpp::Node>("platform_consumer_probe");
    auto publisher = probe->create_publisher<std_msgs::msg::String>(
        "/platform_consumer/facts", rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr received;
    auto subscription = probe->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/platform_consumer/map", rclcpp::QoS(1).reliable().transient_local(),
        [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { received = msg; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(map); executor.add_node(probe);
    std_msgs::msg::String message;
    message.data = R"facts({"schema":"lunar-observation-facts/v1","source_resolution_m":0.2,"raw_revision":1,"frame":"map","epoch":7,"complete_snapshot":true,"observation_fragments":[{"polygon":[[0,0],[4,0],[4,4],[0,4]],"height_m":0,"slope_rad":0,"relief_m":0,"state":0}],"processed_stamp_ns":123456789})facts";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      publisher->publish(message);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(received) << "actual map node must consume shared facts and publish projection";
    EXPECT_EQ(received->header.frame_id, "map");
    EXPECT_NEAR(received->info.resolution, .2, 1e-6);
    EXPECT_FALSE(received->data.empty());
    EXPECT_NE(std::find(received->data.begin(), received->data.end(), 0), received->data.end());
    EXPECT_EQ(map->get_parameter("platform_selector").as_string(), selector.empty() ? "wheel" : selector);
  }
};
TEST_F(ExplorationMapConsumer, LeggedOrinConsumesSharedFacts) { Consume("legged_orin.yaml", "legged"); }
TEST_F(ExplorationMapConsumer, DefaultWheelConsumesSharedFacts) { Consume("wheel.yaml"); }
TEST_F(ExplorationMapConsumer, InvalidSelectorFailsBeforeSubscription) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("platform_selector", "hover")});
  EXPECT_THROW(std::make_shared<ExplorationMapNode>(options), std::invalid_argument);
}
}  // namespace
}  // namespace lunar::incremental_navigation_ros
