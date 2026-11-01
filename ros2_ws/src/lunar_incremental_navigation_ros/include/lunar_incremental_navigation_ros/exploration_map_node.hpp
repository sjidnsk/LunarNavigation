#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace lunar::incremental_navigation_ros {
class ExplorationMapNode final : public rclcpp::Node {
 public:
  explicit ExplorationMapNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~ExplorationMapNode() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lunar::incremental_navigation_ros
