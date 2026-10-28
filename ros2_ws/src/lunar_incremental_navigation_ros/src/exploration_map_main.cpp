#include <rclcpp/rclcpp.hpp>
#include "lunar_incremental_navigation_ros/exploration_map_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node=std::make_shared<lunar::incremental_navigation_ros::ExplorationMapNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
