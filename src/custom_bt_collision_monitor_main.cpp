#include "nav2_custom_bt/custom_bt_collision_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_custom_bt::CustomBTCollisionMonitorNode>();  // 클래스 이름 변경
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
} 