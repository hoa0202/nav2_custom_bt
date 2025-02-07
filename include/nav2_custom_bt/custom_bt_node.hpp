#ifndef NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_
#define NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_custom_bt
{

class CustomBTNode : public BT::SyncActionNode
{
public:
  CustomBTNode(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomBTNode")};  // 정적 로거 사용
};

}  // namespace nav2_custom_bt

#endif  // NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_ 