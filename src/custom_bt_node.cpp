#include "nav2_custom_bt/custom_bt_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/node_utils.hpp"

namespace nav2_custom_bt
{

CustomBTNode::CustomBTNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  // 노드 생성하지 않고 로거만 사용
}

BT::PortsList CustomBTNode::providedPorts()
{
  return {};
}

BT::NodeStatus CustomBTNode::tick()
{
  RCLCPP_INFO(logger_, "커스텀 BT 노드 실행!");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_custom_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt::CustomBTNode>("CustomBTNode");
} 