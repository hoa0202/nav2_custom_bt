#ifndef NAV2_CUSTOM_BT__CUSTOM_BT_COLLISION_MONITOR_NODE_HPP_
#define NAV2_CUSTOM_BT__CUSTOM_BT_COLLISION_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_util/lifecycle_node.hpp>

namespace nav2_custom_bt
{

class CustomBTCollisionMonitorNode : public nav2_util::LifecycleNode
{
public:
  CustomBTCollisionMonitorNode()
    : nav2_util::LifecycleNode("custom_bt_collision_monitor")
  {
    RCLCPP_INFO(get_logger(), "Creating custom_bt_collision_monitor");
  }
  ~CustomBTCollisionMonitorNode();

protected:
  // Lifecycle 관련 콜백 추가
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // 기존 멤버들은 그대로 유지
  void publishPolygons();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool isPointInPolygon(double x, double y, const std::vector<double>& polygon_points);
  
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr slow_polygon_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr stop_polygon_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr polygon_timer_;
  rclcpp::TimerBase::SharedPtr collision_check_timer_;
  
  // Parameters
  std::string sensor_type_;
  std::string scan_topic_;
  std::string pointcloud_topic_;
  double pointcloud_min_height_;  // 최소 높이
  double pointcloud_max_height_;  // 최대 높이
  double slow_speed_ratio_{0.2};  // 기본값 20%
  std::vector<double> slow_polygon_points_;
  std::vector<double> stop_polygon_points_;
  bool obstacle_in_slow_zone_{false};
  bool obstacle_in_stop_zone_{false};  // stop zone 장애물 감지 플래그 추가
  
  // Data
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  bool scan_received_{false};
  bool pointcloud_received_{false};
  
  void checkCollision();
};

}  // namespace nav2_custom_bt

#endif  // NAV2_CUSTOM_BT__CUSTOM_BT_COLLISION_MONITOR_NODE_HPP_ 