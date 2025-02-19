#ifndef NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_
#define NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <thread>
#include <atomic>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nav2_custom_bt
{

class CustomBTNode : public BT::SyncActionNode
{
public:
  CustomBTNode(
    const std::string & name,
    const BT::NodeConfiguration & config);

  // 선언만 하고 정의는 cpp 파일로 이동
  static BT::PortsList providedPorts();

  ~CustomBTNode() override;

private:
  // 기존 polygon_points_를 front와 back으로 분리
  std::vector<double> front_polygon_points_;
  std::vector<double> back_polygon_points_;
  
  // 기존 변수들
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomBTNode")};
  rclcpp::Clock::SharedPtr clock_;
  
  double forward_distance_;
  double forward_speed_;
  // 후진 관련 변수 추가
  double backward_distance_;
  double backward_speed_;
  
  // 센서 타입 선택을 위한 변수 추가
  std::string sensor_type_;  // "scan" 또는 "pointcloud"
  std::string scan_topic_;   // scan 토픽 이름 추가
  std::string pointcloud_topic_;
  double pointcloud_min_height_;
  double pointcloud_max_height_;
  
  // 기존 함수 선언 수정
  bool isObstacleInFrontPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  bool isObstacleInBackPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  bool isObstacleInPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                          const std::vector<double>& polygon_points);

  // 기존 멤버들
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr front_polygon_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr back_polygon_pub_;
  rclcpp::TimerBase::SharedPtr polygon_timer_;
  
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  bool scan_received_{false};
  
  // PointCloud 구독자 추가
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  bool pointcloud_received_{false};
  
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishPolygons();
  void loadPolygonPoints();
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // 스캔 데이터 대기 관련 변수 추가
  std::atomic<bool> scan_wait_done_{false};
  std::atomic<bool> should_run_{true};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spin_thread_;
  
  void initialize();  // node_ 멤버를 사용하므로 파라미터 제거
  BT::NodeStatus tick() override;

  // 새로운 콜백 함수
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // PointCloud 데이터를 이용한 장애물 감지 함수
  bool isObstacleInPolygonPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
                                   const std::vector<double>& polygon_points);
  bool isPointInPolygon(double x, double y, const std::vector<double>& polygon_points);
  bool on_configure();
};

static BT::PortsList providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/scan_main", "Laser scan topic")
  };
}

}  // namespace nav2_custom_bt

#endif  // NAV2_CUSTOM_BT__CUSTOM_BT_NODE_HPP_ 