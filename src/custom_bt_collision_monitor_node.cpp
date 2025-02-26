#include "nav2_custom_bt/custom_bt_collision_monitor_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nav2_custom_bt
{

nav2_util::CallbackReturn CustomBTCollisionMonitorNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  
  // 파라미터 선언 및 로드
  this->declare_parameter("sensor_type", "scan");
  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("pointcloud_topic", "/points");
  
  // 폴리곤 파라미터 선언
  this->declare_parameter("slow_polygon_points", 
    std::vector<double>{0.8, 0.5, 0.8, -0.5, -0.8, -0.5, -0.8, 0.5});
  this->declare_parameter("stop_polygon_points", 
    std::vector<double>{0.5, 0.3, 0.5, -0.3, -0.5, -0.3, -0.5, 0.3});
  
  // 파라미터 로드
  sensor_type_ = this->get_parameter("sensor_type").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  
  // 폴리곤 파라미터 로드
  if (!this->get_parameter("slow_polygon_points", slow_polygon_points_)) {
    RCLCPP_WARN(this->get_logger(), "Using default slow polygon points");
  }
  if (!this->get_parameter("stop_polygon_points", stop_polygon_points_)) {
    RCLCPP_WARN(this->get_logger(), "Using default stop polygon points");
  }

  // 로드된 파라미터 출력
  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), "  sensor_type: %s", sensor_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  scan_topic: %s", scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  pointcloud_topic: %s", pointcloud_topic_.c_str());

  std::string slow_points_str = "  slow_polygon_points: ";
  for (const auto& point : slow_polygon_points_) {
    slow_points_str += std::to_string(point) + " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", slow_points_str.c_str());

  std::string stop_points_str = "  stop_polygon_points: ";
  for (const auto& point : stop_polygon_points_) {
    stop_points_str += std::to_string(point) + " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", stop_points_str.c_str());

  // Publisher 초기화
  slow_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/collision_slow_zone_polygon", 10);
  stop_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/collision_stop_zone_polygon", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 센서 타입에 따른 구독자 생성
  if (sensor_type_ == "scan") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&CustomBTCollisionMonitorNode::scanCallback, this, std::placeholders::_1));
  } else if (sensor_type_ == "pointcloud") {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&CustomBTCollisionMonitorNode::pointcloudCallback, this, std::placeholders::_1));
  }

  // 선택된 센서 타입에 따라 초기화
  scan_received_ = false;
  pointcloud_received_ = false;
  if (sensor_type_ == "scan") {
    scan_received_ = false;
  } else if (sensor_type_ == "pointcloud") {
    pointcloud_received_ = false;
  }

  // 폴리곤 발행 타이머
  polygon_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&CustomBTCollisionMonitorNode::publishPolygons, this));

  // 충돌 감지 타이머
  collision_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20Hz
    std::bind(&CustomBTCollisionMonitorNode::checkCollision, this));

  RCLCPP_INFO(this->get_logger(), "Created polygon publishers on topics: %s, %s",
    slow_polygon_pub_->get_topic_name(),
    stop_polygon_pub_->get_topic_name());

  RCLCPP_INFO(this->get_logger(), "CustomBTCollisionMonitorNode initialized");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CustomBTCollisionMonitorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  
  // Publisher 활성화
  slow_polygon_pub_->on_activate();
  stop_polygon_pub_->on_activate();
  cmd_vel_pub_->on_activate();
  
  // Bond 생성
  createBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CustomBTCollisionMonitorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  
  // Publisher 비활성화
  slow_polygon_pub_->on_deactivate();
  stop_polygon_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
  
  // Bond 해제
  destroyBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

void CustomBTCollisionMonitorNode::publishPolygons()
{
  RCLCPP_DEBUG(this->get_logger(), "Publishing polygons...");

  // Slow 폴리곤 발행
  auto slow_polygon_msg = geometry_msgs::msg::PolygonStamped();
  slow_polygon_msg.header.frame_id = "base_link";
  slow_polygon_msg.header.stamp = this->now();

  if (slow_polygon_points_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Slow polygon points is empty!");
    return;
  }

  for (size_t i = 0; i < slow_polygon_points_.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(slow_polygon_points_[i]);
    p.y = static_cast<float>(slow_polygon_points_[i + 1]);
    p.z = 0.0f;
    slow_polygon_msg.polygon.points.push_back(p);
  }

  // 폴리곤을 닫기 위해 첫 점을 다시 추가
  if (!slow_polygon_msg.polygon.points.empty()) {
    slow_polygon_msg.polygon.points.push_back(slow_polygon_msg.polygon.points.front());
  }

  // Stop 폴리곤 발행
  auto stop_polygon_msg = geometry_msgs::msg::PolygonStamped();
  stop_polygon_msg.header.frame_id = "base_link";
  stop_polygon_msg.header.stamp = this->now();

  if (stop_polygon_points_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Stop polygon points is empty!");
    return;
  }

  for (size_t i = 0; i < stop_polygon_points_.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(stop_polygon_points_[i]);
    p.y = static_cast<float>(stop_polygon_points_[i + 1]);
    p.z = 0.0f;
    stop_polygon_msg.polygon.points.push_back(p);
  }

  // 폴리곤을 닫기 위해 첫 점을 다시 추가
  if (!stop_polygon_msg.polygon.points.empty()) {
    stop_polygon_msg.polygon.points.push_back(stop_polygon_msg.polygon.points.front());
  }

  // 발행 상태 로그
  RCLCPP_INFO_THROTTLE(this->get_logger(), 
    *(this->get_clock()), 5000,
    "Publishing polygons - Slow: %zu points, Stop: %zu points, Topics: %s, %s",
    slow_polygon_msg.polygon.points.size(),
    stop_polygon_msg.polygon.points.size(),
    slow_polygon_pub_->get_topic_name(),
    stop_polygon_pub_->get_topic_name());

  // 폴리곤 발행
  slow_polygon_pub_->publish(slow_polygon_msg);
  stop_polygon_pub_->publish(stop_polygon_msg);
}

void CustomBTCollisionMonitorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
  scan_received_ = true;
  // TODO: 스캔 데이터를 이용한 충돌 감지 로직 구현
}

void CustomBTCollisionMonitorNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(get_logger(), "First pointcloud message received on topic: %s", 
                   pointcloud_sub_->get_topic_name());
  latest_pointcloud_ = msg;
  pointcloud_received_ = true;
  
  // 디버그 로그 추가
  RCLCPP_DEBUG(get_logger(), "Received pointcloud with %d points", 
               msg->width * msg->height);
}

void CustomBTCollisionMonitorNode::checkCollision()
{
  // 선택된 센서 타입에 따라 데이터 확인
  if (sensor_type_ == "scan") {
    if (!scan_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000, 
        "No scan data received yet");
      return;
    }
    // scan 데이터만 사용하여 충돌 감지
    // TODO: scan 기반 충돌 감지 로직 구현

  } else if (sensor_type_ == "pointcloud") {
    if (!pointcloud_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000, 
        "No pointcloud data received yet");
      return;
    }
    // pointcloud 데이터만 사용하여 충돌 감지
    // TODO: pointcloud 기반 충돌 감지 로직 구현
  }
}

nav2_util::CallbackReturn CustomBTCollisionMonitorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  
  // 타이머 정리
  polygon_timer_.reset();
  collision_check_timer_.reset();
  
  // Publisher 정리
  slow_polygon_pub_.reset();
  stop_polygon_pub_.reset();
  cmd_vel_pub_.reset();
  
  // Subscriber 정리
  scan_sub_.reset();
  pointcloud_sub_.reset();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CustomBTCollisionMonitorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

CustomBTCollisionMonitorNode::~CustomBTCollisionMonitorNode()
{
  RCLCPP_INFO(this->get_logger(), "CustomBTCollisionMonitorNode destroyed");
}

}  // namespace nav2_custom_bt 