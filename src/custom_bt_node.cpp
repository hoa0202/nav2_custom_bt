#include "nav2_custom_bt/custom_bt_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nav2_custom_bt
{

CustomBTNode::CustomBTNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  initialize();
}

void CustomBTNode::initialize()
{
  node_ = std::make_shared<rclcpp::Node>("custom_bt_node");
  clock_ = node_->get_clock();
  
  // scan_topic 파라미터 선언
  node_->declare_parameter("custom_bt_node.scan_topic", "/scan");  // 기본값은 "/scan"
  
  // 파라미터 선언
  node_->declare_parameter("custom_bt_node.forward_distance", 0.5);
  node_->declare_parameter("custom_bt_node.forward_speed", 0.2);
  node_->declare_parameter("custom_bt_node.backward_distance", 0.5);
  node_->declare_parameter("custom_bt_node.backward_speed", 0.2);
  
  // scan_topic 파라미터 로드
  std::string scan_topic = node_->get_parameter("custom_bt_node.scan_topic").as_string();
  
  // 파라미터 로드
  forward_distance_ = node_->get_parameter("custom_bt_node.forward_distance").as_double();
  forward_speed_ = node_->get_parameter("custom_bt_node.forward_speed").as_double();
  backward_distance_ = node_->get_parameter("custom_bt_node.backward_distance").as_double();
  backward_speed_ = node_->get_parameter("custom_bt_node.backward_speed").as_double();
  
  // 파라미터 선언
  node_->declare_parameter("custom_bt_node.front_polygon_points", 
    std::vector<double>{1.5, 1.5, 1.5, -1.5, 0.0, -0.3, 0.0, 0.3});
  node_->declare_parameter("custom_bt_node.back_polygon_points", 
    std::vector<double>{-1.0, 0.5, -1.0, -0.5, -0.49, -0.3, -0.49, 0.3});
  
  // 파라미터 로드
  if (!node_->get_parameter("custom_bt_node.front_polygon_points", front_polygon_points_)) {
    RCLCPP_WARN(logger_, "Using default front polygon points");
  }
  if (!node_->get_parameter("custom_bt_node.back_polygon_points", back_polygon_points_)) {
    RCLCPP_WARN(logger_, "Using default back polygon points");
  }

  // 로그 추가
  RCLCPP_INFO(logger_, "로드된 파라미터 값:");
  RCLCPP_INFO(logger_, "scan_topic: %s", scan_topic.c_str());
  RCLCPP_INFO(logger_, "forward_distance: %.2f", forward_distance_);
  RCLCPP_INFO(logger_, "forward_speed: %.2f", forward_speed_);
  RCLCPP_INFO(logger_, "backward_distance: %.2f", backward_distance_);
  RCLCPP_INFO(logger_, "backward_speed: %.2f", backward_speed_);

  RCLCPP_INFO(logger_, "Loaded polygon points:");
  std::string front_points_str = "Front: ";
  for (const auto& point : front_polygon_points_) {
    front_points_str += std::to_string(point) + " ";
  }
  RCLCPP_INFO(logger_, "%s", front_points_str.c_str());
  
  std::string back_points_str = "Back: ";
  for (const auto& point : back_polygon_points_) {
    back_points_str += std::to_string(point) + " ";
  }
  RCLCPP_INFO(logger_, "%s", back_points_str.c_str());

  // Publisher 초기화
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  front_polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/front_detection_zone_polygon", 10);
  back_polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/back_detection_zone_polygon", 10);
  
  // Subscriber 초기화
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 10,
    std::bind(&CustomBTNode::scanCallback, this, std::placeholders::_1));
  
  // 폴리곤 발행 타이머
  polygon_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&CustomBTNode::publishPolygons, this));
  
  // 노드 스핀을 위한 실행자 생성
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  
  // 별도 스레드에서 실행자 실행
  spin_thread_ = std::make_unique<std::thread>([this]() {
    while (should_run_ && rclcpp::ok()) {
      if (executor_) {
        executor_->spin_some();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // 초기화 시 선택된 센서 타입의 데이터를 기다림 (별도 스레드에서)
  std::thread sensor_wait_thread([this]() {
    rclcpp::WallRate rate(10);
    auto start = node_->now();
    while (!scan_received_ && (node_->now() - start).seconds() < 5.0 && rclcpp::ok()) {
      if (sensor_type_ == "scan") {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Waiting for initial scan data...");
      } else if (sensor_type_ == "pointcloud") {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Waiting for initial pointcloud data...");
      }
      rate.sleep();
    }
    scan_wait_done_ = true;
  });
  sensor_wait_thread.detach();

  // 새로운 파라미터 선언
  node_->declare_parameter("custom_bt_node.sensor_type", "scan");
  node_->declare_parameter("custom_bt_node.pointcloud_topic", "/livox/lidar");
  node_->declare_parameter("custom_bt_node.pointcloud_min_height", 0.1);
  node_->declare_parameter("custom_bt_node.pointcloud_max_height", 0.5);
  
  // 파라미터 로드
  sensor_type_ = node_->get_parameter("custom_bt_node.sensor_type").as_string();
  pointcloud_topic_ = node_->get_parameter("custom_bt_node.pointcloud_topic").as_string();
  pointcloud_min_height_ = node_->get_parameter("custom_bt_node.pointcloud_min_height").as_double();
  pointcloud_max_height_ = node_->get_parameter("custom_bt_node.pointcloud_max_height").as_double();
  
  // 센서 타입에 따른 구독자 생성
  if (sensor_type_ == "scan") {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&CustomBTNode::scanCallback, this, std::placeholders::_1));
  } else if (sensor_type_ == "pointcloud") {
    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&CustomBTNode::pointcloudCallback, this, std::placeholders::_1));
  }
}

BT::NodeStatus CustomBTNode::tick()
{
  // 선택된 센서 데이터 대기가 완료될 때까지 RUNNING 반환
  if (!scan_wait_done_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!scan_received_) {
    if (sensor_type_ == "scan") {
      RCLCPP_WARN(logger_, "Failed to receive initial scan data after 5 seconds");
    } else if (sensor_type_ == "pointcloud") {
      RCLCPP_WARN(logger_, "Failed to receive initial pointcloud data after 5 seconds");
    }
    return BT::NodeStatus::FAILURE;
  }

  // 전방 장애물 체크
  if (!isObstacleInFrontPolygon(latest_scan_)) {
    RCLCPP_INFO(logger_, "전방 장애물이 없습니다. 전진합니다.");
    
    // 전진 명령 발행
    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = forward_speed_;
    
    // forward_distance만큼 이동하기 위한 시간 계산
    double time_to_move = forward_distance_ / forward_speed_;
    auto start_time = clock_->now();
    
    // 이동 중에는 계속 속도 명령을 발행
    while ((clock_->now() - start_time).seconds() < time_to_move) {
      if (!rclcpp::ok()) {
        return BT::NodeStatus::FAILURE;
      }
      
      cmd_vel_pub_->publish(cmd_vel);
      
      // 이동 중 장애물 체크
      if (isObstacleInFrontPolygon(latest_scan_)) {
        RCLCPP_WARN(logger_, "이동 중 장애물 감지. 정지합니다.");
        cmd_vel.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        return BT::NodeStatus::FAILURE;
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // 정지 명령 발행
    cmd_vel.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    
    RCLCPP_INFO(logger_, "목표 거리만큼 이동 완료");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(logger_, "전방 장애물이 감지되었습니다. 후방을 확인합니다.");
    
    // 후방 장애물 체크
    if (!isObstacleInBackPolygon(latest_scan_)) {
      RCLCPP_INFO(logger_, "후방 장애물이 없습니다. 후진합니다.");
      
      // 후진 명령 발행
      auto cmd_vel = geometry_msgs::msg::Twist();
      cmd_vel.linear.x = -backward_speed_;
      
      RCLCPP_INFO(logger_, "Publishing cmd_vel: linear.x = %.2f", cmd_vel.linear.x);
      cmd_vel_pub_->publish(cmd_vel);
      
      // 로그 추가: 후진 속도와 거리 출력
      RCLCPP_INFO(logger_, "후진 속도: %.2f m/s, 후진 거리: %.2f m", -backward_speed_, backward_distance_);
      
      // backward_distance만큼 이동
      double time_to_move = backward_distance_ / backward_speed_;
      auto start_time = clock_->now();
      
      RCLCPP_INFO(logger_, "계산된 후진 시간: %.2f 초", time_to_move);
      
      while ((clock_->now() - start_time).seconds() < time_to_move && rclcpp::ok()) {
        cmd_vel_pub_->publish(cmd_vel);
        
        // 후진 중 후방 장애물 체크
        if (isObstacleInBackPolygon(latest_scan_)) {
          RCLCPP_WARN(logger_, "후진 중 후방 장애물 감지. 정지합니다.");
          cmd_vel.linear.x = 0.0;
          cmd_vel_pub_->publish(cmd_vel);
          return BT::NodeStatus::FAILURE;
        }
        
        // 로그 추가: 현재 속도 출력
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "현재 후진 속도: %.2f m/s", cmd_vel.linear.x);
        
        // 짧은 주기로 sleep하여 반응성 유지
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      
      // 정지 명령 발행
      cmd_vel.linear.x = 0.0;
      cmd_vel_pub_->publish(cmd_vel);
      
      RCLCPP_INFO(logger_, "후진 완료");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_INFO(logger_, "전후방 모두 장애물이 있습니다. 정지합니다.");
      return BT::NodeStatus::FAILURE;
    }
  }
}

void CustomBTNode::loadPolygonPoints()
{
  try {
    auto parent_node = node_->get_node_base_interface()->get_fully_qualified_name();
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      node_, parent_node);
    
    if (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(logger_, "Parameter service not available");
      return;
    }

    // 전방 폴리곤 파라미터 로드
    auto front_params = parameters_client->get_parameters({"custom_bt_node.front_polygon_points"});
    if (!front_params.empty()) {
      front_polygon_points_ = front_params[0].as_double_array();
      RCLCPP_INFO(logger_, "Successfully loaded front polygon points");
      for (size_t i = 0; i < front_polygon_points_.size(); i += 2) {
        RCLCPP_INFO(logger_, "Front Point %zu: (%.2f, %.2f)", 
          i/2, front_polygon_points_[i], front_polygon_points_[i + 1]);
      }
    } else {
      RCLCPP_WARN(logger_, "Using default front polygon points");
      front_polygon_points_ = {0.5, 0.5, 0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
    }

    // 후방 폴리곤 파라미터 로드
    auto back_params = parameters_client->get_parameters({"custom_bt_node.back_polygon_points"});
    if (!back_params.empty()) {
      back_polygon_points_ = back_params[0].as_double_array();
      RCLCPP_INFO(logger_, "Successfully loaded back polygon points");
      for (size_t i = 0; i < back_polygon_points_.size(); i += 2) {
        RCLCPP_INFO(logger_, "Back Point %zu: (%.2f, %.2f)", 
          i/2, back_polygon_points_[i], back_polygon_points_[i + 1]);
      }
    } else {
      RCLCPP_WARN(logger_, "Using default back polygon points");
      back_polygon_points_ = {-0.5, 0.5, -0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to load polygon points: %s", e.what());
    // 기본값 설정
    front_polygon_points_ = {0.5, 0.5, 0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
    back_polygon_points_ = {-0.5, 0.5, -0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
  }
}

BT::PortsList CustomBTNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/scan_main", "Laser scan topic"),
    BT::InputPort<double>("forward_distance", 0.5, "Forward movement distance"),
    BT::InputPort<double>("forward_speed", 0.2, "Forward movement speed"),
    BT::InputPort<double>("backward_distance", 0.5, "Backward movement distance"),
    BT::InputPort<double>("backward_speed", 0.2, "Backward movement speed")
  };
}

bool CustomBTNode::isObstacleInFrontPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  return isObstacleInPolygon(scan, front_polygon_points_);
}

bool CustomBTNode::isObstacleInBackPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  return isObstacleInPolygon(scan, back_polygon_points_);
}

bool CustomBTNode::isObstacleInPolygon(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                                    const std::vector<double>& polygon_points)
{
  if (!scan) return true;  // 안전을 위해 스캔 데이터가 없으면 장애물이 있다고 가정

  // 각 스캔 포인트에 대해 검사
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double range = scan->ranges[i];
    
    // 유효한 범위의 스캔 데이터만 처리
    if (!std::isinf(range) && !std::isnan(range) && 
        range >= scan->range_min && range <= scan->range_max) {
      
      // 스캔 포인트의 x, y 좌표 계산
      double angle = scan->angle_min + i * scan->angle_increment;
      double x = range * cos(angle);
      double y = range * sin(angle);
      
      // 포인트가 폴리곤 내부에 있는지 확인
      bool inside = false;
      for (size_t j = 0, k = polygon_points.size() - 2; j < polygon_points.size(); k = j, j += 2) {
        if (((polygon_points[j + 1] > y) != (polygon_points[k + 1] > y)) &&
            (x < (polygon_points[k] - polygon_points[j]) * (y - polygon_points[j + 1]) /
                    (polygon_points[k + 1] - polygon_points[j + 1]) + polygon_points[j])) {
          inside = !inside;
        }
      }
      
      // 폴리곤 내부에 스캔 포인트가 있다면 장애물이 있는 것
      if (inside) {
        RCLCPP_DEBUG(logger_, "Obstacle detected at (%.2f, %.2f)", x, y);
        return true;
      }
    }
  }
  
  return false;  // 장애물 없음
}

void CustomBTNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!scan_received_) {
    RCLCPP_INFO(logger_, "First scan message received on topic: %s", scan_sub_->get_topic_name());
  }
  latest_scan_ = msg;
  scan_received_ = true;
}

void CustomBTNode::publishPolygons()
{
  // 전방 폴리곤 발행
  auto front_polygon_msg = geometry_msgs::msg::PolygonStamped();
  front_polygon_msg.header.frame_id = "base_link";
  front_polygon_msg.header.stamp = node_->now();
  
  if (front_polygon_points_.empty()) {
    RCLCPP_WARN(logger_, "Front polygon points is empty, using default values");
    front_polygon_points_ = {0.5, 0.5, 0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
  }

  for (size_t i = 0; i < front_polygon_points_.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(front_polygon_points_[i]);
    p.y = static_cast<float>(front_polygon_points_[i + 1]);
    p.z = 0.0f;
    front_polygon_msg.polygon.points.push_back(p);
  }

  // 폴리곤을 닫기 위해 첫 점을 다시 추가
  if (!front_polygon_msg.polygon.points.empty()) {
    front_polygon_msg.polygon.points.push_back(front_polygon_msg.polygon.points.front());
  }

  // 후방 폴리곤 발행
  auto back_polygon_msg = geometry_msgs::msg::PolygonStamped();
  back_polygon_msg.header.frame_id = "base_link";
  back_polygon_msg.header.stamp = node_->now();
  
  if (back_polygon_points_.empty()) {
    RCLCPP_WARN(logger_, "Back polygon points is empty, using default values");
    back_polygon_points_ = {-0.5, 0.5, -0.5, -0.5, 0.0, -0.3, 0.0, 0.3};
  }

  for (size_t i = 0; i < back_polygon_points_.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(back_polygon_points_[i]);
    p.y = static_cast<float>(back_polygon_points_[i + 1]);
    p.z = 0.0f;
    back_polygon_msg.polygon.points.push_back(p);
  }

  // 폴리곤을 닫기 위해 첫 점을 다시 추가
  if (!back_polygon_msg.polygon.points.empty()) {
    back_polygon_msg.polygon.points.push_back(back_polygon_msg.polygon.points.front());
  }

  // RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, 
  //   "Publishing polygons - Front: %zu points, Back: %zu points",
  //   front_polygon_msg.polygon.points.size(),
  //   back_polygon_msg.polygon.points.size());

  front_polygon_pub_->publish(front_polygon_msg);
  back_polygon_pub_->publish(back_polygon_msg);
}

void CustomBTNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!scan_received_) {
    RCLCPP_INFO(logger_, "First pointcloud message received on topic: %s", 
                pointcloud_sub_->get_topic_name());
    scan_received_ = true;  // 포인트클라우드 데이터를 받았음을 표시
  }
  
  // PointCloud2를 PCL 포맷으로 변환
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  
  // 높이 필터링된 포인트를 LaserScan 형식으로 변환
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header = msg->header;
  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;
  scan->angle_increment = 0.01;  // 약 1도
  scan->range_min = 0.1;
  scan->range_max = 100.0;
  
  size_t num_angles = static_cast<size_t>((scan->angle_max - scan->angle_min) / 
                                        scan->angle_increment);
  scan->ranges.resize(num_angles, std::numeric_limits<float>::infinity());
  
  // 각 포인트를 처리
  for (const auto& point : cloud->points) {
    // 높이 필터링
    if (point.z >= pointcloud_min_height_ && point.z <= pointcloud_max_height_) {
      // x-y 평면에서의 거리와 각도 계산
      float range = std::sqrt(point.x * point.x + point.y * point.y);
      float angle = std::atan2(point.y, point.x);
      
      // 각도를 인덱스로 변환
      int index = static_cast<int>((angle - scan->angle_min) / scan->angle_increment);
      if (index >= 0 && index < static_cast<int>(num_angles)) {
        // 더 가까운 거리로 업데이트
        scan->ranges[index] = std::min(scan->ranges[index], range);
      }
    }
  }
  
  latest_scan_ = scan;
  scan_received_ = true;
}

bool CustomBTNode::isObstacleInPolygonPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
  const std::vector<double>& polygon_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  
  for (const auto& point : pcl_cloud->points) {
    // 높이 필터링
    if (point.z >= pointcloud_min_height_ && point.z <= pointcloud_max_height_) {
      // 포인트가 폴리곤 내부에 있는지 확인
      bool inside = false;
      for (size_t j = 0, k = polygon_points.size() - 2; j < polygon_points.size(); k = j, j += 2) {
        if (((polygon_points[j + 1] > point.y) != (polygon_points[k + 1] > point.y)) &&
            (point.x < (polygon_points[k] - polygon_points[j]) * (point.y - polygon_points[j + 1]) /
                      (polygon_points[k + 1] - polygon_points[j + 1]) + polygon_points[j])) {
          inside = !inside;
        }
      }
      
      if (inside) {
        RCLCPP_DEBUG(logger_, "Obstacle detected at (%.2f, %.2f, %.2f)", 
                    point.x, point.y, point.z);
        return true;
      }
    }
  }
  
  return false;
}

// 소멸자 추가
CustomBTNode::~CustomBTNode()
{
  should_run_ = false;
  
  if (executor_) {
    executor_->cancel();
  }
  
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
}

bool CustomBTNode::on_configure()
{
  // ... 기존 코드 ...
  
  // 파라미터 가져오기
  node_->get_parameter_or("sensor_type", sensor_type_, std::string("scan"));
  node_->get_parameter_or("scan_topic", scan_topic_, std::string("/scan"));
  node_->get_parameter_or("pointcloud_topic", pointcloud_topic_, std::string("/points"));
  
  // 선택된 센서 타입에 따라 적절한 구독자 생성
  if (sensor_type_ == "scan") {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, 
      std::bind(&CustomBTNode::scanCallback, this, std::placeholders::_1));
  } else if (sensor_type_ == "pointcloud") {
    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&CustomBTNode::pointcloudCallback, this, std::placeholders::_1));
  }
  
  return true;
}

}  // namespace nav2_custom_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt::CustomBTNode>("CustomBTNode");
} 