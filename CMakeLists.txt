cmake_minimum_required(VERSION 3.8)
project(nav2_custom_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${nav2_behavior_tree_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Custom BT plugin library
add_library(nav2_custom_bt SHARED
  src/custom_bt_escape_node.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  nav2_util
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pluginlib
  pcl_conversions
  pcl_ros
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  nav2_util
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pluginlib
  pcl_conversions
  pcl_ros
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

ament_package()