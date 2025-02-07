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
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Custom BT plugin library
add_library(nav2_custom_bt SHARED
  src/custom_bt_node.cpp
)

target_compile_definitions(nav2_custom_bt PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(nav2_custom_bt
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  nav2_util
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugin.xml)

install(TARGETS
  nav2_custom_bt
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
ament_export_libraries(nav2_custom_bt)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  nav2_util
  pluginlib
)

ament_package()