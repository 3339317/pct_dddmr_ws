cmake_minimum_required(VERSION 3.8)
project(indoor_fusion_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(fusion_bridge_node src/fusion_bridge_node.cpp)
target_include_directories(fusion_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fusion_bridge_node
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs dddmr_sys_core)
target_link_libraries(fusion_bridge_node nlohmann_json::nlohmann_json)

install(TARGETS fusion_bridge_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
