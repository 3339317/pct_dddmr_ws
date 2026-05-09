cmake_minimum_required(VERSION 3.8)
project(lego_loam_bor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cloud_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  cloud_msgs
  pcl_conversions
  livox_ros_driver2
  cv_bridge
)

add_library(image_projection SHARED
  src/imageProjection.cpp
)
target_link_libraries(image_projection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(image_projection
  ${dependencies}
)

add_library(feature_association SHARED
  src/featureAssociation.cpp
)
target_link_libraries(feature_association
  ${PCL_LIBRARIES}
)
ament_target_dependencies(feature_association
  ${dependencies}
)

add_executable(mcl_feature
  src/mcl_feature_node.cpp
)
target_link_libraries(mcl_feature
  image_projection
  feature_association
)
ament_target_dependencies(mcl_feature
  ${dependencies}
)

install(TARGETS
  image_projection
  feature_association
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mcl_feature
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(image_projection feature_association)
ament_export_dependencies(${dependencies})
ament_package()
