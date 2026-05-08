cmake_minimum_required(VERSION 3.8)
project(pct_dddmr_web)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/web_app.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_app
)

install(PROGRAMS scripts/publish_pcl_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME publish_pcl_pose
)

install(FILES scripts/core_ros.py scripts/nav_state.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY static launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
