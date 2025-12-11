cmake_minimum_required(VERSION 3.8)
project(tb3_yolo_bringup)

# g++ warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set (DEPENDENCIES
  "rclcpp"
  "rcutils"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
)

install(
  DIRECTORY launch rviz params maps urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()