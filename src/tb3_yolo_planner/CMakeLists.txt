cmake_minimum_required(VERSION 3.8)
project(tb3_yolo_planner)

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

# tb3_yolo_planner node
add_executable(tb3_yolo_planner src/tb3_yolo_planner.cpp)
# target_include_directories(tb3_yolo_planner PUBLIC include)
ament_target_dependencies(tb3_yolo_planner ${DEPENDENCIES})
target_compile_features(tb3_yolo_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS tb3_yolo_planner DESTINATION lib/${PROJECT_NAME})

ament_package()