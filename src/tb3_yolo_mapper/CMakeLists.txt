cmake_minimum_required(VERSION 3.8)
project(tb3_yolo_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set (DEPENDENCIES
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "nav_msgs"
)

######## mapper ########
add_executable(tb3_yolo_mapper src/tb3_yolo_mapper.cpp)
# target_include_directories(tb3_yolo_mapper PUBLIC include)
ament_target_dependencies(tb3_yolo_mapper ${DEPENDENCIES})
target_compile_features(tb3_yolo_mapper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS tb3_yolo_mapper DESTINATION lib/${PROJECT_NAME})

ament_package()