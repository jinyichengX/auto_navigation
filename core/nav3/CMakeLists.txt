cmake_minimum_required(VERSION 3.8)
project(nav3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(offboard_control src/control/offboard_control.cpp)
ament_target_dependencies(
  offboard_control
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

install(TARGETS offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs px4_msgs)

ament_package()
