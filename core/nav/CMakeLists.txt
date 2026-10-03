cmake_minimum_required(VERSION 3.8)
project(nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(map_inflation src/map_inflation_node.cpp)
ament_target_dependencies(map_inflation rclcpp nav_msgs)

add_executable(rrt_planner src/rrt_planner_node.cpp)
ament_target_dependencies(
  rrt_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(path_tracker src/path_tracker_node.cpp)
ament_target_dependencies(
  path_tracker
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

add_executable(local_planner src/local_planner_node.cpp)
ament_target_dependencies(
  local_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  map_inflation
  rrt_planner
  local_planner
  path_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs px4_msgs)

ament_package()
