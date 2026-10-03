cmake_minimum_required(VERSION 3.8)
project(mine_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mission.msg"
  "msg/LocalStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  common/include
)

add_executable(task_manager task/src/task_manager_node.cpp)
ament_target_dependencies(task_manager rclcpp std_msgs geometry_msgs)
target_link_libraries(task_manager "${cpp_typesupport_target}")

add_executable(explorer explorer/src/explorer_node.cpp)
ament_target_dependencies(explorer rclcpp std_msgs geometry_msgs nav_msgs px4_msgs)
target_link_libraries(explorer "${cpp_typesupport_target}")

add_executable(global_planner global_planner/src/global_planner_node.cpp)
ament_target_dependencies(global_planner rclcpp std_msgs geometry_msgs nav_msgs px4_msgs)
target_link_libraries(global_planner "${cpp_typesupport_target}")

add_executable(local_controller local_controller/src/local_controller_node.cpp)
ament_target_dependencies(local_controller rclcpp std_msgs geometry_msgs nav_msgs px4_msgs)
target_link_libraries(local_controller "${cpp_typesupport_target}")

install(TARGETS
  task_manager
  explorer
  global_planner
  local_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_world.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  common/include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs nav_msgs px4_msgs
  rosidl_default_runtime
)

ament_package()
