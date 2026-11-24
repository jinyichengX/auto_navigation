cmake_minimum_required(VERSION 3.8)
project(nav3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

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

add_executable(fastlio2_input_bridge src/bridge/fastlio2_input_bridge.cpp)
ament_target_dependencies(
  fastlio2_input_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  px4_msgs
)
target_link_libraries(
  fastlio2_input_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

add_executable(fastlio_odom_bridge src/bridge/fastlio_odom_bridge.cpp)
ament_target_dependencies(
  fastlio_odom_bridge
  rclcpp
  nav_msgs
  px4_msgs
)

add_executable(odom_lp_compare src/bridge/odom_lp_compare.cpp)
ament_target_dependencies(
  odom_lp_compare
  rclcpp
  nav_msgs
  px4_msgs
)

add_executable(nbvplanner_explorer
  src/explorer/nbvplanner_explorer.cpp
  src/explorer/rrt.cpp
  src/explorer/official/kdtree/kdtree.c
)
ament_target_dependencies(
  nbvplanner_explorer
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  px4_msgs
  visualization_msgs
)
target_include_directories(nbvplanner_explorer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/explorer
  ${CMAKE_CURRENT_SOURCE_DIR}/src/explorer/official
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(nbvplanner_explorer ${OCTOMAP_LIBRARIES})

add_executable(esdf_server src/esdf_server/esdf_server.cpp)
ament_target_dependencies(
  esdf_server
  rclcpp
  sensor_msgs
  octomap_msgs
  pcl_conversions
)
target_include_directories(esdf_server PUBLIC ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(esdf_server ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

set(FIESTA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/esdf_server/FIESTA)
add_executable(fiesta_esdf
  ${FIESTA_DIR}/src/fiesta_node.cpp
  ${FIESTA_DIR}/src/parameters.cpp
  ${FIESTA_DIR}/src/ESDFMap.cpp
  ${FIESTA_DIR}/src/raycast.cpp
  ${FIESTA_DIR}/src/timing.cc
)
ament_target_dependencies(
  fiesta_esdf
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
  cv_bridge
)
target_include_directories(fiesta_esdf PUBLIC
  ${FIESTA_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fiesta_esdf
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  offboard_control
  fastlio2_input_bridge
  fastlio_odom_bridge
  odom_lp_compare
  nbvplanner_explorer
  esdf_server
  fiesta_esdf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/ src/esdf_server/FIESTA/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  px4_msgs
  octomap
  octomap_msgs
  pcl_conversions
  cv_bridge
)

ament_package()
