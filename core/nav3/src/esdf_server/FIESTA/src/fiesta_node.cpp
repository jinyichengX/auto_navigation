#include "Fiesta.h"

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fiesta");
  fiesta::Fiesta<sensor_msgs::msg::PointCloud2::SharedPtr,
                 nav_msgs::msg::Odometry::SharedPtr>
      esdf_map(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
