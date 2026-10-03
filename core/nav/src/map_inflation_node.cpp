#include <memory>
#include <string>

#include "nav/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * 通用地图膨胀节点：
 * 订原始 OccupancyGrid，按半径做圆形膨胀，发布给 RRT 使用。
 * 不绑定具体机器人/仿真；话题与阈值全部参数化。
 */
class MapInflationNode : public rclcpp::Node
{
public:
  MapInflationNode()
  : Node("map_inflation")
  {
    input_topic_ = declare_parameter<std::string>("input_map_topic", "/projected_map");
    output_topic_ = declare_parameter<std::string>("output_map_topic", "/inflated_map");
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.45);
    occ_threshold_ = declare_parameter<int>("occ_threshold", 50);
    inflate_unknown_ = declare_parameter<bool>("inflate_unknown", false);

    // 输入/输出都用 TRANSIENT_LOCAL：
    // 1) 兼容 octomap latch:=true 的静态图（只发一次也能收到）
    // 2) 晚启动的 RRT 也能拿到最新膨胀图
    // 注意：ros2 topic hz 对 latched 单次话题常显示“未发布”，用 echo/info 判断。
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, map_qos,
      std::bind(&MapInflationNode::on_map, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "map_inflation: %s -> %s, radius=%.2fm, occ>=%d, inflate_unknown=%s",
      input_topic_.c_str(), output_topic_.c_str(), inflation_radius_,
      occ_threshold_, inflate_unknown_ ? "true" : "false");
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    auto out = nav_pkg::inflate_occupancy_grid(
      *msg, inflation_radius_, occ_threshold_, inflate_unknown_);
    out.header = msg->header;
    out.info = msg->info;
    pub_->publish(out);

    if ((now() - last_log_).seconds() >= 2.0) {
      last_log_ = now();
      RCLCPP_INFO(
        get_logger(),
        "已发布膨胀图 %dx%d res=%.2f frame=%s",
        out.info.width, out.info.height, out.info.resolution,
        out.header.frame_id.c_str());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double inflation_radius_{0.45};
  int occ_threshold_{50};
  bool inflate_unknown_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapInflationNode>());
  rclcpp::shutdown();
  return 0;
}
