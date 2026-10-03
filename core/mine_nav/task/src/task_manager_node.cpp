#include <chrono>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mine_nav/msg/mission.hpp"

using namespace std::chrono_literals;

/**
 * 任务层：告诉整套系统“这次怎么飞”。
 * 人给目标/范围/禁飞区/优先级 -> 本节点整理成 Mission 持续发布。
 * 探索器只认 Mission，不再直接听人的零散命令。
 */
class TaskManagerNode : public rclcpp::Node
{
public:
  TaskManagerNode()
  : Node("task_manager")
  {
    mode_ = declare_parameter<std::string>("mode", "explore");
    priority_ = static_cast<uint8_t>(declare_parameter<int>("priority", 50));
    fly_alt_enu_ = static_cast<float>(declare_parameter<double>("fly_alt_enu", 1.2));
    return_home_when_done_ = declare_parameter<bool>("return_home_when_done", true);
    note_ = declare_parameter<std::string>("note", "默认井下探索任务");

    area_min_x_ = declare_parameter<double>("area_min_x", -10.0);
    area_min_y_ = declare_parameter<double>("area_min_y", -10.0);
    area_max_x_ = declare_parameter<double>("area_max_x", 10.0);
    area_max_y_ = declare_parameter<double>("area_max_y", 10.0);

    has_goal_ = declare_parameter<bool>("has_goal", false);
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);

    // 禁飞区：每 4 个数一组 xmin ymin xmax ymax
    // 显式 ParameterValue，避免 yaml 里写 [] 时类型推断失败
    declare_parameter("no_fly_zones", rclcpp::ParameterValue(std::vector<double>{}));
    no_fly_flat_ = get_parameter("no_fly_zones").as_double_array();

    mission_pub_ = create_publisher<mine_nav::msg::Mission>("/mission", 10);
    timer_ = create_wall_timer(500ms, std::bind(&TaskManagerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "任务层已启动：mode=%s priority=%u area=[%.1f,%.1f]~[%.1f,%.1f]",
      mode_.c_str(), priority_, area_min_x_, area_min_y_, area_max_x_, area_max_y_);
  }

private:
  void reload_params()
  {
    mode_ = get_parameter("mode").as_string();
    priority_ = static_cast<uint8_t>(get_parameter("priority").as_int());
    fly_alt_enu_ = static_cast<float>(get_parameter("fly_alt_enu").as_double());
    return_home_when_done_ = get_parameter("return_home_when_done").as_bool();
    note_ = get_parameter("note").as_string();
    area_min_x_ = get_parameter("area_min_x").as_double();
    area_min_y_ = get_parameter("area_min_y").as_double();
    area_max_x_ = get_parameter("area_max_x").as_double();
    area_max_y_ = get_parameter("area_max_y").as_double();
    has_goal_ = get_parameter("has_goal").as_bool();
    goal_x_ = get_parameter("goal_x").as_double();
    goal_y_ = get_parameter("goal_y").as_double();
    no_fly_flat_ = get_parameter("no_fly_zones").as_double_array();
  }

  void tick()
  {
    reload_params();

    mine_nav::msg::Mission msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_init";
    msg.mode = mode_;
    msg.priority = priority_;
    msg.area_min.x = area_min_x_;
    msg.area_min.y = area_min_y_;
    msg.area_min.z = 0.0;
    msg.area_max.x = area_max_x_;
    msg.area_max.y = area_max_y_;
    msg.area_max.z = 0.0;
    msg.has_goal = has_goal_;
    msg.goal.x = goal_x_;
    msg.goal.y = goal_y_;
    msg.goal.z = fly_alt_enu_;
    msg.fly_alt_enu = fly_alt_enu_;
    msg.return_home_when_done = return_home_when_done_;
    msg.note = note_;

    msg.no_fly_zones.clear();
    // 允许空数组；也允许临时塞非法长度来“清空”（直接忽略）
    if (!no_fly_flat_.empty() && no_fly_flat_.size() % 4 == 0) {
      for (size_t i = 0; i + 3 < no_fly_flat_.size(); i += 4) {
        // 退化矩形（宽/高为 0）视为占位，不当禁飞区
        if (std::abs(no_fly_flat_[i + 2] - no_fly_flat_[i]) < 1e-6 &&
          std::abs(no_fly_flat_[i + 3] - no_fly_flat_[i + 1]) < 1e-6)
        {
          continue;
        }
        geometry_msgs::msg::Point mn;
        geometry_msgs::msg::Point mx;
        mn.x = no_fly_flat_[i];
        mn.y = no_fly_flat_[i + 1];
        mx.x = no_fly_flat_[i + 2];
        mx.y = no_fly_flat_[i + 3];
        msg.no_fly_zones.push_back(mn);
        msg.no_fly_zones.push_back(mx);
      }
    } else if (!no_fly_flat_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no_fly_zones 长度必须是 4 的倍数（xmin ymin xmax ymax），当前=%zu，已忽略",
        no_fly_flat_.size());
    }

    mission_pub_->publish(msg);
  }

  rclcpp::Publisher<mine_nav::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string mode_;
  uint8_t priority_{50};
  float fly_alt_enu_{1.2f};
  bool return_home_when_done_{true};
  std::string note_;
  double area_min_x_{-10.0};
  double area_min_y_{-10.0};
  double area_max_x_{10.0};
  double area_max_y_{10.0};
  bool has_goal_{false};
  double goal_x_{0.0};
  double goal_y_{0.0};
  std::vector<double> no_fly_flat_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManagerNode>());
  rclcpp::shutdown();
  return 0;
}
