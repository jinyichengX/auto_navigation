#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav/grid_map.hpp"
#include "nav/rrt.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * 通用 RRT 全局规划节点：
 * - 吃膨胀后的 OccupancyGrid
 * - 起点实时跟随飞机里程计（可切 TF / 外部起点话题）
 * - 目标：/goal_pose 或 GetPlan 服务
 * - 输出：nav_msgs/Path
 * 不绑定巷道形状、不绑定 mine_nav / PX4。
 */
class RRTPlannerNode : public rclcpp::Node
{
public:
  RRTPlannerNode()
  : Node("rrt_planner")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/inflated_map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");//订阅目标点
    path_topic_ = declare_parameter<std::string>("path_topic", "/global_path");//发布到目标点的路径
    frame_id_ = declare_parameter<std::string>("frame_id", "");  // 空=跟地图

    step_size_ = declare_parameter<double>("step_size", 0.40);
    goal_bias_ = declare_parameter<double>("goal_bias", 0.20);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.35);
    max_iterations_ = declare_parameter<int>("max_iterations", 4000);
    occ_threshold_ = declare_parameter<int>("occ_threshold", 50);
    treat_unknown_as_occ_ = declare_parameter<bool>("treat_unknown_as_occ", true);
    start_snap_radius_ = declare_parameter<double>("start_snap_radius", 1.0);
    goal_snap_radius_ = declare_parameter<double>("goal_snap_radius", 1.5);
    replan_period_ = declare_parameter<double>("replan_period", 0.5);
    replan_on_goal_only_ = declare_parameter<bool>("replan_on_goal_only", false);
    path_yaw_from_segment_ = declare_parameter<bool>("path_yaw_from_segment", true);

    apply_cfg();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&RRTPlannerNode::on_map, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RRTPlannerNode::on_odom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&RRTPlannerNode::on_goal, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    plan_srv_ = create_service<nav_msgs::srv::GetPlan>(
      "get_plan",
      std::bind(
        &RRTPlannerNode::on_get_plan, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, replan_period_)),
      std::bind(&RRTPlannerNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "rrt_planner: map=%s odom=%s goal=%s path=%s step=%.2f iters=%d",
      map_topic_.c_str(), odom_topic_.c_str(), goal_topic_.c_str(),
      path_topic_.c_str(), step_size_, max_iterations_);
  }

private:
  void apply_cfg()
  {
    nav_pkg::RRTConfig cfg;
    cfg.step_size = step_size_;
    cfg.goal_bias = goal_bias_;
    cfg.goal_tolerance = goal_tolerance_;
    cfg.max_iterations = max_iterations_;
    cfg.occ_threshold = occ_threshold_;
    cfg.treat_unknown_as_occ = treat_unknown_as_occ_;
    cfg.start_snap_radius = start_snap_radius_;
    cfg.goal_snap_radius = goal_snap_radius_;
    planner_.set_config(cfg);
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_.update(*msg);
    map_stamp_ = msg->header.stamp;
    if (frame_id_.empty()) {
      active_frame_ = msg->header.frame_id;
    } else {
      active_frame_ = frame_id_;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;
    have_start_ = true;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    have_goal_ = true;
    goal_dirty_ = true;
    RCLCPP_INFO(
      get_logger(), "收到目标 (%.2f, %.2f) frame=%s",
      goal_x_, goal_y_, msg->header.frame_id.c_str());
    try_plan(true);
  }

  void on_timer()
  {
    if (replan_on_goal_only_) {
      return;
    }
    try_plan(false);
  }

  void on_get_plan(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
  {
    const double sx = req->start.pose.position.x;
    const double sy = req->start.pose.position.y;
    const double gx = req->goal.pose.position.x;
    const double gy = req->goal.pose.position.y;
    auto path = build_path(sx, sy, gx, gy);
    res->plan = path;
    if (!path.poses.empty()) {
      RCLCPP_INFO(get_logger(), "GetPlan 成功，点数=%zu", path.poses.size());
    } else {
      RCLCPP_WARN(get_logger(), "GetPlan 失败");
    }
  }

  void try_plan(bool force)
  {
    if (!map_.ready || !have_start_ || !have_goal_) {
      return;
    }
    if (!force && !goal_dirty_ && replan_on_goal_only_) {
      return;
    }
    // 周期重规划：目标未变也允许，便于起点移动后刷新
    auto path = build_path(start_x_, start_y_, goal_x_, goal_y_);
    if (path.poses.empty()) {
      if ((now() - last_fail_log_).seconds() >= 1.0) {
        last_fail_log_ = now();
        RCLCPP_WARN(
          get_logger(),
          "规划失败 start=(%.2f,%.2f) goal=(%.2f,%.2f)",
          start_x_, start_y_, goal_x_, goal_y_);
      }
      return;
    }
    path_pub_->publish(path);
    goal_dirty_ = false;
    if ((now() - last_ok_log_).seconds() >= 1.0) {
      last_ok_log_ = now();
      RCLCPP_INFO(
        get_logger(),
        "已发布路径 points=%zu start=(%.2f,%.2f) goal=(%.2f,%.2f)",
        path.poses.size(), start_x_, start_y_, goal_x_, goal_y_);
    }
  }

  nav_msgs::msg::Path build_path(
    double sx, double sy, double gx, double gy)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = active_frame_.empty() ? map_.frame_id : active_frame_;

    const auto result = planner_.plan(map_, sx, sy, gx, gy);
    if (!result.ok || result.path.size() < 2) {
      RCLCPP_DEBUG(get_logger(), "RRT: %s", result.message.c_str());
      return path;
    }

    path.poses.reserve(result.path.size());
    for (size_t i = 0; i < result.path.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = result.path[i].first;
      ps.pose.position.y = result.path[i].second;
      ps.pose.position.z = 0.0;

      double yaw = 0.0;
      if (path_yaw_from_segment_ && i + 1 < result.path.size()) {
        yaw = std::atan2(
          result.path[i + 1].second - result.path[i].second,
          result.path[i + 1].first - result.path[i].first);
      } else if (path_yaw_from_segment_ && i > 0) {
        yaw = std::atan2(
          result.path[i].second - result.path[i - 1].second,
          result.path[i].first - result.path[i - 1].first);
      }
      ps.pose.orientation.z = std::sin(yaw * 0.5);
      ps.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(ps);
    }
    return path;
  }

  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string frame_id_;
  std::string active_frame_{"map"};

  double step_size_{0.40};
  double goal_bias_{0.20};
  double goal_tolerance_{0.35};
  int max_iterations_{4000};
  int occ_threshold_{50};
  bool treat_unknown_as_occ_{true};
  double start_snap_radius_{1.0};
  double goal_snap_radius_{1.5};
  double replan_period_{0.5};
  bool replan_on_goal_only_{false};
  bool path_yaw_from_segment_{true};

  nav_pkg::GridMap map_;
  nav_pkg::RRTPlanner planner_;
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};

  bool have_start_{false};
  bool have_goal_{false};
  bool goal_dirty_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_ok_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fail_log_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRTPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
