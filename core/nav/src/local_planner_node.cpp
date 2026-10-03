#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * 轻量 2D 局部规划：
 * 订 /global_path + /inflated_map + /Odometry，
 * 沿全局路径取前视点；直线通就直通，不通就近处角度采样绕行，
 * 输出短路径 /local_path 给 path_tracker。
 * 定高平面，不绑巷道。
 */
class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("local_planner")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/inflated_map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/global_path");
    local_path_topic_ = declare_parameter<std::string>("local_path_topic", "/local_path");
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    lookahead_ = declare_parameter<double>("lookahead", 1.6);
    local_horizon_ = declare_parameter<double>("local_horizon", 2.5);
    sample_radius_ = declare_parameter<double>("sample_radius", 1.8);
    sample_angles_ = declare_parameter<int>("sample_angles", 32);
    sample_rings_ = declare_parameter<int>("sample_rings", 4);
    path_step_ = declare_parameter<double>("path_step", 0.25);
    replan_period_ = declare_parameter<double>("replan_period", 0.20);
    arrive_dist_ = declare_parameter<double>("arrive_dist", 0.40);
    escape_radius_ = declare_parameter<double>("escape_radius", 1.5);
    occ_threshold_ = declare_parameter<int>("occ_threshold", 50);
    treat_unknown_as_occ_ = declare_parameter<bool>("treat_unknown_as_occ", true);
    progress_weight_ = declare_parameter<double>("progress_weight", 1.0);
    detour_weight_ = declare_parameter<double>("detour_weight", 0.45);
    clear_bonus_ = declare_parameter<double>("clear_bonus", 0.35);

    auto qos_map = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto qos_rel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/local_planner/status", 10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, qos_map,
      std::bind(&LocalPlannerNode::on_map, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalPlannerNode::on_odom, this, std::placeholders::_1));
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, qos_rel,
      std::bind(&LocalPlannerNode::on_global_path, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(replan_period_),
      std::bind(&LocalPlannerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "local_planner: map=%s odom=%s global=%s local=%s lookahead=%.2f",
      map_topic_.c_str(), odom_topic_.c_str(), global_path_topic_.c_str(),
      local_path_topic_.c_str(), lookahead_);
  }

private:
  struct Point2
  {
    double x{0.0};
    double y{0.0};
  };

  static double hypot2(double dx, double dy)
  {
    return std::sqrt(dx * dx + dy * dy);
  }

  static double dist(const Point2 & a, const Point2 & b)
  {
    return hypot2(a.x - b.x, a.y - b.y);
  }

  void set_status(const std::string & s)
  {
    if (s == status_) {
      return;
    }
    status_ = s;
    std_msgs::msg::String msg;
    msg.data = status_;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "局部规划状态 -> %s", status_.c_str());
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_.update(*msg);
    if (frame_id_.empty()) {
      active_frame_ = map_.frame_id;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    have_pose_ = true;
  }

  void on_global_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_.clear();
    global_path_.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      global_path_.push_back(Point2{ps.pose.position.x, ps.pose.position.y});
    }
    if (!active_frame_.empty() || frame_id_.empty()) {
      // keep
    }
    if (!msg->header.frame_id.empty() && frame_id_.empty()) {
      active_frame_ = msg->header.frame_id;
    }
    have_global_ = !global_path_.empty();
  }

  size_t nearest_index() const
  {
    size_t best = 0;
    double best_d = 1e18;
    for (size_t i = 0; i < global_path_.size(); ++i) {
      const double d = dist(pose_, global_path_[i]);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }

  Point2 lookahead_on_global() const
  {
    if (global_path_.empty()) {
      return pose_;
    }
    size_t i = nearest_index();
    double acc = 0.0;
    Point2 prev = global_path_[i];
    for (; i + 1 < global_path_.size(); ++i) {
      const Point2 & cur = global_path_[i + 1];
      acc += dist(prev, cur);
      prev = cur;
      if (acc >= lookahead_) {
        return cur;
      }
    }
    return global_path_.back();
  }

  bool free_point(double x, double y) const
  {
    return !map_.is_occ_world(x, y, occ_threshold_, treat_unknown_as_occ_);
  }

  bool free_segment(const Point2 & a, const Point2 & b) const
  {
    return map_.segment_free(a.x, a.y, b.x, b.y, occ_threshold_, treat_unknown_as_occ_);
  }

  // 简单净空：四周短半径采样，free 比例越高越好
  double clearance_score(double x, double y) const
  {
    const double r = std::max(map_.resolution * 2.0, 0.20);
    int free_n = 0;
    int total = 0;
    for (int k = 0; k < 8; ++k) {
      const double ang = 2.0 * M_PI * k / 8.0;
      ++total;
      if (free_point(x + r * std::cos(ang), y + r * std::sin(ang))) {
        ++free_n;
      }
    }
    return static_cast<double>(free_n) / static_cast<double>(total);
  }

  nav_msgs::msg::Path make_path(const std::vector<Point2> & pts) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = active_frame_.empty() ? "camera_init" : active_frame_;
    path.poses.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i].x;
      ps.pose.position.y = pts[i].y;
      ps.pose.position.z = 0.0;
      if (i + 1 < pts.size()) {
        const double yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        ps.pose.orientation.w = std::cos(yaw * 0.5);
      } else if (i > 0) {
        const double yaw = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        ps.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        ps.pose.orientation.w = 1.0;
      }
      path.poses.push_back(ps);
    }
    return path;
  }

  std::vector<Point2> densify(const Point2 & a, const Point2 & b) const
  {
    std::vector<Point2> out;
    const double d = dist(a, b);
    const int n = std::max(1, static_cast<int>(std::ceil(d / path_step_)));
    out.reserve(static_cast<size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      out.push_back(Point2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
    return out;
  }

  std::vector<Point2> densify3(const Point2 & a, const Point2 & b, const Point2 & c) const
  {
    auto p1 = densify(a, b);
    auto p2 = densify(b, c);
    if (!p2.empty()) {
      p1.insert(p1.end(), p2.begin() + 1, p2.end());
    }
    return p1;
  }

  // 当前位置若已落入膨胀区，先找最近 free 点逃出，避免永远 blocked
  bool find_escape(Point2 & escape) const
  {
    if (free_point(pose_.x, pose_.y)) {
      escape = pose_;
      return true;
    }
    double best_d = std::numeric_limits<double>::infinity();
    bool found = false;
    const int rings = std::max(3, static_cast<int>(std::ceil(escape_radius_ / std::max(map_.resolution, 0.05))));
    for (int ring = 1; ring <= rings; ++ring) {
      const double radius = escape_radius_ * static_cast<double>(ring) / static_cast<double>(rings);
      for (int k = 0; k < sample_angles_; ++k) {
        const double ang = 2.0 * M_PI * k / sample_angles_;
        const Point2 cand{
          pose_.x + radius * std::cos(ang),
          pose_.y + radius * std::sin(ang)};
        if (!free_point(cand.x, cand.y)) {
          continue;
        }
        // 逃逸段允许穿过当前膨胀，只要求目标点 free
        const double d = dist(pose_, cand);
        if (d < best_d) {
          best_d = d;
          escape = cand;
          found = true;
        }
      }
      if (found) {
        return true;
      }
    }
    return false;
  }

  bool sample_via(
    const Point2 & from, const Point2 & look, Point2 & best_mid, bool require_from_free) const
  {
    const double base_ang = std::atan2(look.y - from.y, look.x - from.x);
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int ring = 1; ring <= sample_rings_; ++ring) {
      const double radius =
        sample_radius_ * static_cast<double>(ring) / static_cast<double>(sample_rings_);
      for (int k = 0; k < sample_angles_; ++k) {
        // 优先前向两侧，再扫整圈
        const double side = (k % 2 == 0) ? 1.0 : -1.0;
        const int step = (k + 2) / 2;
        const double ang = base_ang + side * (2.0 * M_PI * step / sample_angles_);
        const Point2 mid{
          from.x + radius * std::cos(ang),
          from.y + radius * std::sin(ang)};
        if (!free_point(mid.x, mid.y)) {
          continue;
        }
        // from 已在膨胀里时，不要求 from->mid 整段 free，只要求 mid 可走且 mid->look 通
        if (require_from_free && !free_segment(from, mid)) {
          continue;
        }
        if (!free_segment(mid, look)) {
          continue;
        }
        const double progress = dist(mid, look);
        const double detour = dist(from, mid) + dist(mid, look) - dist(from, look);
        const double clear = clearance_score(mid.x, mid.y);
        const double cost =
          progress_weight_ * progress +
          detour_weight_ * detour -
          clear_bonus_ * clear;
        if (cost < best_cost) {
          best_cost = cost;
          best_mid = mid;
          found = true;
        }
      }
    }
    return found;
  }

  bool plan_local(std::vector<Point2> & out_pts, std::string & mode)
  {
    out_pts.clear();
    if (!map_.ready || !have_pose_ || !have_global_) {
      return false;
    }

    const Point2 goal = global_path_.back();
    if (dist(pose_, goal) <= arrive_dist_) {
      out_pts = {pose_, goal};
      mode = "arrived";
      return true;
    }

    Point2 look = lookahead_on_global();
    // 局部地平线裁剪：别一下看太远
    const double look_d = dist(pose_, look);
    if (look_d > local_horizon_ && look_d > 1e-6) {
      const double s = local_horizon_ / look_d;
      look = Point2{
        pose_.x + (look.x - pose_.x) * s,
        pose_.y + (look.y - pose_.y) * s};
    }

    // 若前视点本身在障碍里，沿全局路径再往后找一个 free 前视点
    if (!free_point(look.x, look.y)) {
      size_t i = nearest_index();
      bool replaced = false;
      for (; i < global_path_.size(); ++i) {
        if (free_point(global_path_[i].x, global_path_[i].y) &&
          dist(pose_, global_path_[i]) >= 0.4)
        {
          look = global_path_[i];
          replaced = true;
          break;
        }
      }
      if (!replaced && free_point(goal.x, goal.y)) {
        look = goal;
      }
    }

    const bool pose_free = free_point(pose_.x, pose_.y);

    // 0) 自身已在膨胀区：先逃逸到最近 free，再继续规划
    if (!pose_free) {
      Point2 escape;
      if (find_escape(escape)) {
        Point2 mid;
        if (free_point(look.x, look.y) && free_segment(escape, look)) {
          out_pts = densify3(pose_, escape, look);
          mode = "escape";
          return true;
        }
        if (sample_via(escape, look, mid, true)) {
          auto p1 = densify(pose_, escape);
          auto p2 = densify3(escape, mid, look);
          if (!p2.empty()) {
            p1.insert(p1.end(), p2.begin() + 1, p2.end());
          }
          out_pts = std::move(p1);
          mode = "escape";
          return true;
        }
        // 至少先挪到 free，避免一直原地 blocked
        out_pts = densify(pose_, escape);
        mode = "escape";
        return true;
      }
    }

    // 1) 直线通：直接输出短路径
    if (pose_free && free_point(look.x, look.y) && free_segment(pose_, look)) {
      out_pts = densify(pose_, look);
      mode = "direct";
      return true;
    }

    // 2) 近处采样绕行：找 mid，使 pose->mid 与 mid->look 都通
    Point2 best_mid = pose_;
    if (sample_via(pose_, look, best_mid, pose_free)) {
      out_pts = densify3(pose_, best_mid, look);
      mode = "detour";
      return true;
    }

    // 3) 再试：不强制 mid->look，只找更靠近目标的 free 点推进一点
    {
      double best_prog = dist(pose_, look);
      Point2 best = pose_;
      bool found = false;
      const double base_ang = std::atan2(look.y - pose_.y, look.x - pose_.x);
      for (int ring = 1; ring <= sample_rings_; ++ring) {
        const double radius =
          sample_radius_ * static_cast<double>(ring) / static_cast<double>(sample_rings_);
        for (int k = 0; k < sample_angles_; ++k) {
          const double side = (k % 2 == 0) ? 1.0 : -1.0;
          const int step = (k + 2) / 2;
          const double ang = base_ang + side * (2.0 * M_PI * step / sample_angles_);
          const Point2 mid{
            pose_.x + radius * std::cos(ang),
            pose_.y + radius * std::sin(ang)};
          if (!free_point(mid.x, mid.y)) {
            continue;
          }
          if (pose_free && !free_segment(pose_, mid)) {
            continue;
          }
          const double prog = dist(mid, look);
          if (prog + 0.15 < best_prog) {
            best_prog = prog;
            best = mid;
            found = true;
          }
        }
      }
      if (found) {
        out_pts = densify(pose_, best);
        mode = "creep";
        return true;
      }
    }

    // 4) 绕不开：输出原地短路径让跟踪悬停
    out_pts = {pose_, pose_};
    mode = "blocked";
    return true;
  }

  void tick()
  {
    if (!frame_id_.empty()) {
      active_frame_ = frame_id_;
    }

    std::vector<Point2> pts;
    std::string mode;
    if (!plan_local(pts, mode)) {
      if (status_ != "waiting") {
        set_status("waiting");
      }
      return;
    }

    auto path = make_path(pts);
    local_path_pub_->publish(path);
    set_status(mode);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "local %s: pts=%zu pose=(%.2f,%.2f) look_end=(%.2f,%.2f)",
      mode.c_str(), pts.size(), pose_.x, pose_.y,
      pts.empty() ? 0.0 : pts.back().x,
      pts.empty() ? 0.0 : pts.back().y);
  }

  std::string map_topic_;
  std::string odom_topic_;
  std::string global_path_topic_;
  std::string local_path_topic_;
  std::string frame_id_;
  std::string active_frame_{"camera_init"};

  double lookahead_{1.6};
  double local_horizon_{2.5};
  double sample_radius_{1.8};
  int sample_angles_{32};
  int sample_rings_{4};
  double path_step_{0.25};
  double replan_period_{0.20};
  double arrive_dist_{0.40};
  double escape_radius_{1.5};
  int occ_threshold_{50};
  bool treat_unknown_as_occ_{true};
  double progress_weight_{1.0};
  double detour_weight_{0.45};
  double clear_bonus_{0.35};

  nav_pkg::GridMap map_;
  Point2 pose_;
  bool have_pose_{false};
  std::vector<Point2> global_path_;
  bool have_global_{false};
  std::string status_{"idle"};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
