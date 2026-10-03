#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mine_nav/common.hpp"
#include "mine_nav/msg/mission.hpp"

using namespace std::chrono_literals;

/**
 * 全局规划器：给定探索器目标点，在 /projected_map 上 A* 出一条路径。
 * 输出 /global_path 给局部控制器。
 */
class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner")
  {
    robot_radius_ = declare_parameter<double>("robot_radius", 0.35);
    path_step_ = declare_parameter<double>("path_step", 0.5);
    replan_period_ = declare_parameter<double>("replan_period", 1.0);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", true);
    unknown_cost_ = declare_parameter<double>("unknown_cost", 1.5);
    infl_extra_ = declare_parameter<double>("inflation_extra", 0.05);
    goal_snap_radius_ = declare_parameter<double>("goal_snap_radius", 2.0);
    start_snap_radius_ = declare_parameter<double>("start_snap_radius", 1.5);
    // 提高净空权重：路径更靠通道中线，而不是贴墙走
    clear_cost_gain_ = declare_parameter<double>("clear_cost_gain", 1.10);

    auto qos_be = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto qos_rel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/explore_goal", qos_rel,
      std::bind(&GlobalPlannerNode::on_goal, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", qos_rel,
      std::bind(&GlobalPlannerNode::on_map, this, std::placeholders::_1));
    mission_sub_ = create_subscription<mine_nav::msg::Mission>(
      "/mission", qos_rel,
      std::bind(&GlobalPlannerNode::on_mission, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos_be,
      std::bind(&GlobalPlannerNode::on_local_pos, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    timer_ = create_wall_timer(200ms, std::bind(&GlobalPlannerNode::tick, this));

    RCLCPP_INFO(get_logger(), "全局规划器已启动：/explore_goal -> A* -> /global_path");
  }

private:
  struct NodeKey
  {
    int x{0};
    int y{0};
    bool operator==(const NodeKey & o) const {return x == o.x && y == o.y;}
    bool operator<(const NodeKey & o) const
    {
      return x < o.x || (x == o.x && y < o.y);
    }
  };

  struct NodeKeyHash
  {
    size_t operator()(const NodeKey & k) const
    {
      return (static_cast<size_t>(k.x) << 32) ^ static_cast<size_t>(k.y);
    }
  };

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const bool changed =
      !have_goal_ ||
      mine_nav::hypot2(
        msg->pose.position.x - goal_.pose.position.x,
        msg->pose.position.y - goal_.pose.position.y) > 0.25;
    goal_ = *msg;
    have_goal_ = true;
    if (changed) {
      // 目标变了，下一拍强制重规划
      last_plan_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_.update(*msg);
  }

  void on_mission(const mine_nav::msg::Mission::SharedPtr msg)
  {
    mission_ = *msg;
    have_mission_ = true;
  }

  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    auto xy = mine_nav::ned_to_map(msg->x, msg->y);
    cur_x_ = xy.first;
    cur_y_ = xy.second;
    have_pose_ = true;
  }

  double plan_radius() const
  {
    return robot_radius_ + infl_extra_;
  }

  bool outside_or_nofly(double x, double y) const
  {
    if (!have_mission_) {
      return false;
    }
    if (!mine_nav::point_in_rect(x, y, mission_.area_min, mission_.area_max)) {
      return true;
    }
    if (mine_nav::point_in_any_nofly(x, y, mission_.no_fly_zones)) {
      return true;
    }
    return false;
  }

  bool blocked_world(double x, double y) const
  {
    if (outside_or_nofly(x, y)) {
      return true;
    }
    if (!grid_.ready) {
      return false;  // 没图时先放行直线兜底
    }
    if (grid_.is_occupied(x, y, plan_radius())) {
      return true;
    }
    if (!allow_unknown_ && grid_.is_unknown_near(x, y, plan_radius())) {
      return true;
    }
    return false;
  }

  bool line_clear(double x0, double y0, double x1, double y1) const
  {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = mine_nav::hypot2(dx, dy);
    const double step = std::max(0.15, grid_.ready ? grid_.res : 0.2);
    const int n = std::max(1, static_cast<int>(dist / step));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      if (blocked_world(x0 + dx * t, y0 + dy * t)) {
        return false;
      }
    }
    return true;
  }

  // 离障碍越近代价越高，任意地图都会更贴可行域中心，而不是贴墙走
  double clearance_penalty(double x, double y) const
  {
    if (!grid_.ready || clear_cost_gain_ <= 1e-6) {
      return 0.0;
    }
    // 双圈八向探测：近圈重罚、远圈轻罚，逼路径靠通道中线
    const double r_near = plan_radius() + 0.15;
    const double r_far = plan_radius() + 0.45;
    const double dirs[8][2] = {
      {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0},
      {0.707, 0.707}, {0.707, -0.707},
      {-0.707, 0.707}, {-0.707, -0.707}
    };
    double penalty = 0.0;
    for (const auto & d : dirs) {
      if (grid_.is_occupied(x + d[0] * r_near, y + d[1] * r_near, 0.05)) {
        penalty += 1.0;
      } else if (grid_.is_occupied(x + d[0] * r_far, y + d[1] * r_far, 0.05)) {
        penalty += 0.35;
      }
    }
    return clear_cost_gain_ * penalty;
  }

  double step_cost(double x, double y, bool diagonal) const
  {
    double c = diagonal ? 1.414 : 1.0;
    if (grid_.ready) {
      const int v = grid_.cell_value(x, y);
      if (v < 0) {
        c *= unknown_cost_;
      }
    }
    c += clearance_penalty(x, y);
    return c;
  }

  bool snap_idx(
    int ix, int iy, double search_m, int & out_x, int & out_y) const
  {
    auto idx_to_world = [&](int x, int y) {
        return std::make_pair(
          grid_.origin_x + (x + 0.5) * grid_.res,
          grid_.origin_y + (y + 0.5) * grid_.res);
      };
    auto wy0 = idx_to_world(ix, iy);
    if (!blocked_world(wy0.first, wy0.second)) {
      out_x = ix;
      out_y = iy;
      return true;
    }
    const int rings = std::max(1, static_cast<int>(std::ceil(search_m / grid_.res)));
    for (int r = 1; r <= rings; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {
            continue;
          }
          const int nx = ix + dx;
          const int ny = iy + dy;
          if (nx < 0 || ny < 0 || nx >= grid_.width || ny >= grid_.height) {
            continue;
          }
          auto wy = idx_to_world(nx, ny);
          if (!blocked_world(wy.first, wy.second)) {
            out_x = nx;
            out_y = ny;
            return true;
          }
        }
      }
    }
    return false;
  }

  // 简单折线简化：三点共线或中间点可直连就删
  void simplify_path(nav_msgs::msg::Path & path) const
  {
    if (path.poses.size() <= 2) {
      return;
    }
    std::vector<geometry_msgs::msg::PoseStamped> keep;
    keep.push_back(path.poses.front());
    size_t i = 0;
    while (i + 1 < path.poses.size()) {
      size_t j = i + 1;
      size_t best = j;
      while (j + 1 < path.poses.size()) {
        const auto & a = path.poses[i].pose.position;
        const auto & b = path.poses[j + 1].pose.position;
        // 直线采样检查是否被挡
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dist = mine_nav::hypot2(dx, dy);
        const double step = std::max(0.15, grid_.ready ? grid_.res : 0.2);
        const int n = std::max(1, static_cast<int>(dist / step));
        bool blocked = false;
        for (int k = 1; k < n; ++k) {
          const double t = static_cast<double>(k) / n;
          if (blocked_world(a.x + dx * t, a.y + dy * t)) {
            blocked = true;
            break;
          }
        }
        if (blocked) {
          break;
        }
        best = j + 1;
        ++j;
      }
      keep.push_back(path.poses[best]);
      i = best;
    }
    path.poses = std::move(keep);
  }

  bool plan_astar(nav_msgs::msg::Path & path_out)
  {
    path_out.poses.clear();
    path_out.header.stamp = now();
    path_out.header.frame_id = "camera_init";

    if (!have_pose_ || !have_goal_) {
      return false;
    }

    const double gz = goal_.pose.position.z;

    // 没地图：只有直线也干净时才兜底；穿禁飞区/范围外就别发
    if (!grid_.ready) {
      if (line_clear(cur_x_, cur_y_, goal_.pose.position.x, goal_.pose.position.y)) {
        path_out.poses.push_back(mine_nav::make_pose(path_out.header, cur_x_, cur_y_, gz));
        path_out.poses.push_back(
          mine_nav::make_pose(
            path_out.header, goal_.pose.position.x, goal_.pose.position.y, gz));
        path_out = mine_nav::densify_path(path_out, path_step_);
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "无地图且直线穿禁飞/越界，暂不发路径");
      return false;
    }

    int sx = 0;
    int sy = 0;
    int gx = 0;
    int gy = 0;
    if (!grid_.world_to_idx(cur_x_, cur_y_, sx, sy) ||
      !grid_.world_to_idx(goal_.pose.position.x, goal_.pose.position.y, gx, gy))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "起点/终点不在地图内");
      return false;
    }

    auto idx_to_world = [&](int ix, int iy) {
        const double x = grid_.origin_x + (ix + 0.5) * grid_.res;
        const double y = grid_.origin_y + (iy + 0.5) * grid_.res;
        return std::make_pair(x, y);
      };

    int ssx = sx;
    int ssy = sy;
    int ggx = gx;
    int ggy = gy;
    if (!snap_idx(sx, sy, start_snap_radius_, ssx, ssy)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "起点附近找不到可走格");
      return false;
    }
    if (!snap_idx(gx, gy, goal_snap_radius_, ggx, ggy)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "终点附近找不到可走格");
      return false;
    }
    sx = ssx;
    sy = ssy;
    gx = ggx;
    gy = ggy;

    using QItem = std::pair<double, NodeKey>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    std::unordered_map<NodeKey, double, NodeKeyHash> gscore;
    std::unordered_map<NodeKey, NodeKey, NodeKeyHash> parent;

    NodeKey start{sx, sy};
    NodeKey goal{gx, gy};
    open.push({0.0, start});
    gscore[start] = 0.0;

    auto h = [&](const NodeKey & a) {
        return mine_nav::hypot2(a.x - goal.x, a.y - goal.y);
      };

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    bool ok = false;
    int expand = 0;
    const int expand_limit = grid_.width * grid_.height;

    while (!open.empty() && expand < expand_limit) {
      const auto cur = open.top().second;
      open.pop();
      ++expand;
      if (cur == goal) {
        ok = true;
        break;
      }
      for (int i = 0; i < 8; ++i) {
        NodeKey nxt{cur.x + dx8[i], cur.y + dy8[i]};
        if (nxt.x < 0 || nxt.y < 0 || nxt.x >= grid_.width || nxt.y >= grid_.height) {
          continue;
        }
        // 对角移动时，角点两侧也得可走，避免穿墙角
        if (dx8[i] != 0 && dy8[i] != 0) {
          auto w1 = idx_to_world(cur.x + dx8[i], cur.y);
          auto w2 = idx_to_world(cur.x, cur.y + dy8[i]);
          if (blocked_world(w1.first, w1.second) || blocked_world(w2.first, w2.second)) {
            continue;
          }
        }
        auto wy = idx_to_world(nxt.x, nxt.y);
        if (blocked_world(wy.first, wy.second)) {
          continue;
        }
        const bool diag = (dx8[i] != 0 && dy8[i] != 0);
        const double ng = gscore[cur] + step_cost(wy.first, wy.second, diag);
        auto it = gscore.find(nxt);
        if (it == gscore.end() || ng < it->second) {
          gscore[nxt] = ng;
          parent[nxt] = cur;
          open.push({ng + h(nxt), nxt});
        }
      }
    }

    if (!ok) {
      // 只有直线真的可走才兜底；穿禁飞/穿墙就别硬发，避免局部跟着闯禁区
      if (line_clear(cur_x_, cur_y_, goal_.pose.position.x, goal_.pose.position.y)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "A* 失败，直线可走，发直线兜底");
        path_out.poses.push_back(mine_nav::make_pose(path_out.header, cur_x_, cur_y_, gz));
        path_out.poses.push_back(
          mine_nav::make_pose(
            path_out.header, goal_.pose.position.x, goal_.pose.position.y, gz));
        path_out = mine_nav::densify_path(path_out, path_step_);
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "A* 失败且直线也不可走（禁飞/障碍），不发穿障路径");
      return false;
    }

    std::vector<NodeKey> rev;
    for (NodeKey c = goal; ; c = parent[c]) {
      rev.push_back(c);
      if (c == start) {
        break;
      }
    }
    std::reverse(rev.begin(), rev.end());

    // 从真实当前位置起
    path_out.poses.push_back(mine_nav::make_pose(path_out.header, cur_x_, cur_y_, gz));
    for (const auto & k : rev) {
      auto wy = idx_to_world(k.x, k.y);
      path_out.poses.push_back(mine_nav::make_pose(path_out.header, wy.first, wy.second, gz));
    }
    // 保证终点精确（探索器给的目标，可能经过 snap）
    path_out.poses.push_back(
      mine_nav::make_pose(
        path_out.header, goal_.pose.position.x, goal_.pose.position.y, gz));

    simplify_path(path_out);
    path_out = mine_nav::densify_path(path_out, path_step_);
    return true;
  }

  void publish_empty_path(const rclcpp::Time & t)
  {
    nav_msgs::msg::Path empty;
    empty.header.stamp = t;
    empty.header.frame_id = "camera_init";
    last_path_ = empty;
    path_pub_->publish(last_path_);
  }

  void tick()
  {
    if (!have_goal_ || !have_pose_) {
      return;
    }
    const auto t = now();
    if ((t - last_plan_time_).seconds() < replan_period_ && !last_path_.poses.empty()) {
      last_path_.header.stamp = t;
      path_pub_->publish(last_path_);
      return;
    }
    nav_msgs::msg::Path path;
    if (!plan_astar(path) || path.poses.size() < 2) {
      // 目标不可达：清空旧路径，别让局部继续跟着旧任务飞
      if (!last_path_.poses.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "规划失败，清空旧路径，等待可达目标");
        publish_empty_path(t);
      }
      last_plan_time_ = t;
      return;
    }
    last_path_ = path;
    last_plan_time_ = t;
    path_pub_->publish(last_path_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "全局路径已更新：%zu 点 -> 目标(%.2f, %.2f)",
      last_path_.poses.size(), goal_.pose.position.x, goal_.pose.position.y);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<mine_nav::msg::Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mine_nav::GridMap grid_;
  mine_nav::msg::Mission mission_;
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Path last_path_;
  bool have_goal_{false};
  bool have_pose_{false};
  bool have_mission_{false};
  double cur_x_{0.0};
  double cur_y_{0.0};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  double robot_radius_{0.40};
  double path_step_{0.5};
  double replan_period_{1.0};
  bool allow_unknown_{true};
  double unknown_cost_{1.5};
  double infl_extra_{0.20};
  double goal_snap_radius_{2.0};
  double start_snap_radius_{1.5};
  double clear_cost_gain_{0.85};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
