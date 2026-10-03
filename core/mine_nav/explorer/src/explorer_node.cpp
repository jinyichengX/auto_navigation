#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mine_nav/common.hpp"
#include "mine_nav/msg/local_status.hpp"
#include "mine_nav/msg/mission.hpp"

using namespace std::chrono_literals;

/**
 * 探索器：大脑。
 * 读任务层 Mission + 地图 + 局部状态，决定“下一个目标点”。
 * 不做全图搜路，也不直接控飞。
 */
class ExplorerNode : public rclcpp::Node
{
public:
  ExplorerNode()
  : Node("explorer")
  {
    frontier_unknown_need_ = declare_parameter<int>("frontier_unknown_need", 1);
    sample_step_ = declare_parameter<double>("sample_step", 1.0);
    goal_hold_sec_ = declare_parameter<double>("goal_hold_sec", 1.5);
    replan_period_ = declare_parameter<double>("replan_period", 1.0);
    home_tol_ = declare_parameter<double>("home_tol", 0.8);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.40);
    // 探索落点按硬半径检查，避免目标贴柱后全局搜不通
    goal_radius_ = declare_parameter<double>("goal_radius", 0.40);
    nearest_search_radius_ = declare_parameter<double>("nearest_search_radius", 3.0);
    nearest_search_step_ = declare_parameter<double>("nearest_search_step", 0.25);
    prefer_mid_dist_ = declare_parameter<double>("prefer_mid_dist", 4.0);
    visited_clear_radius_ = declare_parameter<double>("visited_clear_radius", 1.2);
    visited_max_ = declare_parameter<int>("visited_max", 60);
    info_gain_weight_ = declare_parameter<double>("info_gain_weight", 1.0);
    dist_weight_ = declare_parameter<double>("dist_weight", 0.12);
    stuck_switch_cooldown_ = declare_parameter<double>("stuck_switch_cooldown", 8.0);
    // 前沿目标往已知区回退一点，避免局部在未知边上空绕
    frontier_pullback_ = declare_parameter<double>("frontier_pullback", 0.55);
    // 井下墙后常有未知；用射线看“开口朝未知”比邻域贴未知更稳
    frontier_ray_range_ = declare_parameter<double>("frontier_ray_range", 2.8);
    frontier_ray_angles_ = declare_parameter<int>("frontier_ray_angles", 12);
    // 目标突然非法后先冷却，避免每 200ms 狂换点把局部/EKF 拖崩
    invalid_goal_cooldown_ = declare_parameter<double>("invalid_goal_cooldown", 2.5);
    // 直连过长时连通判定太严，限制检查距离；更远点交给全局规划绕路
    reachable_line_max_ = declare_parameter<double>("reachable_line_max", 2.2);

    auto qos_rel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto qos_be_tl = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    mission_sub_ = create_subscription<mine_nav::msg::Mission>(
      "/mission", qos_rel,
      std::bind(&ExplorerNode::on_mission, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", qos_rel,
      std::bind(&ExplorerNode::on_map, this, std::placeholders::_1));
    status_sub_ = create_subscription<mine_nav::msg::LocalStatus>(
      "/local_controller/status", qos_rel,
      std::bind(&ExplorerNode::on_status, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos_be_tl,
      std::bind(&ExplorerNode::on_local_pos, this, std::placeholders::_1));

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/explore_goal", 10);
    timer_ = create_wall_timer(200ms, std::bind(&ExplorerNode::tick, this));

    RCLCPP_INFO(get_logger(), "探索器已启动：等 /mission，输出 /explore_goal");
  }

private:
  void on_mission(const mine_nav::msg::Mission::SharedPtr msg)
  {
    // 任务模式或目标变了，清掉旧目标，逼着重新决策
    if (have_mission_) {
      const bool mode_changed = msg->mode != mission_.mode;
      const bool goal_changed =
        msg->has_goal != mission_.has_goal ||
        (msg->has_goal &&
        mine_nav::hypot2(msg->goal.x - mission_.goal.x, msg->goal.y - mission_.goal.y) > 0.2);
      if (mode_changed || goal_changed) {
        have_goal_ = false;
        last_why_.clear();
        have_last_snap_ = false;
      }
    }
    mission_ = *msg;
    have_mission_ = true;
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_.update(*msg);
  }

  void on_status(const mine_nav::msg::LocalStatus::SharedPtr msg)
  {
    if (msg->status != local_status_) {
      // 刚进入 stuck：标记一下，tick 里只处理一次，避免每拍都清目标
      if (msg->status == "stuck") {
        stuck_edge_ = true;
      }
      local_status_ = msg->status;
    }
  }

  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    // NED -> map(ENU)
    auto xy = mine_nav::ned_to_map(msg->x, msg->y);
    cur_x_ = xy.first;
    cur_y_ = xy.second;
    have_pose_ = true;
  }

  bool in_mission_area(double x, double y) const
  {
    if (!have_mission_) {
      return false;
    }
    if (!mine_nav::point_in_rect(x, y, mission_.area_min, mission_.area_max)) {
      return false;
    }
    if (mine_nav::point_in_any_nofly(x, y, mission_.no_fly_zones)) {
      return false;
    }
    return true;
  }

  double stand_radius() const
  {
    // 探索落点按硬半径检查：太松会选到柱边，全局规划再膨胀就搜不通
    if (goal_radius_ > 1e-3) {
      return goal_radius_;
    }
    return std::max(0.30, robot_radius_);
  }

  bool line_free_known(
    double x0, double y0, double x1, double y1, double radius) const
  {
    if (!grid_.ready) {
      return false;
    }
    const double dist = mine_nav::hypot2(x1 - x0, y1 - y0);
    const double step = std::max(grid_.res * 0.8, 0.08);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const double x = x0 + (x1 - x0) * t;
      const double y = y0 + (y1 - y0) * t;
      if (grid_.is_occupied(x, y, radius)) {
        return false;
      }
      // 探索目标连线不允许穿未知：未知里可能是墙后口袋
      const int v = grid_.cell_value(x, y);
      if (v < 0) {
        return false;
      }
    }
    return true;
  }

  bool goal_allowed(double x, double y) const
  {
    if (!in_mission_area(x, y)) {
      return false;
    }
    // 没图时不允许瞎选目标；有图时必须可站，且自身最好是已知 free
    if (!grid_.ready) {
      return false;
    }
    if (grid_.is_occupied(x, y, stand_radius())) {
      return false;
    }
    const int self_v = grid_.cell_value(x, y);
    if (self_v < 0 || self_v > mine_nav::kOccThresh) {
      return false;
    }
    // 离任务边界太近的点先丢掉，常见是墙外投影噪点；别收太狠，否则巷道尽头全被砍
    const double edge_margin = std::max(0.45, stand_radius() + 0.05);
    if (x < mission_.area_min.x + edge_margin ||
      x > mission_.area_max.x - edge_margin ||
      y < mission_.area_min.y + edge_margin ||
      y > mission_.area_max.y - edge_margin)
    {
      return false;
    }
    return true;
  }

  bool goal_reachable(double x, double y) const
  {
    if (!goal_allowed(x, y) || !have_pose_) {
      return false;
    }
    const double dist = mine_nav::hypot2(x - cur_x_, y - cur_y_);
    // 只挡“近处穿墙口袋”；中远距离交给全局规划绕柱/绕墙
    if (dist > reachable_line_max_) {
      return true;
    }
    // 近处连通检查放宽半径，避免红柱膨胀把巷道直连误杀
    const double r = std::max(0.18, stand_radius() - 0.16);
    if (line_free_known(cur_x_, cur_y_, x, y, r)) {
      return true;
    }
    // 直连被挡时，允许终点附近再 snap 一次；仍要求近处已知 free 连通
    double sx = x;
    double sy = y;
    if (!snap_to_free(x, y, sx, sy)) {
      return false;
    }
    if (mine_nav::hypot2(sx - cur_x_, sy - cur_y_) > reachable_line_max_) {
      return true;
    }
    return line_free_known(cur_x_, cur_y_, sx, sy, std::max(0.16, r - 0.02));
  }

  bool snap_to_free(double in_x, double in_y, double & out_x, double & out_y) const
  {
    return mine_nav::find_nearest_free(
      in_x, in_y, nearest_search_radius_, nearest_search_step_,
      [this](double x, double y) {return goal_allowed(x, y);},
      out_x, out_y);
  }

  void remember_visited(double x, double y)
  {
    for (const auto & p : visited_) {
      if (mine_nav::hypot2(p.first - x, p.second - y) < 0.35) {
        return;
      }
    }
    visited_.emplace_back(x, y);
    if (static_cast<int>(visited_.size()) > visited_max_) {
      visited_.erase(
        visited_.begin(),
        visited_.begin() + (static_cast<int>(visited_.size()) - visited_max_));
    }
  }

  bool visited_near(double x, double y) const
  {
    for (const auto & p : visited_) {
      if (mine_nav::hypot2(p.first - x, p.second - y) < visited_clear_radius_) {
        return true;
      }
    }
    return false;
  }

  // 返回: 0=free/出界当未知前都没撞墙, 1=unknown, 2=occupied
  int ray_first_hit(
    double x, double y, double dx, double dy, double max_range,
    double & hit_dist) const
  {
    hit_dist = max_range;
    if (!grid_.ready || max_range < grid_.res) {
      return 0;
    }
    const double step = grid_.res;
    const int n = std::max(1, static_cast<int>(max_range / step));
    for (int i = 1; i <= n; ++i) {
      const double d = i * step;
      const int v = grid_.cell_value(x + dx * d, y + dy * d);
      if (v > mine_nav::kOccThresh) {
        hit_dist = d;
        return 2;
      }
      if (v < 0) {
        hit_dist = d;
        return 1;
      }
    }
    return 0;
  }

  int frontier_score(double x, double y) const
  {
    if (!grid_.ready) {
      return 0;
    }
    // 候选点本身必须可站；任意形状地图只看占用/未知开口，不假设巷道
    if (grid_.is_occupied(x, y, stand_radius())) {
      return 0;
    }
    // 自己最好落在已知 free 上，避免把未知内部点当前沿
    const int self_v = grid_.cell_value(x, y);
    if (self_v < 0 || self_v > mine_nav::kOccThresh) {
      return 0;
    }

    // 邻域统计：井下墙边 free 常贴很多 occ，不能再用 occ<=8 误杀
    const double step = grid_.res;
    int unk = 0;
    int freeish = 0;
    int occ = 0;
    for (int dy = -4; dy <= 4; ++dy) {
      for (int dx = -4; dx <= 4; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int v = grid_.cell_value(x + dx * step, y + dy * step);
        if (v < 0) {
          ++unk;
        } else if (v <= mine_nav::kOccThresh) {
          ++freeish;
        } else {
          ++occ;
        }
      }
    }

    // 井下关键：墙后未知很多，射线常先撞墙看不到 unknown。
    // 邻域贴 unknown 的 free 就算候选；射线开口只做加分，不再当硬门槛。
    const int n_ang = std::max(4, frontier_ray_angles_);
    int open_rays = 0;
    double best_open = frontier_ray_range_;
    for (int i = 0; i < n_ang; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / n_ang;
      const double dx = std::cos(ang);
      const double dy = std::sin(ang);
      double hit = 0.0;
      const int kind = ray_first_hit(x, y, dx, dy, frontier_ray_range_, hit);
      if (kind == 1) {
        ++open_rays;
        best_open = std::min(best_open, hit);
      }
    }

    const bool neigh_ok = unk >= frontier_unknown_need_ && freeish >= 1;
    if (!neigh_ok && open_rays <= 0) {
      return 0;
    }
    // 开口越近、开口越多，信息增益越高；贴墙过多降分，但别把贴未知点直接打成 0
    const int open_gain =
      open_rays * 8 +
      static_cast<int>(
        12.0 * (1.0 - best_open / std::max(0.2, frontier_ray_range_)));
    int clear_bonus = 0;
    if (!grid_.is_occupied(x, y, stand_radius() + 0.15)) {
      clear_bonus += 6;
    }
    if (!grid_.is_occupied(x, y, stand_radius() + 0.30)) {
      clear_bonus += 4;
    }
    return static_cast<int>(
      info_gain_weight_ *
      (open_gain + unk * 3 + freeish - std::min(occ, 12) + clear_bonus));
  }

  int unknown_near_count(double x, double y, int ring) const
  {
    if (!grid_.ready) {
      return 0;
    }
    const double step = grid_.res;
    int unk = 0;
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dx = -ring; dx <= ring; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (grid_.cell_value(x + dx * step, y + dy * step) < 0) {
          ++unk;
        }
      }
    }
    return unk;
  }

  // 统计任务区∩地图里还剩多少未知，以及贴未知的 free；用来禁止过早 done
  void count_mission_unknown(int & unk_cells, int & free_near_unk) const
  {
    unk_cells = 0;
    free_near_unk = 0;
    if (!have_mission_ || !grid_.ready) {
      return;
    }
    const double step = std::max(grid_.res, 0.2);
    const double xmin = std::max(mission_.area_min.x, grid_.origin_x);
    const double ymin = std::max(mission_.area_min.y, grid_.origin_y);
    const double xmax = std::min(
      mission_.area_max.x, grid_.origin_x + grid_.width * grid_.res);
    const double ymax = std::min(
      mission_.area_max.y, grid_.origin_y + grid_.height * grid_.res);
    for (double y = ymin + 0.5 * grid_.res; y <= ymax + 1e-6; y += step) {
      for (double x = xmin + 0.5 * grid_.res; x <= xmax + 1e-6; x += step) {
        if (!in_mission_area(x, y)) {
          continue;
        }
        const int v = grid_.cell_value(x, y);
        if (v < 0) {
          ++unk_cells;
          continue;
        }
        if (v > mine_nav::kOccThresh) {
          continue;
        }
        if (unknown_near_count(x, y, 2) > 0 &&
          !grid_.is_occupied(x, y, std::max(0.25, stand_radius() - 0.10)))
        {
          ++free_near_unk;
        }
      }
    }
  }

  bool finalize_goal(double best_x, double best_y, double & out_x, double & out_y) const
  {
    if (!goal_allowed(best_x, best_y)) {
      return false;
    }
    // 把目标从未知边往当前位置回退一点，落在更稳的已知 free 上
    double gx = best_x;
    double gy = best_y;
    const double dx = cur_x_ - best_x;
    const double dy = cur_y_ - best_y;
    const double d = mine_nav::hypot2(dx, dy);
    if (d > 1e-3 && frontier_pullback_ > 1e-3) {
      const double pull = std::min(frontier_pullback_, d * 0.45);
      const double px = best_x + dx / d * pull;
      const double py = best_y + dy / d * pull;
      if (goal_allowed(px, py)) {
        gx = px;
        gy = py;
      }
    }
    // 回退后仍要能从当前位置连通；否则再 snap 一次
    if (!goal_reachable(gx, gy)) {
      double sx = gx;
      double sy = gy;
      if (!snap_to_free(gx, gy, sx, sy) || !goal_reachable(sx, sy)) {
        return false;
      }
      gx = sx;
      gy = sy;
    }
    out_x = gx;
    out_y = gy;
    return true;
  }

  bool pick_frontier(double & out_x, double & out_y) const
  {
    if (!have_mission_ || !have_pose_ || !grid_.ready) {
      return false;
    }
    // 采样范围 = 任务区 ∩ 当前地图，适配不规则/局部建图
    double xmin = std::max(mission_.area_min.x, grid_.origin_x);
    double ymin = std::max(mission_.area_min.y, grid_.origin_y);
    double xmax = std::min(
      mission_.area_max.x, grid_.origin_x + grid_.width * grid_.res);
    double ymax = std::min(
      mission_.area_max.y, grid_.origin_y + grid_.height * grid_.res);
    if (xmax < xmin || ymax < ymin) {
      return false;
    }
    // 步长对齐地图分辨率，避免刚好踩到墙格中心
    const double step = std::max(grid_.res, sample_step_);

    int best_score = 0;
    double best_x = cur_x_;
    double best_y = cur_y_;
    double best_cost = std::numeric_limits<double>::infinity();

    for (double y = ymin + 0.5 * grid_.res; y <= ymax + 1e-6; y += step) {
      for (double x = xmin + 0.5 * grid_.res; x <= xmax + 1e-6; x += step) {
        if (!goal_allowed(x, y) || visited_near(x, y)) {
          continue;
        }
        // 近处候选必须已知连通；远处交给全局规划，但仍禁止站在墙/未知上
        if (!goal_reachable(x, y)) {
          continue;
        }

        const int score = frontier_score(x, y);
        if (score <= 0) {
          continue;
        }
        const double dist = mine_nav::hypot2(x - cur_x_, y - cur_y_);
        // stuck 后偏好近处，避免反向猛换远目标直接撞墙
        const double prefer_dist =
          prefer_near_after_stuck_ ? std::min(2.0, prefer_mid_dist_ * 0.5) : prefer_mid_dist_;
        const double dist_pen =
          dist_weight_ * std::abs(dist - prefer_dist) +
          (prefer_near_after_stuck_ ? 0.35 * dist : 0.0);
        const double cost = -static_cast<double>(score) + dist_pen;
        if (score > best_score || (score == best_score && cost < best_cost)) {
          best_score = score;
          best_cost = cost;
          best_x = x;
          best_y = y;
        }
      }
    }

    if (best_score <= 0) {
      return false;
    }
    return finalize_goal(best_x, best_y, out_x, out_y);
  }

  // 找不到开口前沿时：朝最近未知推进，避免过早 explore_done
  bool pick_unknown_push(double & out_x, double & out_y) const
  {
    if (!have_mission_ || !have_pose_ || !grid_.ready) {
      return false;
    }
    double xmin = std::max(mission_.area_min.x, grid_.origin_x);
    double ymin = std::max(mission_.area_min.y, grid_.origin_y);
    double xmax = std::min(
      mission_.area_max.x, grid_.origin_x + grid_.width * grid_.res);
    double ymax = std::min(
      mission_.area_max.y, grid_.origin_y + grid_.height * grid_.res);
    if (xmax < xmin || ymax < ymin) {
      return false;
    }
    const double step = std::max(grid_.res, sample_step_);
    const int n_ang = std::max(4, frontier_ray_angles_);

    double best_x = cur_x_;
    double best_y = cur_y_;
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;

    for (double y = ymin + 0.5 * grid_.res; y <= ymax + 1e-6; y += step) {
      for (double x = xmin + 0.5 * grid_.res; x <= xmax + 1e-6; x += step) {
        if (!goal_allowed(x, y) || visited_near(x, y) || !goal_reachable(x, y)) {
          continue;
        }
        int open_rays = 0;
        double nearest_unk = frontier_ray_range_;
        for (int i = 0; i < n_ang; ++i) {
          const double ang = 2.0 * M_PI * static_cast<double>(i) / n_ang;
          double hit = 0.0;
          const int kind = ray_first_hit(
            x, y, std::cos(ang), std::sin(ang), frontier_ray_range_, hit);
          if (kind == 1) {
            ++open_rays;
            nearest_unk = std::min(nearest_unk, hit);
          }
        }
        if (open_rays <= 0) {
          continue;
        }
        const double dist = mine_nav::hypot2(x - cur_x_, y - cur_y_);
        // stuck 后优先近处开口，避免大掉头撞墙
        const double prefer_dist =
          prefer_near_after_stuck_ ? std::min(2.0, prefer_mid_dist_ * 0.5) : prefer_mid_dist_;
        const double cost =
          nearest_unk * 3.0 +
          0.15 * std::abs(dist - prefer_dist) +
          (prefer_near_after_stuck_ ? 0.40 * dist : 0.0) -
          0.8 * open_rays;
        if (!found || cost < best_cost) {
          found = true;
          best_cost = cost;
          best_x = x;
          best_y = y;
        }
      }
    }
    if (!found) {
      return false;
    }
    return finalize_goal(best_x, best_y, out_x, out_y);
  }

  // 兜底1：不看射线开口，只要 free 邻域贴未知就推进
  bool pick_free_near_unknown(double & out_x, double & out_y) const
  {
    if (!have_mission_ || !have_pose_ || !grid_.ready) {
      return false;
    }
    double xmin = std::max(mission_.area_min.x, grid_.origin_x);
    double ymin = std::max(mission_.area_min.y, grid_.origin_y);
    double xmax = std::min(
      mission_.area_max.x, grid_.origin_x + grid_.width * grid_.res);
    double ymax = std::min(
      mission_.area_max.y, grid_.origin_y + grid_.height * grid_.res);
    if (xmax < xmin || ymax < ymin) {
      return false;
    }
    const double step = std::max(grid_.res, std::min(sample_step_, 0.5));
    double best_x = cur_x_;
    double best_y = cur_y_;
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;

    for (double y = ymin + 0.5 * grid_.res; y <= ymax + 1e-6; y += step) {
      for (double x = xmin + 0.5 * grid_.res; x <= xmax + 1e-6; x += step) {
        if (!goal_allowed(x, y) || visited_near(x, y)) {
          continue;
        }
        // 远处交给全局；近处仍要求不穿墙口袋
        if (!goal_reachable(x, y)) {
          continue;
        }
        const int unk = unknown_near_count(x, y, 3);
        if (unk <= 0) {
          continue;
        }
        const double dist = mine_nav::hypot2(x - cur_x_, y - cur_y_);
        const double prefer_dist =
          prefer_near_after_stuck_ ? std::min(2.0, prefer_mid_dist_ * 0.5) : prefer_mid_dist_;
        const double cost =
          -3.0 * unk +
          0.20 * std::abs(dist - prefer_dist) +
          (prefer_near_after_stuck_ ? 0.45 * dist : 0.05 * dist);
        if (!found || cost < best_cost) {
          found = true;
          best_cost = cost;
          best_x = x;
          best_y = y;
        }
      }
    }
    if (!found) {
      return false;
    }
    return finalize_goal(best_x, best_y, out_x, out_y);
  }

  // 兜底2：找任务区最近未知格，沿方向落在已知 free 上往前推
  bool pick_toward_unknown(double & out_x, double & out_y) const
  {
    if (!have_mission_ || !have_pose_ || !grid_.ready) {
      return false;
    }
    double xmin = std::max(mission_.area_min.x, grid_.origin_x);
    double ymin = std::max(mission_.area_min.y, grid_.origin_y);
    double xmax = std::min(
      mission_.area_max.x, grid_.origin_x + grid_.width * grid_.res);
    double ymax = std::min(
      mission_.area_max.y, grid_.origin_y + grid_.height * grid_.res);
    if (xmax < xmin || ymax < ymin) {
      return false;
    }

    // 粗扫最近未知；步长别太细，避免每拍卡死
    const double scan_step = std::max(grid_.res * 2.0, 0.4);
    double unk_x = cur_x_;
    double unk_y = cur_y_;
    double best_unk_dist = std::numeric_limits<double>::infinity();
    bool found_unk = false;
    for (double y = ymin + 0.5 * grid_.res; y <= ymax + 1e-6; y += scan_step) {
      for (double x = xmin + 0.5 * grid_.res; x <= xmax + 1e-6; x += scan_step) {
        if (!in_mission_area(x, y)) {
          continue;
        }
        if (grid_.cell_value(x, y) >= 0) {
          continue;
        }
        const double d = mine_nav::hypot2(x - cur_x_, y - cur_y_);
        if (d < 0.4 || d > 12.0) {
          continue;
        }
        if (d < best_unk_dist) {
          best_unk_dist = d;
          unk_x = x;
          unk_y = y;
          found_unk = true;
        }
      }
    }
    if (!found_unk) {
      return false;
    }

    const double dx = unk_x - cur_x_;
    const double dy = unk_y - cur_y_;
    const double dist = mine_nav::hypot2(dx, dy);
    if (dist < 1e-3) {
      return false;
    }
    const double ux = dx / dist;
    const double uy = dy / dist;

    // 沿未知方向挑几个候选落点；优先中近处，避免一脚踹到墙后
    const double candidates[] = {1.2, 1.8, 2.4, 3.0, 3.6};
    double best_x = cur_x_;
    double best_y = cur_y_;
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;
    for (double push : candidates) {
      if (push >= dist) {
        break;
      }
      const double cx = cur_x_ + ux * push;
      const double cy = cur_y_ + uy * push;
      double sx = cx;
      double sy = cy;
      if (!goal_allowed(cx, cy)) {
        if (!snap_to_free(cx, cy, sx, sy)) {
          continue;
        }
      }
      if (visited_near(sx, sy) || !goal_reachable(sx, sy)) {
        continue;
      }
      const double d = mine_nav::hypot2(sx - cur_x_, sy - cur_y_);
      const double cost = std::abs(d - 2.0) + 0.15 * d;
      if (!found || cost < best_cost) {
        found = true;
        best_cost = cost;
        best_x = sx;
        best_y = sy;
      }
    }
    if (!found) {
      return false;
    }
    return finalize_goal(best_x, best_y, out_x, out_y);
  }

  void publish_goal(double x, double y, const std::string & why)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "camera_init";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = have_mission_ ? mission_.fly_alt_enu : 1.2;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);

    const bool moved =
      !have_goal_ || mine_nav::hypot2(x - goal_x_, y - goal_y_) > 0.3;
    if (moved || why != last_why_) {
      RCLCPP_INFO(
        get_logger(), "探索目标 -> (%.2f, %.2f) | %s | local=%s",
        x, y, why.c_str(), local_status_.c_str());
      last_why_ = why;
    }
    goal_x_ = x;
    goal_y_ = y;
    have_goal_ = true;
    // 只有目标真变了才刷新计时；否则 arrived_hold / stuck 冷却会被每拍刷掉
    if (moved) {
      last_goal_time_ = now();
      // 真正换到新点后，取消“近处偏好”
      if (prefer_near_after_stuck_ && why.find("stuck_hold") == std::string::npos) {
        prefer_near_after_stuck_ = false;
      }
    }
  }

  void handle_goto()
  {
    const double gx = mission_.goal.x;
    const double gy = mission_.goal.y;
    if (goal_allowed(gx, gy)) {
      have_last_snap_ = false;
      publish_goal(gx, gy, "goto");
      return;
    }

    // 旧 snap 还合法就锁定，避免等距多解时目标左右抖
    if (have_last_snap_ &&
      mine_nav::hypot2(gx - last_raw_gx_, gy - last_raw_gy_) <= 0.05 &&
      goal_allowed(last_snap_x_, last_snap_y_))
    {
      publish_goal(last_snap_x_, last_snap_y_, "goto_snapped");
      return;
    }

    // 目标本身不合法：附近找最近可走点，而不是直接回原点
    double sx = 0.0;
    double sy = 0.0;
    if (snap_to_free(gx, gy, sx, sy)) {
      const bool snap_changed =
        !have_last_snap_ ||
        mine_nav::hypot2(sx - last_snap_x_, sy - last_snap_y_) > 0.2 ||
        mine_nav::hypot2(gx - last_raw_gx_, gy - last_raw_gy_) > 0.05;
      if (snap_changed) {
        RCLCPP_WARN(
          get_logger(),
          "任务目标(%.2f,%.2f)不可达，改用附近可走点(%.2f,%.2f)",
          gx, gy, sx, sy);
      }
      last_snap_x_ = sx;
      last_snap_y_ = sy;
      last_raw_gx_ = gx;
      last_raw_gy_ = gy;
      have_last_snap_ = true;
      publish_goal(sx, sy, "goto_snapped");
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "任务目标(%.2f,%.2f)附近找不到可走点，原地待命", gx, gy);
    publish_goal(cur_x_, cur_y_, "goto_invalid_hold");
  }

  void tick()
  {
    if (!have_mission_ || !have_pose_) {
      return;
    }

    // 卡住：短时稳住；持续 stuck / 长时间假 avoiding 都按冷却换近处目标
    // 假净空时局部会一直报 avoiding 不进 stuck，所以 avoiding 超时也当卡住
    // 短时正常绕障仍走后面的 hold_goal，不在这里抢换点
    if (local_status_ == "avoiding") {
      if (!avoiding_since_valid_) {
        avoiding_since_ = now();
        avoiding_since_valid_ = true;
      }
    } else {
      avoiding_since_valid_ = false;
    }
    const double avoiding_for =
      avoiding_since_valid_ ? (now() - avoiding_since_).seconds() : 0.0;
    const bool avoiding_stuck =
      local_status_ == "avoiding" &&
      avoiding_for >= std::max(12.0, 1.5 * stuck_switch_cooldown_);
    const bool jammed_now =
      stuck_edge_ ||
      local_status_ == "stuck" ||
      avoiding_stuck;
    if (jammed_now) {
      stuck_edge_ = false;
      if (!stuck_since_valid_) {
        stuck_since_ = now();
        stuck_since_valid_ = true;
      }
      const double since_switch = (now() - last_stuck_switch_time_).seconds();
      const double stuck_for = (now() - stuck_since_).seconds();
      const double need_hold =
        avoiding_stuck ?
        std::max(12.0, 1.5 * stuck_switch_cooldown_) :
        stuck_switch_cooldown_;
      const bool force_switch = stuck_for >= std::max(10.0, 2.0 * stuck_switch_cooldown_);
      if (since_switch < stuck_switch_cooldown_ ||
        (!force_switch && stuck_for < need_hold))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "局部%s %.1fs，先稳住当前目标让局部绕（距上次换目标 %.1fs）",
          local_status_.c_str(),
          avoiding_stuck ? avoiding_for : stuck_for,
          since_switch);
        if (have_goal_ && goal_allowed(goal_x_, goal_y_)) {
          publish_goal(goal_x_, goal_y_, "explore_stuck_hold");
          return;
        }
      } else if (mission_.mode == "explore") {
        if (have_goal_) {
          remember_visited(goal_x_, goal_y_);
          RCLCPP_WARN(
            get_logger(),
            "局部%s %.1fs，暂避当前目标(%.2f,%.2f)，换下一个近处前沿%s",
            local_status_.c_str(),
            avoiding_stuck ? avoiding_for : stuck_for,
            goal_x_, goal_y_,
            force_switch ? "（强制）" : "");
        }
        have_goal_ = false;
        last_stuck_switch_time_ = now();
        stuck_since_valid_ = false;
        avoiding_since_valid_ = false;
        // 标记：下一次选点偏好近处，避免 stuck 后反向大掉头
        prefer_near_after_stuck_ = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "局部%s，当前模式=%s，保持目标等任务层改指令",
          local_status_.c_str(), mission_.mode.c_str());
      }
    } else {
      stuck_since_valid_ = false;
    }

    const auto now_tp = now();
    const std::string mode = mission_.mode;

    // abort/idle：立刻悬停当前位置
    if (mode == "abort" || mode == "idle") {
      publish_goal(cur_x_, cur_y_, mode);
      return;
    }

    // goto / return_home：按周期刷，但目标本身不乱换（snap 已有锁定）
    if (mode == "goto" && mission_.has_goal) {
      if (!have_goal_ || (now_tp - last_replan_time_).seconds() >= replan_period_) {
        last_replan_time_ = now_tp;
        handle_goto();
      } else if (have_goal_) {
        publish_goal(goal_x_, goal_y_, last_why_);
      }
      return;
    }

    if (mode == "return_home") {
      if (!have_goal_ || (now_tp - last_replan_time_).seconds() >= replan_period_) {
        last_replan_time_ = now_tp;
        double hx = 0.0;
        double hy = 0.0;
        if (snap_to_free(0.0, 0.0, hx, hy)) {
          publish_goal(hx, hy, "return_home");
        } else {
          publish_goal(0.0, 0.0, "return_home");
        }
      } else if (have_goal_) {
        publish_goal(goal_x_, goal_y_, last_why_);
      }
      return;
    }

    // 到位后先稳住，再记访问并换下一前沿；别在 hold 窗口里重选导致左右抖
    if (have_goal_ && local_status_ == "arrived") {
      const double dt = (now_tp - last_goal_time_).seconds();
      if (dt < goal_hold_sec_) {
        publish_goal(goal_x_, goal_y_, last_why_.empty() ? "explore_arrived_hold" : last_why_);
        return;
      }
      remember_visited(goal_x_, goal_y_);
      // 顺手把当前位置也记一下，避免贴身另一前沿立刻被选中
      remember_visited(cur_x_, cur_y_);
      RCLCPP_INFO(
        get_logger(),
        "目标已到位并稳住 %.1fs，记访问(%.2f,%.2f)，换下一前沿",
        dt, goal_x_, goal_y_);
      have_goal_ = false;
    }

    // explore：飞行中 / 等路径 / 短时 stuck 时，只要当前目标还合法就稳住
    // 避免 waiting_path 时每秒换前沿，把候选一口气吃光
    const bool hold_goal =
      local_status_ == "tracking" ||
      local_status_ == "avoiding" ||
      local_status_ == "waiting_path" ||
      local_status_ == "stuck";
    // hold 阶段只要求落点仍合法；连通交给全局规划，避免红柱挡直连就弃目标
    if (have_goal_ && hold_goal && goal_allowed(goal_x_, goal_y_)) {
      publish_goal(goal_x_, goal_y_, last_why_.empty() ? "explore_hold" : last_why_);
      return;
    }

    // 当前目标落点已非法：丢掉并冷却；冷却期不再把当前位置当目标发，避免回原点吃 visited
    if (have_goal_ && !goal_allowed(goal_x_, goal_y_)) {
      RCLCPP_WARN(
        get_logger(),
        "当前探索目标(%.2f,%.2f)已非法，冷却 %.1fs 后再选",
        goal_x_, goal_y_, invalid_goal_cooldown_);
      remember_visited(goal_x_, goal_y_);
      have_goal_ = false;
      last_invalid_goal_time_ = now_tp;
      last_why_ = "explore_invalid_hold";
      return;
    }

    if (!have_goal_ &&
      (now_tp - last_invalid_goal_time_).seconds() < invalid_goal_cooldown_)
    {
      // 冷却期静默悬停：不刷假目标
      return;
    }

    const bool need_replan =
      !have_goal_ ||
      (now_tp - last_replan_time_).seconds() >= replan_period_;
    if (!need_replan) {
      if (have_goal_) {
        publish_goal(goal_x_, goal_y_, last_why_);
      }
      return;
    }
    last_replan_time_ = now_tp;

    // 地图还没来：原地等，别误报 explore_done
    if (!grid_.ready) {
      publish_goal(cur_x_, cur_y_, "explore_wait_map");
      return;
    }

    // explore：开口前沿 → 未知推进 → 贴未知 free → 朝最近未知；仍没有再看未知量
    double fx = 0.0;
    double fy = 0.0;
    auto try_pick = [this, &fx, &fy](const char * why) -> bool {
      if (pick_frontier(fx, fy)) {
        publish_goal(fx, fy, why);
        return true;
      }
      if (pick_unknown_push(fx, fy)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "严格前沿为空，改向未知推进 (%.2f,%.2f) visited=%zu",
          fx, fy, visited_.size());
        publish_goal(fx, fy, "explore_unknown_push");
        return true;
      }
      if (pick_free_near_unknown(fx, fy)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "改用贴未知 free (%.2f,%.2f) visited=%zu",
          fx, fy, visited_.size());
        publish_goal(fx, fy, "explore_free_near_unknown");
        return true;
      }
      if (pick_toward_unknown(fx, fy)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "改朝最近未知推进 (%.2f,%.2f) visited=%zu",
          fx, fy, visited_.size());
        publish_goal(fx, fy, "explore_toward_unknown");
        return true;
      }
      return false;
    };

    if (try_pick("explore_frontier")) {
      return;
    }

    int unk_cells = 0;
    int free_near_unk = 0;
    count_mission_unknown(unk_cells, free_near_unk);

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "未找到前沿：map=%dx%d pose=(%.2f,%.2f) visited=%zu unk=%d free_near_unk=%d local=%s",
      grid_.width, grid_.height, cur_x_, cur_y_,
      visited_.size(), unk_cells, free_near_unk, local_status_.c_str());

    // 真正 done 前，清掉过期 visited 再试一次，避免 done 循环把点吃光
    if (!visited_.empty()) {
      visited_.clear();
      if (try_pick("explore_retry_after_clear")) {
        return;
      }
      count_mission_unknown(unk_cells, free_near_unk);
    }

    // 地图还大量未知时禁止 done 回原点：原地等下一拍再选，别把探索提前掐死
    if (unk_cells >= 200 || free_near_unk >= 8) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "任务区仍有未知(unk=%d free_near_unk=%d)，禁止 explore_done，原地等下一拍",
        unk_cells, free_near_unk);
      publish_goal(cur_x_, cur_y_, "explore_wait_unknown");
      return;
    }

    if (mission_.return_home_when_done) {
      double hx = 0.0;
      double hy = 0.0;
      snap_to_free(0.0, 0.0, hx, hy);
      if (mine_nav::hypot2(cur_x_, cur_y_) > home_tol_) {
        publish_goal(hx, hy, "explore_done_return_home");
      } else {
        publish_goal(hx, hy, "explore_done_hold_home");
      }
    } else {
      publish_goal(cur_x_, cur_y_, "explore_done_hold");
    }
  }

  rclcpp::Subscription<mine_nav::msg::Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<mine_nav::msg::LocalStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mine_nav::GridMap grid_;
  mine_nav::msg::Mission mission_;
  bool have_mission_{false};
  bool have_pose_{false};
  bool have_goal_{false};
  double cur_x_{0.0};
  double cur_y_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  std::string local_status_{"idle"};
  std::string last_why_;
  std::vector<std::pair<double, double>> visited_;
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  bool stuck_edge_{false};
  bool stuck_since_valid_{false};
  bool avoiding_since_valid_{false};
  bool prefer_near_after_stuck_{false};
  bool have_last_snap_{false};
  double last_snap_x_{0.0};
  double last_snap_y_{0.0};
  double last_raw_gx_{0.0};
  double last_raw_gy_{0.0};
  rclcpp::Time last_stuck_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time avoiding_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_invalid_goal_time_{0, 0, RCL_ROS_TIME};

  int frontier_unknown_need_{1};
  double sample_step_{1.0};
  double goal_hold_sec_{1.5};
  double replan_period_{1.0};
  double home_tol_{0.8};
  double robot_radius_{0.55};
  double goal_radius_{0.32};
  double nearest_search_radius_{3.0};
  double nearest_search_step_{0.25};
  double prefer_mid_dist_{4.0};
  double visited_clear_radius_{1.2};
  int visited_max_{60};
  double info_gain_weight_{1.0};
  double dist_weight_{0.12};
  double stuck_switch_cooldown_{5.0};
  double frontier_pullback_{0.55};
  double frontier_ray_range_{2.8};
  int frontier_ray_angles_{12};
  double invalid_goal_cooldown_{2.5};
  double reachable_line_max_{6.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
