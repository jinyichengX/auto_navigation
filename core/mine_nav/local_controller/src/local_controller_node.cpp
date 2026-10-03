#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mine_nav/common.hpp"
#include "mine_nav/msg/local_status.hpp"

using namespace std::chrono_literals;

/**
 * 局部控制器：跟踪 /global_path + 任意地图近处绕障。
 * 不假设巷道轴对齐；障碍时在周围做短距方向采样，选通达的局部目标。
 */
class LocalControllerNode : public rclcpp::Node
{
public:
  LocalControllerNode()
  : Node("local_controller")
  {
    max_speed_ = declare_parameter<double>("max_speed", 0.30);
    avoid_speed_scale_ = declare_parameter<double>("avoid_speed_scale", 0.55);
    fly_alt_ned_ = declare_parameter<double>("fly_alt_ned", -1.2);
    // hard: 机体+桨叶硬碰撞半径；soft: 规划/避障膨胀半径（只用于净空，不用于“贴墙侧移”）
    robot_radius_ = declare_parameter<double>("robot_radius", 0.40);
    soft_radius_ = declare_parameter<double>("soft_radius", 0.55);
    arrive_dist_ = declare_parameter<double>("arrive_dist", 0.40);
    hold_sec_ = declare_parameter<double>("hold_sec", 1.0);
    lookahead_ = declare_parameter<double>("lookahead", 1.6);
    unknown_near_ = declare_parameter<double>("unknown_near", 1.8);
    // 离路径终点这么近时，未知不再当硬挡：探索前沿本来就贴未知
    final_approach_dist_ = declare_parameter<double>("final_approach_dist", 1.2);
    replan_period_ = declare_parameter<double>("replan_period", 0.35);
    stuck_timeout_sec_ = declare_parameter<double>("stuck_timeout_sec", 4.0);
    path_stale_sec_ = declare_parameter<double>("path_stale_sec", 3.0);
    sample_radius_ = declare_parameter<double>("sample_radius", 2.0);
    sample_rings_ = declare_parameter<int>("sample_rings", 4);
    sample_angles_ = declare_parameter<int>("sample_angles", 24);
    progress_weight_ = declare_parameter<double>("progress_weight", 1.0);
    detour_weight_ = declare_parameter<double>("detour_weight", 0.40);
    clear_bonus_ = declare_parameter<double>("clear_bonus", 0.55);
    // 新路径大掉头先悬停；高度骤降也先悬停，避免贴墙急转撞墙
    turn_hold_deg_ = declare_parameter<double>("turn_hold_deg", 100.0);
    alt_drop_limit_ = declare_parameter<double>("alt_drop_limit", 0.28);
    min_fly_alt_ = declare_parameter<double>("min_fly_alt", 0.70);
    // EKF 发散/位姿无效时，立刻停跟路径，避免继续推飞
    pose_abs_limit_ = declare_parameter<double>("pose_abs_limit", 80.0);
    pose_jump_limit_ = declare_parameter<double>("pose_jump_limit", 2.5);

    auto qos_be = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto qos_be_tl = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    auto qos_rel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_be);
    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos_be);
    sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos_be);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    status_pub_ = create_publisher<mine_nav::msg::LocalStatus>("/local_controller/status", 10);

    status_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos_be_tl,
      std::bind(&LocalControllerNode::on_ack, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos_be_tl,
      std::bind(&LocalControllerNode::on_local_pos, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", qos_rel,
      std::bind(&LocalControllerNode::on_map, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", qos_rel,
      std::bind(&LocalControllerNode::on_global_path, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&LocalControllerNode::tick, this));
    RCLCPP_INFO(get_logger(), "局部控制器已启动：跟踪 /global_path + 任意地图短距绕障");
  }

private:
  struct NedPoint
  {
    double n{0.0};
    double e{0.0};
    double d{0.0};
  };

  static double dist3(const NedPoint & a, const NedPoint & b)
  {
    return std::sqrt(
      (a.n - b.n) * (a.n - b.n) +
      (a.e - b.e) * (a.e - b.e) +
      (a.d - b.d) * (a.d - b.d));
  }

  static double dist2(const NedPoint & a, const NedPoint & b)
  {
    return mine_nav::hypot2(a.n - b.n, a.e - b.e);
  }

  void set_status(const std::string & s)
  {
    if (s == status_) {
      return;
    }
    status_ = s;
    publish_status();
    RCLCPP_INFO(get_logger(), "局部状态 -> %s", status_.c_str());
  }

  void publish_status()
  {
    mine_nav::msg::LocalStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_init";
    msg.status = status_;
    msg.path_index = static_cast<uint32_t>(wp_index_);
    msg.path_size = static_cast<uint32_t>(global_path_.size());
    msg.avoiding = avoid_active_;
    if (have_pose_ && goal_valid_) {
      msg.remain_m = static_cast<float>(dist3(pose_, goal_));
    }
    status_pub_->publish(msg);
  }

  void on_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "命令回执 -> command=%u result=%u",
      msg->command, msg->result);
  }

  void enter_pose_safe_hold(const std::string & why)
  {
    pose_safe_hold_ = true;
    goal_valid_ = false;
    avoid_active_ = false;
    global_path_.clear();
    // 已起飞就锁当前悬停；没起飞就别硬推路径
    if (phase_ == "fly" || phase_ == "wait_path" || phase_ == "done") {
      local_target_ = pose_;
      goal_ = pose_;
      sp_ = pose_;
      sp_vel_ = NedPoint{0, 0, 0};
      phase_ = "wait_path";
      set_status("pose_invalid");
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "位姿异常(%s)：停止跟路径，安全悬停 NED=(%.2f,%.2f,%.2f) xy_valid=%d",
      why.c_str(), pose_.n, pose_.e, pose_.d, pose_xy_valid_ ? 1 : 0);
  }

  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const NedPoint incoming{msg->x, msg->y, msg->z};
    pose_xy_valid_ = msg->xy_valid;
    pose_z_valid_ = msg->z_valid;
    // 假世界/DDS 幽灵节点偶发把位姿瞬间钉回原点；跳变过大就忽略这帧
    if (have_pose_) {
      const double jump = dist2(pose_, incoming);
      if (jump > pose_jump_limit_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "位姿跳变过大 %.2fm (%.2f,%.2f)->(%.2f,%.2f)，忽略本帧",
          jump, pose_.n, pose_.e, incoming.n, incoming.e);
        // 连续大跳变也当发散：停跟路径
        ++pose_bad_ticks_;
        if (pose_bad_ticks_ >= 10) {
          enter_pose_safe_hold("jump");
        }
        return;
      }
    }

    // EKF 报告水平位姿无效，或坐标漂到离谱范围：立刻安全悬停
    const bool abs_bad =
      std::fabs(incoming.n) > pose_abs_limit_ ||
      std::fabs(incoming.e) > pose_abs_limit_ ||
      std::fabs(incoming.d) > pose_abs_limit_;
    if (!msg->xy_valid || abs_bad) {
      ++pose_bad_ticks_;
      if (have_pose_ && pose_bad_ticks_ >= 3) {
        enter_pose_safe_hold(msg->xy_valid ? "abs_limit" : "xy_valid=false");
      }
      // 无效帧不更新跟踪位姿，避免把 setpoint 跟着漂走
      if (!have_pose_) {
        return;
      }
      return;
    }

    pose_bad_ticks_ = 0;
    if (pose_safe_hold_ && msg->xy_valid && !abs_bad) {
      // 位姿恢复后仍保持悬停等新路径，不自动猛冲旧目标
      pose_safe_hold_ = false;
      RCLCPP_WARN(
        get_logger(),
        "位姿已恢复有效，继续悬停等新 /global_path");
      set_status("waiting_path");
    }

    // 高度骤降 / 过低：立刻停推。宁悬停，不贴墙硬推。
    if (have_pose_ && phase_ == "fly" && !alt_drop_hold_) {
      const double drop = (-pose_.d) - (-incoming.d);  // 正值=掉高
      const double alt = -incoming.d;
      if ((drop >= alt_drop_limit_ && alt < (-fly_alt_ned_) * 0.85) ||
        alt < min_fly_alt_)
      {
        alt_drop_hold_ = true;
        local_target_ = pose_;
        sp_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        avoid_active_ = false;
        RCLCPP_ERROR(
          get_logger(),
          "高度异常 drop=%.2fm alt=%.2fm，停推防撞墙/EKF炸",
          drop, alt);
        set_status("stuck");
      }
    }

    pose_ = incoming;
    have_pose_ = true;
    if (!sp_init_) {
      sp_ = pose_;
      sp_init_ = true;
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_.update(*msg);
  }

  bool path_same(const std::vector<NedPoint> & pts) const
  {
    if (pts.size() != global_path_.size()) {
      return false;
    }
    for (size_t i = 0; i < pts.size(); ++i) {
      if (mine_nav::hypot2(pts[i].n - global_path_[i].n, pts[i].e - global_path_[i].e) > 0.15) {
        return false;
      }
    }
    return true;
  }

  size_t nearest_index() const
  {
    if (global_path_.empty() || !have_pose_) {
      return 0;
    }
    size_t best = 0;
    double best_d = 1e9;
    for (size_t i = 0; i < global_path_.size(); ++i) {
      const double d = dist2(global_path_[i], pose_);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    if (best + 1 < global_path_.size() && best_d < arrive_dist_) {
      return best + 1;
    }
    return best;
  }

  // 从当前位置沿路径向前，取约 lookahead 距离处的点作为跟踪目标
  NedPoint lookahead_point(double horizon, size_t & out_idx) const
  {
    out_idx = wp_index_;
    if (global_path_.empty()) {
      return pose_;
    }
    size_t i = nearest_index();
    double remain = horizon;
    NedPoint cur = pose_;
    while (i + 1 < global_path_.size() && remain > 1e-3) {
      const NedPoint & nxt = global_path_[i + 1];
      const double seg = dist2(cur, nxt);
      if (seg <= 1e-6) {
        ++i;
        continue;
      }
      if (seg >= remain) {
        const double t = remain / seg;
        out_idx = i + 1;
        return NedPoint{
          cur.n + (nxt.n - cur.n) * t,
          cur.e + (nxt.e - cur.e) * t,
          fly_alt_ned_};
      }
      remain -= seg;
      cur = nxt;
      ++i;
    }
    out_idx = global_path_.size() - 1;
    return global_path_.back();
  }

  void on_global_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<NedPoint> pts;
    for (const auto & p : msg->poses) {
      auto ne = mine_nav::map_to_ned(p.pose.position.x, p.pose.position.y);
      NedPoint q{ne.first, ne.second, fly_alt_ned_};
      if (!pts.empty() && mine_nav::hypot2(q.n - pts.back().n, q.e - pts.back().e) < 0.05) {
        continue;
      }
      pts.push_back(q);
    }
    if (pts.size() < 2) {
      // 起飞/预热中收到空路径：只清缓存，别打断垂直起飞
      if (phase_ == "preheat" || phase_ == "takeoff") {
        global_path_.clear();
        path_stamp_ = now();
        return;
      }
      if (!global_path_.empty() || status_ == "tracking" || status_ == "avoiding") {
        RCLCPP_WARN(get_logger(), "全局路径被清空/过短，悬停等待新路径");
        global_path_.clear();
        goal_valid_ = false;
        avoid_active_ = false;
        phase_ = "wait_path";
        set_status("waiting_path");
      }
      path_stamp_ = now();
      return;
    }
    path_stamp_ = now();
    if (!global_path_.empty() && path_same(pts)) {
      // 起飞/预热阶段只缓存路径，绝不立刻横飞
      if (phase_ == "wait_path") {
        phase_ = "fly";
        set_status("tracking");
      }
      return;
    }
    global_path_ = std::move(pts);
    ++path_id_;
    wp_index_ = nearest_index();
    avoid_active_ = false;
    hold_ticks_ = 0;
    last_progress_pose_ = pose_;
    last_progress_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "收到全局路径 #%d，共 %zu 点；从最近点 %zu/%zu 开始跟（当前阶段=%s）",
      path_id_, global_path_.size(), wp_index_ + 1, global_path_.size(),
      phase_.c_str());
    // 预热/起飞中只收路径，等垂直起飞到位后再切 fly
    if (phase_ == "preheat" || phase_ == "takeoff") {
      return;
    }
    goal_ = global_path_[wp_index_];
    goal_valid_ = true;
    // 新路径相对当前运动方向大掉头：先短悬停，避免贴墙急转撞墙
    if (have_pose_ && phase_ == "fly") {
      const double dn = goal_.n - pose_.n;
      const double de = goal_.e - pose_.e;
      const double dlen = mine_nav::hypot2(dn, de);
      const double vlen = mine_nav::hypot2(sp_vel_.n, sp_vel_.e);
      if (dlen > 0.4 && vlen > 0.05) {
        const double cosang =
          (dn * sp_vel_.n + de * sp_vel_.e) / (dlen * vlen);
        const double ang_deg = std::acos(std::clamp(cosang, -1.0, 1.0)) * 180.0 / M_PI;
        if (ang_deg >= turn_hold_deg_) {
          turn_hold_until_ = now() + rclcpp::Duration::from_seconds(0.8);
          local_target_ = pose_;
          sp_vel_ = NedPoint{0, 0, 0};
          RCLCPP_WARN(
            get_logger(),
            "新路径大掉头 %.0f°，先悬停 0.8s 再跟", ang_deg);
          set_status("tracking");
          return;
        }
      }
    }
    local_target_ = goal_;
    if (phase_ == "wait_path" || phase_ == "fly" || phase_ == "done") {
      phase_ = "fly";
      set_status("tracking");
      replan_local_target(true);
    }
  }

  bool segment_blocked(
    double n0, double e0, double n1, double e1,
    std::string & why,
    bool ignore_unknown = false) const
  {
    if (!grid_.ready) {
      why = "no_map";
      return false;
    }
    auto m0 = mine_nav::ned_to_map(n0, e0);
    auto m1 = mine_nav::ned_to_map(n1, e1);
    const double dx = m1.first - m0.first;
    const double dy = m1.second - m0.second;
    const double dist = mine_nav::hypot2(dx, dy);
    const double step = std::max(0.1, grid_.res * 0.8);
    const int n = std::max(1, static_cast<int>(dist / step));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const double x = m0.first + dx * t;
      const double y = m0.second + dy * t;
      const double along = dist * t;
      if (grid_.is_occupied(x, y, robot_radius_)) {
        why = "occupied";
        return true;
      }
      // 未知只挡近处；终点冲刺阶段忽略未知，避免在前沿前空绕
      if (!ignore_unknown &&
        along <= unknown_near_ &&
        grid_.is_unknown_near(x, y, robot_radius_))
      {
        why = "unknown_near";
        return true;
      }
    }
    why = "clear";
    return false;
  }

  // 找真正离开当前位置的净空点；脚下假净空直接拒绝
  bool find_escape_free(double cur_mx, double cur_my, double search_r, NedPoint & out) const
  {
    double free_x = cur_mx;
    double free_y = cur_my;
    const double min_move = std::max(0.30, min_escape_dist_);
    const bool got = mine_nav::find_nearest_free(
      cur_mx, cur_my, search_r, 0.15,
      [this, cur_mx, cur_my, min_move](double x, double y) {
        if (mine_nav::hypot2(x - cur_mx, y - cur_my) < min_move) {
          return false;  // 拒绝脚下/贴身假净空
        }
        if (grid_.is_occupied(x, y, robot_radius_)) {
          return false;
        }
        // 优先要软半径也更干净；硬半径合法即可通过，软半径加分在外层采样里
        return true;
      },
      free_x, free_y);
    if (!got) {
      return false;
    }
    auto free_ned = mine_nav::map_to_ned(free_x, free_y);
    out = NedPoint{free_ned.first, free_ned.second, fly_alt_ned_};
    return dist2(pose_, out) >= min_move;
  }

  bool point_free(double n, double e, bool ignore_unknown = false) const
  {
    if (!grid_.ready) {
      return true;
    }
    auto m = mine_nav::ned_to_map(n, e);
    if (grid_.is_occupied(m.first, m.second, robot_radius_)) {
      return false;
    }
    // 落点本身若未知也别去；终点冲刺时只挡真实占用
    if (!ignore_unknown &&
      grid_.is_unknown_near(m.first, m.second, robot_radius_ * 0.35))
    {
      return false;
    }
    return true;
  }

  bool near_path_end(double dist_thresh) const
  {
    if (global_path_.empty() || !have_pose_) {
      return false;
    }
    return dist2(pose_, global_path_.back()) <= dist_thresh;
  }

  // 任意地图通用：直线通就跟前瞻点；不通就周围采样绕障
  void replan_local_target(bool force)
  {
    if (!goal_valid_ || !have_pose_) {
      return;
    }
    if (!grid_.ready) {
      local_target_ = goal_;
      avoid_active_ = false;
      if (status_ != "waiting_path" && status_ != "arrived") {
        set_status("tracking");
      }
      return;
    }

    const NedPoint path_end = global_path_.empty() ? goal_ : global_path_.back();
    const double remain_end = dist2(pose_, path_end);

    // 只有“离终点近 + 到终点直线没有真实占用”才允许冲刺。
    // 否则会在绕障半途把前瞻改成终点，又撞回红柱/墙角。
    std::string why_end_probe;
    const bool end_line_clear = !segment_blocked(
      pose_.n, pose_.e, path_end.n, path_end.e, why_end_probe, true);
    const bool final_approach =
      remain_end <= final_approach_dist_ && end_line_clear;
    const bool ignore_unknown = final_approach || remain_end <= final_approach_dist_;

    size_t look_idx = wp_index_;
    NedPoint look = lookahead_point(lookahead_, look_idx);
    // 已经贴终点且直线通：别再拿远处前瞻去绕未知，直接冲终点
    if (final_approach) {
      look = path_end;
      look_idx = global_path_.empty() ? wp_index_ : global_path_.size() - 1;
    } else if (!global_path_.empty() && look_idx + 1 < global_path_.size()) {
      // 优先跟“还没走完的路径点”，避免被终点吸引穿障
      const NedPoint & nxt = global_path_[std::min(look_idx + 1, global_path_.size() - 1)];
      std::string why_nxt;
      if (!segment_blocked(pose_.n, pose_.e, nxt.n, nxt.e, why_nxt, ignore_unknown)) {
        look = nxt;
        look_idx = std::min(look_idx + 1, global_path_.size() - 1);
      }
    }
    goal_ = look;
    wp_index_ = std::max(wp_index_, look_idx > 0 ? look_idx - 1 : 0);

    std::string why;
    bool blocked =
      segment_blocked(pose_.n, pose_.e, look.n, look.e, why, ignore_unknown);

    // 关键修复：膨胀圆碰到墙 ≠ 机体已经贴墙。
    // 旧逻辑会“假贴墙侧移硬推”，这就是撞墙根因。
    // 新策略：
    // 1) 硬半径撞实障 → 立刻悬停，等全局重规划
    // 2) 仅软半径吃到墙 → 回路径中心/近前瞻，绝不侧移硬推
    if (blocked && grid_.ready) {
      auto pm = mine_nav::ned_to_map(pose_.n, pose_.e);
      const bool hard_hit = grid_.is_occupied(pm.first, pm.second, robot_radius_);
      const bool soft_hit = grid_.is_occupied(pm.first, pm.second, soft_radius_);
      if (hard_hit) {
        // 硬半径擦障：找真正离开当前位置的硬半径净空，绝不横向硬推墙
        NedPoint escape;
        if (find_escape_free(pm.first, pm.second, 2.0, escape)) {
          local_target_ = escape;
          avoid_active_ = true;
          set_status("avoiding");
          if (force || (now() - last_diag_).seconds() >= 1.0) {
            last_diag_ = now();
            RCLCPP_WARN(
              get_logger(),
              "硬半径擦障，挪到近处净空 NED=(%.2f,%.2f)，禁止侧移硬推",
              local_target_.n, local_target_.e);
          }
          return;
        }
        local_target_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        avoid_active_ = false;
        set_status("stuck");
        if (force || (now() - last_diag_).seconds() >= 1.0) {
          last_diag_ = now();
          RCLCPP_ERROR(get_logger(), "硬半径撞障且附近无有效净空，悬停等全局重规划");
        }
        return;
      }
      if (soft_hit) {
        // 优先回到路径上更中心的点，而不是横向挤墙
        size_t center_idx = wp_index_;
        NedPoint center = lookahead_point(std::max(0.8, lookahead_ * 0.6), center_idx);
        std::string why_center;
        const bool center_ok =
          point_free(center.n, center.e, true) &&
          dist2(pose_, center) >= min_escape_dist_ &&
          !segment_blocked(pose_.n, pose_.e, center.n, center.e, why_center, true);
        if (center_ok) {
          local_target_ = center;
          avoid_active_ = true;
          set_status("avoiding");
          if (force || (now() - last_diag_).seconds() >= 1.0) {
            last_diag_ = now();
            RCLCPP_WARN(
              get_logger(),
              "软膨胀贴墙，回路径中心 NED=(%.2f,%.2f)，禁止侧移硬推",
              center.n, center.e);
          }
          return;
        }
        // 回中失败：找真正离开当前位置的净空；脚下假净空直接判 stuck
        NedPoint escape;
        if (find_escape_free(pm.first, pm.second, 2.2, escape)) {
          local_target_ = escape;
          avoid_active_ = true;
          set_status("avoiding");
          if (force || (now() - last_diag_).seconds() >= 1.0) {
            last_diag_ = now();
            RCLCPP_WARN(
              get_logger(),
              "软膨胀贴墙回中失败，挪到近处净空 NED=(%.2f,%.2f)",
              local_target_.n, local_target_.e);
          }
          return;
        }
        local_target_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        avoid_active_ = false;
        set_status("stuck");
        if (force || (now() - last_diag_).seconds() >= 1.0) {
          last_diag_ = now();
          RCLCPP_WARN(get_logger(), "软膨胀贴墙且附近无有效净空（拒绝脚下假净空），悬停等重规划");
        }
        return;
      }
    }

    if (force || (now() - last_diag_).seconds() >= 1.0) {
      last_diag_ = now();
      RCLCPP_INFO(
        get_logger(),
        "局部检查 look=(%.2f,%.2f) remain=%.2f end=%.2f | %s%s",
        look.n, look.e, dist2(pose_, look), remain_end, why.c_str(),
        final_approach ? " | final_approach" : "");
    }

    if (!blocked) {
      local_target_ = look;
      avoid_active_ = false;
      set_status("tracking");
      return;
    }

    // 终点附近只剩未知挡着，且直线无真实占用：直接跟终点，别左右空绕
    if (final_approach && why == "unknown_near") {
      local_target_ = path_end;
      avoid_active_ = false;
      set_status("tracking");
      return;
    }

    // 直线不通：在周围按环+角度采样，选“能走到、且更接近路径前方”的点
    struct Cand
    {
      NedPoint p;
      double score;
      std::string how;
    };
    std::vector<Cand> cands;
    const int rings = std::max(1, sample_rings_);
    const int angles = std::max(8, sample_angles_);
    const double base_r = std::max(0.5, sample_radius_ / rings);
    // 采样绕障统一用硬半径：只保证机体能过去；软半径只做净空偏好，不再缩小硬半径硬挤
    const double check_radius = robot_radius_;
    const double old_radius = robot_radius_;
    robot_radius_ = check_radius;

    // 路径前进方向，用于打分偏好；优先沿着全局路径走，而不是直扑终点
    double path_dn = look.n - pose_.n;
    double path_de = look.e - pose_.e;
    if (!global_path_.empty() && wp_index_ + 1 < global_path_.size()) {
      path_dn = global_path_[wp_index_ + 1].n - pose_.n;
      path_de = global_path_[wp_index_ + 1].e - pose_.e;
    }
    const double path_len = mine_nav::hypot2(path_dn, path_de);
    if (path_len > 1e-3) {
      path_dn /= path_len;
      path_de /= path_len;
    } else {
      path_dn = 1.0;
      path_de = 0.0;
    }

    auto try_add_cand = [&](double cn, double ce, const char * tag) {
      if (!point_free(cn, ce, ignore_unknown)) {
        return;
      }
      std::string why_seg;
      if (segment_blocked(pose_.n, pose_.e, cn, ce, why_seg, ignore_unknown)) {
        return;
      }
      const double step_fwd = std::min(1.0, lookahead_ * 0.35);
      const NedPoint fwd{
        cn + path_dn * step_fwd,
        ce + path_de * step_fwd,
        fly_alt_ned_};
      std::string why_fwd;
      const bool fwd_ok =
        !segment_blocked(cn, ce, fwd.n, fwd.e, why_fwd, ignore_unknown) &&
        point_free(fwd.n, fwd.e, ignore_unknown);

      const double to_look = mine_nav::hypot2(cn - look.n, ce - look.e);
      const double to_end = mine_nav::hypot2(cn - path_end.n, ce - path_end.e);
      const double progress = path_dn * (cn - pose_.n) + path_de * (ce - pose_.e);
      const double detour = std::abs(path_dn * (ce - pose_.e) - path_de * (cn - pose_.n));
      auto cm = mine_nav::ned_to_map(cn, ce);
      const bool soft_clear = !grid_.is_occupied(cm.first, cm.second, soft_radius_);
      Cand c;
      c.p = NedPoint{cn, ce, fly_alt_ned_};
      // 越小越好：靠近路径前方、少侧向绕、有前进量；前方可续飞/软半径净空再加分
      c.score = progress_weight_ * to_look +
        0.35 * to_end +
        detour_weight_ * detour -
        0.55 * progress -
        (fwd_ok ? clear_bonus_ : 0.0) -
        (soft_clear ? 0.25 : 0.0);
      c.how = std::string(tag) + (fwd_ok ? "|clear_ahead" : ("|" + why_fwd)) +
        (soft_clear ? "|soft_ok" : "|soft_tight");
      cands.push_back(c);
    };

    // 先塞几个“沿全局路径侧移/回退”的强候选，避免只围着终点打转
    if (!global_path_.empty()) {
      for (size_t i = wp_index_; i < global_path_.size(); ++i) {
        try_add_cand(global_path_[i].n, global_path_[i].e, "path_wp");
      }
      // 左右侧移半个机身，专门绕柱
      const double side_n = -path_de;
      const double side_e = path_dn;
      for (double s : {-1.0, 1.0}) {
        for (double rad : {0.7, 1.1, 1.6}) {
          try_add_cand(
            pose_.n + side_n * s * rad + path_dn * 0.4,
            pose_.e + side_e * s * rad + path_de * 0.4,
            "side_step");
        }
      }
    }

    for (int r = 1; r <= rings; ++r) {
      const double rad = base_r * r;
      for (int a = 0; a < angles; ++a) {
        // 每圈错半格，覆盖更密，减少窄缝漏采
        const double ang =
          2.0 * M_PI * (static_cast<double>(a) + 0.5 * (r % 2)) / angles;
        const double cn = pose_.n + rad * std::cos(ang);
        const double ce = pose_.e + rad * std::sin(ang);
        try_add_cand(cn, ce, "ring");
      }
    }
    robot_radius_ = old_radius;

    if (cands.empty()) {
      // 终点冲刺：采样也绕不过，但终点本身没有真实占用且直线通 → 直接跟终点
      if (final_approach && point_free(path_end.n, path_end.e, true) && end_line_clear) {
        local_target_ = path_end;
        avoid_active_ = false;
        set_status("tracking");
        RCLCPP_INFO(
          get_logger(),
          "终点冲刺：忽略未知，直冲 NED=(%.2f,%.2f)",
          path_end.n, path_end.e);
        return;
      }
      // 先别立刻 stuck：缩短前瞻再试一次；仍用硬半径，绝不缩小半径硬挤
      if (lookahead_ > 1.2) {
        const NedPoint near = lookahead_point(std::max(0.8, lookahead_ * 0.45), look_idx);
        std::string why_near;
        const bool near_ok = !segment_blocked(
          pose_.n, pose_.e, near.n, near.e, why_near, ignore_unknown) &&
          point_free(near.n, near.e, ignore_unknown);
        if (near_ok) {
          local_target_ = near;
          avoid_active_ = true;
          set_status("avoiding");
          RCLCPP_INFO(
            get_logger(),
            "远前瞻被挡(%s)，改跟近前瞻 NED=(%.2f,%.2f)",
            why.c_str(), near.n, near.e);
          return;
        }
      }
      local_target_ = pose_;
      avoid_active_ = true;
      set_status("stuck");
      RCLCPP_WARN(get_logger(), "前方有障(%s)，周围采样绕不过，先悬停", why.c_str());
      return;
    }

    std::sort(
      cands.begin(), cands.end(), [](const Cand & a, const Cand & b) {
        return a.score < b.score;
      });
    const auto & best = cands.front();
    local_target_ = best.p;
    avoid_active_ = true;
    set_status("avoiding");
    if (force || (now() - last_diag_).seconds() >= 1.0) {
      last_diag_ = now();
      RCLCPP_INFO(
        get_logger(),
        "检测到障碍(%s)，绕到 NED=(%.2f,%.2f) | %s",
        why.c_str(), best.p.n, best.p.e, best.how.c_str());
    }
  }

  NedPoint step_toward(const NedPoint & cur, const NedPoint & tgt, double max_step, NedPoint & vel)
  {
    const double dn = tgt.n - cur.n;
    const double de = tgt.e - cur.e;
    const double dd = tgt.d - cur.d;
    const double dist = std::sqrt(dn * dn + de * de + dd * dd);
    if (dist <= max_step || dist == 0.0) {
      vel = NedPoint{0, 0, 0};
      return tgt;
    }
    const double s = max_step / dist;
    vel = NedPoint{dn / dist * max_speed_, de / dist * max_speed_, dd / dist * max_speed_};
    return NedPoint{cur.n + dn * s, cur.e + de * s, cur.d + dd * s};
  }

  void publish_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = true;
    mode_pub_->publish(msg);
  }

  void publish_setpoint()
  {
    const float nanf = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = {
      static_cast<float>(sp_.n),
      static_cast<float>(sp_.e),
      static_cast<float>(sp_.d)};
    // 起飞阶段只发位置（速度 NaN），避免水平速度前馈把桨一打就侧倾擦地
    // 对照能飞的 offboard_takeoff：position 有效 + velocity=NaN
    if (phase_ == "preheat" || phase_ == "takeoff") {
      msg.velocity = {nanf, nanf, nanf};
    } else {
      msg.velocity = {
        static_cast<float>(sp_vel_.n),
        static_cast<float>(sp_vel_.e),
        static_cast<float>(sp_vel_.d)};
    }
    msg.acceleration = {nanf, nanf, nanf};
    msg.jerk = {nanf, nanf, nanf};
    msg.yaw = 0.0f;
    msg.yawspeed = nanf;
    sp_pub_->publish(msg);
  }

  // 起飞悬停点：锁当前水平位置，只改高度。绝不能硬编码 (0,0)
  NedPoint takeoff_hover() const
  {
    return NedPoint{hover_n_, hover_e_, fly_alt_ned_};
  }

  bool takeoff_reached()
  {
    // 相对起飞高度爬升，且连续多帧达标；防 EKF 单帧 z 跳变骗过判定后立刻横飞擦地
    const double climb = takeoff_z0_ - pose_.d;  // NED: 向上爬升 => d 变小
    const double need = std::max(0.6, (-fly_alt_ned_) * 0.80);
    if (climb >= need) {
      ++takeoff_ok_ticks_;
    } else {
      takeoff_ok_ticks_ = 0;
    }
    return takeoff_ok_ticks_ >= 20;  // 连续 1.0s
  }

  void send_command(uint32_t command, float p1, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
  }

  void publish_local_path()
  {
    if (!have_pose_ || !goal_valid_) {
      return;
    }
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_init";
    auto push_ned = [&](const NedPoint & p) {
        auto m = mine_nav::ned_to_map(p.n, p.e);
        msg.poses.push_back(mine_nav::make_pose(msg.header, m.first, m.second, -fly_alt_ned_));
      };
    push_ned(pose_);
    push_ned(local_target_);
    push_ned(goal_);
    local_path_pub_->publish(msg);
  }

  void tick()
  {
    ++n_;
    const double elapsed = n_ * 0.05;
    if (!sp_init_) {
      return;
    }

    // 位姿异常保护：只发悬停 setpoint，不再跟路径/绕障
    if (pose_safe_hold_) {
      if (phase_ != "preheat" && phase_ != "takeoff") {
        local_target_ = pose_;
        goal_ = pose_;
        sp_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        goal_valid_ = false;
        avoid_active_ = false;
        if (status_ != "pose_invalid") {
          set_status("pose_invalid");
        }
      }
      publish_mode();
      publish_setpoint();
      if (n_ % 40 == 0) {
        publish_status();
        RCLCPP_ERROR(
          get_logger(),
          "[ %.0fs | pose_safe_hold ] 位置 N=%.2f E=%.2f 高=%.2f xy_valid=%d",
          elapsed, pose_.n, pose_.e, -pose_.d, pose_xy_valid_ ? 1 : 0);
      }
      return;
    }

    // 第一次拿到位姿时锁定起飞水平位置（相对起飞点，不是写死地图原点）
    if (!hover_locked_) {
      hover_n_ = pose_.n;
      hover_e_ = pose_.e;
      takeoff_z0_ = pose_.d;
      hover_locked_ = true;
      RCLCPP_INFO(
        get_logger(),
        "锁定起飞点 NED=(%.2f,%.2f,z0=%.2f)，目标高度=%.2fm",
        hover_n_, hover_e_, takeoff_z0_, -fly_alt_ned_);
    }

    if (phase_ == "preheat") {
      // 预热阶段就持续发“当前水平 + 起飞高度”，和 offboard_takeoff 一样
      // 绝不能发硬编码 (0,0)：视觉定位一起飞原点可能已漂，一解锁就横飞左倾
      local_target_ = takeoff_hover();
      goal_ = local_target_;
      goal_valid_ = true;
      sp_ = local_target_;
      sp_vel_ = NedPoint{0, 0, 0};
      if (!armed_sent_ && elapsed >= 2.0) {
        armed_sent_ = true;
        // 解锁瞬间再刷一次起飞基准高度，防止预热期 EKF 已漂
        takeoff_z0_ = pose_.d;
        takeoff_ok_ticks_ = 0;
        RCLCPP_INFO(
          get_logger(),
          "切换到 OFFBOARD 并解锁；先垂直起飞到 %.2fm（相对爬升），到位后再跟路径",
          -fly_alt_ned_);
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        send_command(100001, 14.0f);
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        phase_ = "takeoff";
        set_status("taking_off");
      }
    } else if (phase_ == "takeoff") {
      // 只垂直升：水平锁在 hover，高度到 fly_alt；到位前绝不跟全局路径
      local_target_ = takeoff_hover();
      goal_ = local_target_;
      goal_valid_ = true;
      sp_ = local_target_;
      sp_vel_ = NedPoint{0, 0, 0};
      if (takeoff_reached()) {
        RCLCPP_INFO(
          get_logger(),
          "起飞到位 高度=%.2fm 相对爬升=%.2fm，开始等/跟 /global_path",
          -pose_.d, takeoff_z0_ - pose_.d);
        if (!global_path_.empty()) {
          wp_index_ = nearest_index();
          goal_ = global_path_[wp_index_];
          goal_valid_ = true;
          local_target_ = goal_;
          phase_ = "fly";
          set_status("tracking");
          last_progress_pose_ = pose_;
          last_progress_sp_ = sp_;
          last_progress_time_ = now();
          replan_local_target(true);
        } else {
          phase_ = "wait_path";
          set_status("waiting_path");
        }
      }
    } else if (phase_ == "wait_path") {
      // 等路径时悬停在起飞水平点 + 起飞高度，不要每拍盖回地图原点
      local_target_ = takeoff_hover();
      goal_ = local_target_;
      goal_valid_ = true;
      sp_ = step_toward(sp_, local_target_, max_speed_ * 0.05, sp_vel_);
      if (dist3(pose_, local_target_) <= arrive_dist_) {
        sp_vel_ = NedPoint{0, 0, 0};
      }
      if (!global_path_.empty()) {
        phase_ = "fly";
        set_status("tracking");
        last_progress_pose_ = pose_;
        last_progress_sp_ = sp_;
        last_progress_time_ = now();
        replan_local_target(true);
      }
    } else if (phase_ == "fly") {
      if (global_path_.empty()) {
        phase_ = "wait_path";
        set_status("waiting_path");
        last_progress_pose_ = pose_;
        last_progress_time_ = now();
      }
      if ((now() - path_stamp_).seconds() > path_stale_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "全局路径已 %.1fs 未更新，继续跟旧路径",
          (now() - path_stamp_).seconds());
      }
      if ((now() - last_replan_).seconds() >= replan_period_) {
        last_replan_ = now();
        replan_local_target(false);
      }

      const double remain_to_local = dist3(pose_, local_target_);
      const double remain_to_goal = goal_valid_ ? dist3(pose_, goal_) : 0.0;
      // setpoint 已经在往前推时，别只看位姿：假世界抖动/幽灵节点会把 pose 钉死
      const double sp_to_target = dist2(sp_, local_target_);
      const bool expect_motion =
        status_ != "arrived" &&
        status_ != "waiting_path" &&
        status_ != "stuck" &&
        remain_to_local > arrive_dist_ * 1.5 &&
        remain_to_goal > arrive_dist_ * 1.5 &&
        sp_to_target > arrive_dist_ * 0.5;
      const double moved = mine_nav::hypot2(
        pose_.n - last_progress_pose_.n, pose_.e - last_progress_pose_.e);
      const double sp_moved = mine_nav::hypot2(
        sp_.n - last_progress_sp_.n, sp_.e - last_progress_sp_.e);
      if (!expect_motion || moved > 0.25 || sp_moved > 0.20) {
        last_progress_pose_ = pose_;
        last_progress_sp_ = sp_;
        last_progress_time_ = now();
        if (status_ == "stuck" && (moved > 0.25 || sp_moved > 0.20)) {
          set_status(avoid_active_ ? "avoiding" : "tracking");
        }
      } else if ((now() - last_progress_time_).seconds() >= stuck_timeout_sec_) {
        if (status_ != "stuck") {
          RCLCPP_WARN(
            get_logger(),
            "疑似卡住：%.1fs 几乎没动，remain_local=%.2f remain_goal=%.2f sp_to_tgt=%.2f，先悬停",
            (now() - last_progress_time_).seconds(), remain_to_local, remain_to_goal,
            sp_to_target);
        }
        set_status("stuck");
        local_target_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        avoid_active_ = false;
      }

      // 大掉头悬停窗口 / 高度骤降：先稳住，别继续硬推
      if (turn_hold_until_.nanoseconds() > 0 && now() < turn_hold_until_) {
        local_target_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        sp_ = pose_;
      } else if (alt_drop_hold_) {
        local_target_ = pose_;
        sp_vel_ = NedPoint{0, 0, 0};
        sp_ = pose_;
        if ((-pose_.d) >= (-fly_alt_ned_) * 0.75) {
          alt_drop_hold_ = false;
          RCLCPP_WARN(get_logger(), "高度已回升，解除高度骤降保护");
        }
      } else {
        const double speed_cap =
          avoid_active_ ? max_speed_ * std::clamp(avoid_speed_scale_, 0.2, 1.0) : max_speed_;
        sp_ = step_toward(sp_, local_target_, speed_cap * 0.05, sp_vel_);
      }
      const double remain_local = dist3(pose_, local_target_);
      const double remain_goal = dist3(pose_, goal_);
      if (status_ == "stuck") {
        // 卡住时先悬停，等探索器换目标 / 新路径到来
      } else if (avoid_active_ && remain_local <= arrive_dist_) {
        hold_ticks_ = 0;
        replan_local_target(true);
      } else if (!avoid_active_ && remain_goal <= arrive_dist_) {
        ++hold_ticks_;
        sp_vel_ = NedPoint{0, 0, 0};
        if (hold_ticks_ >= static_cast<int>(hold_sec_ / 0.05)) {
          // 路径终点附近：推进到更远的路径点，或宣布到位
          size_t look_idx = wp_index_;
          const NedPoint far = lookahead_point(lookahead_, look_idx);
          if (look_idx + 1 >= global_path_.size() &&
            dist2(pose_, global_path_.back()) <= arrive_dist_ * 1.5)
          {
            if (status_ != "arrived") {
              RCLCPP_INFO(get_logger(), "全局路径跟完，悬停待命");
              set_status("arrived");
            }
            local_target_ = pose_;
            goal_ = pose_;
          } else {
            wp_index_ = std::min(look_idx, global_path_.size() - 1);
            goal_ = far;
            local_target_ = goal_;
            avoid_active_ = false;
            hold_ticks_ = 0;
            last_progress_pose_ = pose_;
            last_progress_time_ = now();
            RCLCPP_INFO(
              get_logger(), "继续跟路径 %zu/%zu: NED=(%.2f,%.2f)",
              wp_index_ + 1, global_path_.size(), goal_.n, goal_.e);
            replan_local_target(true);
          }
        }
      } else {
        hold_ticks_ = 0;
      }
    }

    publish_mode();
    publish_setpoint();
    publish_local_path();
    if (n_ % 40 == 0) {
      publish_status();
      RCLCPP_INFO(
        get_logger(),
        "[ %.0fs | %s | %s ] path=%zu/%zu 位置 N=%.2f E=%.2f 高=%.2f | avoid=%d",
        elapsed, phase_.c_str(), status_.c_str(),
        global_path_.empty() ? 0 : wp_index_ + 1, global_path_.size(),
        pose_.n, pose_.e, -pose_.d, avoid_active_ ? 1 : 0);
    }
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<mine_nav::msg::LocalStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mine_nav::GridMap grid_;
  std::vector<NedPoint> global_path_;
  NedPoint pose_;
  NedPoint goal_;
  NedPoint local_target_;
  NedPoint sp_;
  NedPoint sp_vel_;
  bool have_pose_{false};
  bool goal_valid_{false};
  bool sp_init_{false};
  bool avoid_active_{false};
  bool armed_sent_{false};
  bool hover_locked_{false};
  bool pose_safe_hold_{false};
  bool pose_xy_valid_{true};
  bool pose_z_valid_{true};
  int pose_bad_ticks_{0};
  double hover_n_{0.0};
  double hover_e_{0.0};
  double takeoff_z0_{0.0};
  int takeoff_ok_ticks_{0};
  size_t wp_index_{0};
  int path_id_{0};
  int n_{0};
  int hold_ticks_{0};
  std::string phase_{"preheat"};
  std::string status_{"idle"};
  rclcpp::Time path_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_{0, 0, RCL_ROS_TIME};

  double max_speed_{0.30};
  double avoid_speed_scale_{0.55};
  double fly_alt_ned_{-1.2};
  double robot_radius_{0.40};   // 硬半径：机体+桨叶真实碰撞
  double soft_radius_{0.55};    // 软半径：净空偏好，不用于侧移硬推
  double arrive_dist_{0.40};
  double hold_sec_{1.0};
  double lookahead_{1.6};
  double unknown_near_{1.8};
  double final_approach_dist_{1.2};
  double replan_period_{0.35};
  double stuck_timeout_sec_{4.0};
  double path_stale_sec_{3.0};
  double sample_radius_{2.4};
  int sample_rings_{4};
  int sample_angles_{24};
  double progress_weight_{1.0};
  double detour_weight_{0.35};
  double clear_bonus_{0.45};
  double turn_hold_deg_{110.0};
  double alt_drop_limit_{0.28};
  double min_fly_alt_{0.70};
  double pose_abs_limit_{80.0};
  double pose_jump_limit_{2.5};
  double min_escape_dist_{0.35};  // 净空脱困至少离开当前位置这么远，拒绝脚下假净空
  bool alt_drop_hold_{false};
  rclcpp::Time turn_hold_until_{0, 0, RCL_ROS_TIME};
  NedPoint last_progress_pose_;
  NedPoint last_progress_sp_;
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalControllerNode>());
  rclcpp::shutdown();
  return 0;
}
