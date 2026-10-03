#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * 精简路径跟踪：订局部路径（默认 /local_path），起飞后跟路径飞到终点悬停。
 * 不绑巷道；坐标约定：路径在 map/ENU(camera_init)，飞控用 NED。
 * map_x = ned_e, map_y = ned_n
 */
class PathTrackerNode : public rclcpp::Node
{
public:
  PathTrackerNode()
  : Node("path_tracker")
  {
    path_topic_ = declare_parameter<std::string>("path_topic", "/local_path");
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/global_path");
    local_pos_topic_ = declare_parameter<std::string>(
      "local_pos_topic", "/fmu/out/vehicle_local_position_v1");
    max_speed_ = declare_parameter<double>("max_speed", 0.30);
    fly_alt_ned_ = declare_parameter<double>("fly_alt_ned", -1.2);
    arrive_dist_ = declare_parameter<double>("arrive_dist", 0.40);
    lookahead_ = declare_parameter<double>("lookahead", 1.0);
    hold_sec_ = declare_parameter<double>("hold_sec", 2.0);
    path_stale_sec_ = declare_parameter<double>("path_stale_sec", 5.0);
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
    status_pub_ = create_publisher<std_msgs::msg::String>("/path_tracker/status", 10);
    track_path_pub_ = create_publisher<nav_msgs::msg::Path>("/tracked_path", 10);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_pos_topic_, qos_be_tl,
      std::bind(&PathTrackerNode::on_local_pos, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, qos_rel,
      std::bind(&PathTrackerNode::on_path, this, std::placeholders::_1));
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, qos_rel,
      std::bind(&PathTrackerNode::on_global_path, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&PathTrackerNode::tick, this));
    RCLCPP_INFO(
      get_logger(),
      "path_tracker: local=%s global=%s local_pos=%s speed=%.2f alt=%.2fm",
      path_topic_.c_str(), global_path_topic_.c_str(), local_pos_topic_.c_str(),
      max_speed_, -fly_alt_ned_);
  }

private:
  struct Ned
  {
    double n{0.0};
    double e{0.0};
    double d{0.0};
  };

  static double hypot2(double dx, double dy)
  {
    return std::sqrt(dx * dx + dy * dy);
  }

  static double dist2(const Ned & a, const Ned & b)
  {
    return hypot2(a.n - b.n, a.e - b.e);
  }

  static double dist3(const Ned & a, const Ned & b)
  {
    return std::sqrt(
      (a.n - b.n) * (a.n - b.n) +
      (a.e - b.e) * (a.e - b.e) +
      (a.d - b.d) * (a.d - b.d));
  }

  // map/ENU -> NED
  static Ned map_to_ned(double mx, double my, double alt_ned)
  {
    return Ned{my, mx, alt_ned};
  }

  // NED -> map/ENU
  static std::pair<double, double> ned_to_map(double n, double e)
  {
    return {e, n};
  }

  void set_status(const std::string & s)
  {
    if (s == status_) {
      return;
    }
    status_ = s;
    publish_status();
    RCLCPP_INFO(get_logger(), "状态 -> %s", status_.c_str());
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data = status_;
    status_pub_->publish(msg);
  }

  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      if (!pose_safe_hold_) {
        RCLCPP_ERROR(get_logger(), "位姿无效 xy_valid=%d z_valid=%d，进入安全悬停",
          msg->xy_valid ? 1 : 0, msg->z_valid ? 1 : 0);
      }
      pose_safe_hold_ = true;
      return;
    }

    const Ned p{msg->x, msg->y, msg->z};
    if (std::abs(p.n) > pose_abs_limit_ || std::abs(p.e) > pose_abs_limit_) {
      if (!pose_safe_hold_) {
        RCLCPP_ERROR(get_logger(), "位姿越界 N=%.1f E=%.1f，进入安全悬停", p.n, p.e);
      }
      pose_safe_hold_ = true;
      return;
    }

    if (have_pose_) {
      const double jump = dist2(pose_, p);
      if (jump > pose_jump_limit_) {
        ++pose_jump_count_;
        if (pose_jump_count_ >= 3) {
          if (!pose_safe_hold_) {
            RCLCPP_ERROR(get_logger(), "连续大跳变 %.2fm，进入安全悬停", jump);
          }
          pose_safe_hold_ = true;
          return;
        }
        return;  // 丢弃这一帧
      }
      pose_jump_count_ = 0;
    }

    pose_ = p;
    have_pose_ = true;
    if (!sp_init_) {
      sp_ = pose_;
      sp_init_ = true;
    }
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }
    std::vector<Ned> pts;
    pts.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      pts.push_back(map_to_ned(ps.pose.position.x, ps.pose.position.y, fly_alt_ned_));
    }
    path_ = std::move(pts);
    path_stamp_ = now();
    wp_index_ = 0;
    if (have_pose_ && !path_.empty()) {
      wp_index_ = nearest_index();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "收到局部路径 %zu 点，起点NED=(%.2f,%.2f) 末端NED=(%.2f,%.2f)",
      path_.size(), path_.front().n, path_.front().e, path_.back().n, path_.back().e);
  }

  void on_global_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      have_global_goal_ = false;
      return;
    }
    const auto & p = msg->poses.back().pose.position;
    global_goal_ = map_to_ned(p.x, p.y, fly_alt_ned_);
    have_global_goal_ = true;
  }

  size_t nearest_index() const
  {
    size_t best = 0;
    double best_d = 1e18;
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = dist2(pose_, path_[i]);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }

  Ned lookahead_target()
  {
    if (path_.empty()) {
      return pose_;
    }
    // 从最近点往前找，累计弧长达到 lookahead
    size_t i = nearest_index();
    wp_index_ = i;
    double acc = 0.0;
    Ned prev = path_[i];
    for (; i + 1 < path_.size(); ++i) {
      const Ned & cur = path_[i + 1];
      acc += dist2(prev, cur);
      prev = cur;
      if (acc >= lookahead_) {
        wp_index_ = i + 1;
        return cur;
      }
    }
    wp_index_ = path_.size() - 1;
    return path_.back();
  }

  Ned step_toward(const Ned & cur, const Ned & tgt, double dt, Ned & vel)
  {
    const double dn = tgt.n - cur.n;
    const double de = tgt.e - cur.e;
    const double dd = tgt.d - cur.d;
    const double dist = std::sqrt(dn * dn + de * de + dd * dd);
    const double max_step = max_speed_ * dt;
    if (dist <= max_step || dist < 1e-6) {
      vel = Ned{0, 0, 0};
      return tgt;
    }
    const double s = max_step / dist;
    vel = Ned{dn / dist * max_speed_, de / dist * max_speed_, dd / dist * max_speed_};
    return Ned{cur.n + dn * s, cur.e + de * s, cur.d + dd * s};
  }

  void publish_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = true;
    mode_pub_->publish(msg);
  }

  void publish_setpoint(bool velocity_nan)
  {
    const float nanf = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = {
      static_cast<float>(sp_.n),
      static_cast<float>(sp_.e),
      static_cast<float>(sp_.d)};
    if (velocity_nan) {
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

  Ned takeoff_hover() const
  {
    return Ned{hover_n_, hover_e_, fly_alt_ned_};
  }

  bool takeoff_reached()
  {
    const double climb = takeoff_z0_ - pose_.d;
    const double need = std::max(0.6, (-fly_alt_ned_) * 0.80);
    if (climb >= need) {
      ++takeoff_ok_ticks_;
    } else {
      takeoff_ok_ticks_ = 0;
    }
    return takeoff_ok_ticks_ >= 20;  // 连续 1s
  }

  void publish_tracked_path(const Ned & target)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_init";
    auto push = [&](const Ned & p) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        auto m = ned_to_map(p.n, p.e);
        ps.pose.position.x = m.first;
        ps.pose.position.y = m.second;
        ps.pose.position.z = -fly_alt_ned_;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
      };
    push(pose_);
    push(target);
    if (!path_.empty()) {
      push(path_.back());
    }
    track_path_pub_->publish(msg);
  }

  void tick()
  {
    ++n_;
    const double elapsed = n_ * 0.05;
    if (!sp_init_ || !have_pose_) {
      return;
    }

    if (pose_safe_hold_) {
      sp_ = pose_;
      sp_vel_ = Ned{0, 0, 0};
      if (status_ != "pose_invalid") {
        set_status("pose_invalid");
      }
      publish_mode();
      publish_setpoint(true);
      return;
    }

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

    const bool takeoff_phase = (phase_ == "preheat" || phase_ == "takeoff");

    if (phase_ == "preheat") {
      sp_ = takeoff_hover();
      sp_vel_ = Ned{0, 0, 0};
      if (!armed_sent_ && elapsed >= 2.0) {
        armed_sent_ = true;
        takeoff_z0_ = pose_.d;
        takeoff_ok_ticks_ = 0;
        RCLCPP_INFO(
          get_logger(),
          "切 OFFBOARD 并解锁，先垂直起飞到 %.2fm", -fly_alt_ned_);
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        // 某些仿真桥需要额外确认命令；没有也不影响
        send_command(100001, 14.0f);
        send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        phase_ = "takeoff";
        set_status("taking_off");
      }
    } else if (phase_ == "takeoff") {
      sp_ = takeoff_hover();
      sp_vel_ = Ned{0, 0, 0};
      if (takeoff_reached()) {
        RCLCPP_INFO(
          get_logger(),
          "起飞到位 高度=%.2fm 相对爬升=%.2fm",
          -pose_.d, takeoff_z0_ - pose_.d);
        if (!path_.empty()) {
          phase_ = "fly";
          set_status("tracking");
        } else {
          phase_ = "wait_path";
          set_status("waiting_path");
        }
      }
    } else if (phase_ == "wait_path") {
      sp_ = step_toward(sp_, takeoff_hover(), 0.05, sp_vel_);
      if (dist3(pose_, takeoff_hover()) <= arrive_dist_) {
        sp_vel_ = Ned{0, 0, 0};
      }
      if (!path_.empty()) {
        phase_ = "fly";
        set_status("tracking");
      }
    } else if (phase_ == "fly") {
      if (path_.empty()) {
        phase_ = "wait_path";
        set_status("waiting_path");
      } else {
        if ((now() - path_stamp_).seconds() > path_stale_sec_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "局部路径已 %.1fs 未更新，继续跟旧路径",
            (now() - path_stamp_).seconds());
        }

        // 到达判据看全局终点；没有全局终点时退回局部末端
        const Ned goal = have_global_goal_ ? global_goal_ : path_.back();
        const double remain_goal = dist2(pose_, goal);
        const double local_span = dist2(path_.front(), path_.back());

        if (remain_goal <= arrive_dist_) {
          sp_ = Ned{goal.n, goal.e, fly_alt_ned_};
          sp_vel_ = Ned{0, 0, 0};
          if (status_ != "arrived") {
            arrived_since_ = now();
            set_status("arrived");
            RCLCPP_INFO(
              get_logger(),
              "到达全局目标 NED=(%.2f,%.2f)，开始悬停", goal.n, goal.e);
          }
          if ((now() - arrived_since_).seconds() >= hold_sec_) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "已在目标悬停 %.1fs", (now() - arrived_since_).seconds());
          }
        } else if (local_span < 0.05) {
          // 局部规划输出原地路径：前方挡路，悬停等下一次绕行
          sp_ = pose_;
          sp_vel_ = Ned{0, 0, 0};
          if (status_ != "blocked") {
            set_status("blocked");
          }
        } else {
          const Ned tgt = lookahead_target();
          sp_ = step_toward(sp_, tgt, 0.05, sp_vel_);
          if (status_ != "tracking") {
            set_status("tracking");
          }
          publish_tracked_path(tgt);
        }
      }
    }

    publish_mode();
    publish_setpoint(takeoff_phase);

    if (n_ % 40 == 0) {
      publish_status();
      RCLCPP_INFO(
        get_logger(),
        "[%.0fs | %s] N=%.2f E=%.2f 高=%.2f path=%zu",
        elapsed, status_.c_str(), pose_.n, pose_.e, -pose_.d, path_.size());
    }
  }

  std::string path_topic_;
  std::string global_path_topic_;
  std::string local_pos_topic_;
  double max_speed_{0.30};
  double fly_alt_ned_{-1.2};
  double arrive_dist_{0.40};
  double lookahead_{1.0};
  double hold_sec_{2.0};
  double path_stale_sec_{5.0};
  double pose_abs_limit_{80.0};
  double pose_jump_limit_{2.5};

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_path_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Ned pose_;
  Ned sp_;
  Ned sp_vel_;
  Ned global_goal_;
  bool have_pose_{false};
  bool have_global_goal_{false};
  bool sp_init_{false};
  bool pose_safe_hold_{false};
  int pose_jump_count_{0};

  bool hover_locked_{false};
  double hover_n_{0.0};
  double hover_e_{0.0};
  double takeoff_z0_{0.0};
  int takeoff_ok_ticks_{0};
  bool armed_sent_{false};

  std::vector<Ned> path_;
  rclcpp::Time path_stamp_{0, 0, RCL_ROS_TIME};
  size_t wp_index_{0};
  rclcpp::Time arrived_since_{0, 0, RCL_ROS_TIME};

  std::string phase_{"preheat"};
  std::string status_{"idle"};
  int n_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
