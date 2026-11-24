// 移植自 fastlio_odom_bridge.py：
// /Odometry (ENU+FLU) -> /fmu/in/vehicle_visual_odometry (NED+FRD)
//
// /Odometry 只在一帧点云处理完时才发布（5~10 Hz，负载高时会出现 0.3~1.1 s 空档）。
// EKF2 的 EV_MAX_INTERVAL 是 0.2 s，间隔一超就会把 ev_pos/ev_yaw/ev_hgt 停掉。
// 这里按固定频率发布，帧间只做一件事：在最近两帧之间按时间做线性插值
// （位置线性、姿态 slerp）。插值只能用已经收到的两帧，所以输出滞后约 1 帧。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace
{

using Quat = std::array<double, 4>;  // [w, x, y, z]

Quat QuatMul(const Quat & q1, const Quat & q2)
{
  return {
    q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
    q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
    q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0],
  };
}

Quat QuatConj(const Quat & q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}

Quat FluEnuToFrdNed(const Quat & q_flu_enu)
{
  // 与 PX4 GZBridge::rotateQuaternion 对齐
  static const Quat q_flu_to_frd{0.0, 1.0, 0.0, 0.0};
  static const Quat q_enu_to_ned{0.0, 0.70711, 0.70711, 0.0};
  return QuatMul(q_enu_to_ned, QuatMul(q_flu_enu, QuatConj(q_flu_to_frd)));
}

Quat QuatNormalize(const Quat & q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-9) return {1.0, 0.0, 0.0, 0.0};
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Quat QuatSlerp(const Quat & q0_in, const Quat & q1_in, double a)
{
  Quat q0 = QuatNormalize(q0_in);
  Quat q1 = QuatNormalize(q1_in);

  double dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
  if (dot < 0.0) {
    q1 = {-q1[0], -q1[1], -q1[2], -q1[3]};
    dot = -dot;
  }
  dot = std::clamp(dot, -1.0, 1.0);

  if (dot > 0.9995) {
    return QuatNormalize({
      q0[0] + (q1[0] - q0[0]) * a,
      q0[1] + (q1[1] - q0[1]) * a,
      q0[2] + (q1[2] - q0[2]) * a,
      q0[3] + (q1[3] - q0[3]) * a,
    });
  }

  const double theta0 = std::acos(dot);
  const double theta = theta0 * a;
  const double s0 = std::sin(theta0 - theta) / std::sin(theta0);
  const double s1 = std::sin(theta) / std::sin(theta0);
  return {
    q0[0] * s0 + q1[0] * s1,
    q0[1] * s0 + q1[1] * s1,
    q0[2] * s0 + q1[2] * s1,
    q0[3] * s0 + q1[3] * s1,
  };
}

bool AllFinite(const std::array<double, 3> & p, const Quat & q)
{
  for (double v : p) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  for (double v : q) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

rclcpp::QoS Px4InQos()
{
  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.keep_last(1);
  return qos;
}

rclcpp::QoS OdomQos()
{
  rclcpp::QoS qos(20);
  qos.reliable();
  qos.keep_last(20);
  return qos;
}

}  // namespace

class FastLioOdomBridge : public rclcpp::Node
{
public:
  FastLioOdomBridge()
  : Node("fastlio_odom_bridge")
  {
    pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", Px4InQos());
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", OdomQos(),
      std::bind(&FastLioOdomBridge::OnOdom, this, std::placeholders::_1));
    // EKF2 用 PX4 自己的时基做量测对齐；如果外部视觉的时间戳用墙钟，
    // 在 SITL（仿真时钟比墙钟慢）里会被当成"未来"数据而融合失败。
    px4_time_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", Px4InQos(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
        if (m->timestamp > 0) px4_time_us_ = m->timestamp;
      });

    publish_rate_hz_ = declare_parameter<double>("publish_rate", 30.0);
    // 插值必须用“已经收到的两帧”，因此一定会比最新帧再晚一段才能落在两帧之间。
    // 0 = 不插值，直接按固定频率重复最新帧（零附加滞后）；>0 = 额外延迟这么多秒后插值。
    interp_delay_sec_ = declare_parameter<double>("interp_delay_sec", 0.0);
    // 超过这个时间没有新帧就停止发布（保持原来的“没有数据就不发”的语义）
    hold_stop_sec_ = declare_parameter<double>("hold_stop_sec", 0.6);
    max_jump_m_ = declare_parameter<double>("max_jump", 1.5);
    max_speed_mps_ = declare_parameter<double>("max_speed", 8.0);

    const double rate = std::max(1.0, publish_rate_hz_);
    // 用节点时钟（use_sim_time=true 时为仿真时钟）驱动
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      std::bind(&FastLioOdomBridge::PublishTick, this));

    RCLCPP_INFO(
      get_logger(),
      "订阅 /Odometry -> 以 %.0f Hz 发布 /fmu/in/vehicle_visual_odometry"
      "（插值延迟 %.2fs, hold_stop=%.1fs）",
      rate, interp_delay_sec_, hold_stop_sec_);
  }

private:
  struct Sample
  {
    double t;                          // 采样时刻（仿真时间, s）
    std::array<double, 3> pos_ned;
    Quat q_ned;
    std::array<float, 3> pos_var;
    std::array<float, 3> ori_var;
  };

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double t =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const std::array<double, 3> pos_enu{p.x, p.y, p.z};
    const Quat q_flu_enu{q.w, q.x, q.y, q.z};

    if (!AllFinite(pos_enu, q_flu_enu)) {
      ++dropped_;
      return;
    }

    // ENU -> NED
    const std::array<double, 3> pos_ned{pos_enu[1], pos_enu[0], -pos_enu[2]};

    std::optional<double> dt;
    if (last_t_.has_value()) {
      const double dtt = t - *last_t_;
      if (dtt > 1e-4 && dtt < 1.0) dt = dtt;
    }

    // 过滤异常跳变，避免发散位姿灌进 EKF
    if (last_pos_ned_.has_value() && dt.has_value()) {
      const double dx = pos_ned[0] - (*last_pos_ned_)[0];
      const double dy = pos_ned[1] - (*last_pos_ned_)[1];
      const double dz = pos_ned[2] - (*last_pos_ned_)[2];
      const double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double speed = jump / *dt;
      if (jump > max_jump_m_ || speed > max_speed_mps_) {
        ++dropped_;
        if (dropped_ == 1 || dropped_ % 20 == 0) {
          RCLCPP_WARN(
            get_logger(),
            "丢弃异常视觉里程计: jump=%.2fm speed=%.1fm/s "
            "pos_ned=(%.2f,%.2f,%.2f) dropped=%lu",
            jump, speed, pos_ned[0], pos_ned[1], pos_ned[2],
            static_cast<unsigned long>(dropped_));
        }
        last_t_ = t;
        return;
      }
    }

    last_t_ = t;
    last_pos_ned_ = pos_ned;

    // 帧间隔（插值用）：EMA，限制在合理范围
    if (prev_stamp_.has_value()) {
      const double d = t - *prev_stamp_;
      if (d > 0.02 && d < 1.0) {
        interval_est_ += 0.2 * (d - interval_est_);
      }
    }
    prev_stamp_ = t;

    const auto variance = [](double c) -> float {
        return (std::isfinite(c) && c > 1e-6) ? static_cast<float>(c) : 2.5e-3f;
      };
    Sample s{};
    s.t = t;
    s.pos_ned = pos_ned;
    s.q_ned = QuatNormalize(FluEnuToFrdNed(q_flu_enu));
    const auto & cov = msg->pose.covariance;
    if (cov.size() >= 36) {
      s.pos_var = {variance(cov[7]), variance(cov[0]), variance(cov[14])};
      s.ori_var = {variance(cov[21]), variance(cov[28]), variance(cov[35])};
    } else {
      s.pos_var = {2.5e-3f, 2.5e-3f, 2.5e-3f};
      s.ori_var = {2.5e-3f, 2.5e-3f, 2.5e-3f};
    }

    if (!buf_.empty() && t < buf_.back().t) {
      buf_.clear();
    }
    buf_.push_back(s);
    while (buf_.size() > 20) {
      buf_.pop_front();
    }
    ++rx_frames_;
  }

  void PublishTick()
  {
    if (buf_.empty()) {
      return;
    }
    const double now_s = get_clock()->now().seconds();
    if (now_s <= 0.0) {
      return;
    }

    const double newest_age = now_s - buf_.back().t;
    if (newest_age > hold_stop_sec_) {
      ++stalled_;
      if (stalled_ == 1 || stalled_ % 200 == 0) {
        RCLCPP_WARN(
          get_logger(), "%.1fs 没有新的 /Odometry，暂停发布（累计 %lu 次）",
          hold_stop_sec_, static_cast<unsigned long>(stalled_));
      }
      return;
    }

    // interp_delay_sec_ = 0：不插值，直接发最新帧（零附加滞后）。
    // interp_delay_sec_ > 0：把查询点退到 now - delay，在最近两帧之间插值。
    const double tq = now_s - interp_delay_sec_;

    std::array<double, 3> pos{};
    Quat q_ned{};
    std::array<float, 3> pos_var{};
    std::array<float, 3> ori_var{};

    bool done = false;
    if (interp_delay_sec_ > 0.0) {
      for (size_t i = 0; i + 1 < buf_.size(); ++i) {
        const Sample & s0 = buf_[i];
        const Sample & s1 = buf_[i + 1];
        if (s0.t <= tq && tq <= s1.t) {
          const double span = s1.t - s0.t;
          if (span > 1e-6) {
            const double a = std::clamp((tq - s0.t) / span, 0.0, 1.0);
            for (int k = 0; k < 3; ++k) {
              pos[k] = s0.pos_ned[k] + (s1.pos_ned[k] - s0.pos_ned[k]) * a;
            }
            q_ned = QuatSlerp(s0.q_ned, s1.q_ned, a);
            for (int k = 0; k < 3; ++k) {
              pos_var[k] = std::max(s0.pos_var[k], s1.pos_var[k]);
              ori_var[k] = std::max(s0.ori_var[k], s1.ori_var[k]);
            }
            ++interp_frames_;
            done = true;
          }
          break;
        }
      }
    }

    if (!done) {
      const Sample & s = buf_.back();
      pos = s.pos_ned;
      q_ned = s.q_ned;
      pos_var = s.pos_var;
      ori_var = s.ori_var;
      ++hold_frames_;
    }

    const uint64_t now_us = px4_time_us_ > 0
        ? px4_time_us_
        : static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
    px4_msgs::msg::VehicleOdometry out;
    out.timestamp = now_us;
    out.timestamp_sample = now_us;
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.position[0] = static_cast<float>(pos[0]);
    out.position[1] = static_cast<float>(pos[1]);
    out.position[2] = static_cast<float>(pos[2]);
    out.q[0] = static_cast<float>(q_ned[0]);
    out.q[1] = static_cast<float>(q_ned[1]);
    out.q[2] = static_cast<float>(q_ned[2]);
    out.q[3] = static_cast<float>(q_ned[3]);

    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    // 不发送“位置差分速度”：FAST-LIO 的 z 只要抖 0.1 m，差分出来就是几 m/s，
    // 在 EKF2_EV_CTRL 允许速度融合时会直接把高度估计带飞。
    out.velocity[0] = std::numeric_limits<float>::quiet_NaN();
    out.velocity[1] = std::numeric_limits<float>::quiet_NaN();
    out.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    out.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
    out.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
    out.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

    for (int k = 0; k < 3; ++k) {
      out.position_variance[k] = pos_var[k];
      out.orientation_variance[k] = ori_var[k];
    }
    out.velocity_variance[0] = std::numeric_limits<float>::quiet_NaN();
    out.reset_counter = 0;
    out.quality = 0;

    pub_->publish(out);
    ++frames_;
    ++frames_in_window_;

    const rclcpp::Time stamp = now();
    if (last_rate_t_.nanoseconds() == 0) last_rate_t_ = stamp;
    const double win = (stamp - last_rate_t_).seconds();
    if (win >= 3.0) {
      RCLCPP_INFO(
        get_logger(),
        "视觉里程计 %.1f Hz（收到 %.1f Hz，插值 %lu / 保持 %lu）帧间隔 %.2fs NED=(%.2f,%.2f,%.2f)",
        frames_in_window_ / win, rx_frames_ / win,
        static_cast<unsigned long>(interp_frames_),
        static_cast<unsigned long>(hold_frames_),
        interval_est_, out.position[0], out.position[1], out.position[2]);
      frames_in_window_ = 0;
      rx_frames_ = 0;
      interp_frames_ = 0;
      hold_frames_ = 0;
      last_rate_t_ = stamp;
    }
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_time_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t px4_time_us_{0};

  std::deque<Sample> buf_;
  std::optional<double> last_t_;
  std::optional<double> prev_stamp_;
  std::optional<std::array<double, 3>> last_pos_ned_;
  double interval_est_{0.1};

  uint64_t frames_{0};
  uint64_t rx_frames_{0};
  uint64_t frames_in_window_{0};
  uint64_t interp_frames_{0};
  uint64_t hold_frames_{0};
  uint64_t stalled_{0};
  uint64_t dropped_{0};
  rclcpp::Time last_rate_t_{0, 0, RCL_ROS_TIME};

  double publish_rate_hz_{20.0};
  double hold_stop_sec_{0.6};
  double interp_delay_sec_{0.0};
  double max_jump_m_{1.5};
  double max_speed_mps_{8.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
