// 对比 FAST-LIO2 里程计与飞控本地位置，判断两者是否基本同步：
//   /Odometry                        (nav_msgs/Odometry, ENU + FLU)
//   /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition, NED + FRD)
//
// 坐标变换与 fastlio_odom_bridge 完全一致（ENU->NED、FLU->FRD），
// 这样打印出来的差值才是“送进 EKF 的那一份”和 EKF 输出之间的真实差。
// 同时打印两条消息的接收时延和到达时差，用来区分“轨迹不一致”和“时间不同步”。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

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
  static const Quat q_flu_to_frd{0.0, 1.0, 0.0, 0.0};
  static const Quat q_enu_to_ned{0.0, 0.70711, 0.70711, 0.0};
  return QuatMul(q_enu_to_ned, QuatMul(q_flu_enu, QuatConj(q_flu_to_frd)));
}

// NED 下的偏航角（与 PX4 heading 同一约定）
double YawFromQuat(const Quat & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double WrapDeg(double deg)
{
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

constexpr double kRadToDeg = 57.29577951308232;

rclcpp::QoS OdomQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
}

rclcpp::QoS Px4OutQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

}  // namespace

class OdomLpCompare : public rclcpp::Node
{
public:
  OdomLpCompare()
  : Node("odom_lp_compare")
  {
    rate_hz_ = declare_parameter<double>("rate_hz", 1.0);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 1.5);
    const std::string lio_topic = declare_parameter<std::string>("lio_topic", "/Odometry");
    const std::string lp_topic =
      declare_parameter<std::string>(
      "lp_topic", "/fmu/out/vehicle_local_position_v1");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_topic, OdomQos(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        lio_ = *msg;
        lio_recv_ = now();
        ++lio_count_;
      });

    lp_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      lp_topic, Px4OutQos(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        lp_ = *msg;
        lp_recv_ = now();
        ++lp_count_;
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { Tick(); });

    rate_window_start_ = now();
    RCLCPP_INFO(
      get_logger(), "对比 %s 与 %s，%.1f Hz（首次比对时将记录初始偏置）",
      lio_topic.c_str(), lp_topic.c_str(), rate_hz_);
  }

private:
  void Tick()
  {
    const rclcpp::Time t = now();
    const double win = (t - rate_window_start_).seconds();
    const double lio_hz = win > 1e-6 ? static_cast<double>(lio_count_) / win : 0.0;
    const double lp_hz = win > 1e-6 ? static_cast<double>(lp_count_) / win : 0.0;
    lio_count_ = 0;
    lp_count_ = 0;
    rate_window_start_ = t;

    if (!lio_recv_.has_value() || !lp_recv_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待数据：/Odometry=%s  local_position=%s",
        lio_recv_.has_value() ? "已收到" : "无",
        lp_recv_.has_value() ? "已收到" : "无");
      return;
    }

    const double lio_age = (t - *lio_recv_).seconds();
    const double lp_age = (t - *lp_recv_).seconds();
    const double recv_gap = (*lp_recv_ - *lio_recv_).seconds();

    // LIO: ENU + FLU -> NED（与 fastlio_odom_bridge 同一变换）
    const auto & p = lio_.pose.pose.position;
    const auto & q = lio_.pose.pose.orientation;
    const std::array<double, 3> lio_ned{p.y, p.x, -p.z};
    const Quat q_ned = FluEnuToFrdNed({q.w, q.x, q.y, q.z});

    // LIO 自身速度（相邻两帧位移/时间），用于判断“位置差”随时间戳延迟放大多少
    double lio_speed = std::nan("");
    if (prev_lio_ned_.has_value() && prev_lio_recv_.has_value()) {
      const double dt = (*lio_recv_ - *prev_lio_recv_).seconds();
      if (dt > 1e-4) {
        const double dx = lio_ned[0] - (*prev_lio_ned_)[0];
        const double dy = lio_ned[1] - (*prev_lio_ned_)[1];
        const double dz = lio_ned[2] - (*prev_lio_ned_)[2];
        lio_speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
      }
    }
    prev_lio_ned_ = lio_ned;
    prev_lio_recv_ = *lio_recv_;

    const bool stale = lio_age > stale_timeout_ || lp_age > stale_timeout_;
    if (stale) {
      RCLCPP_WARN(
        get_logger(),
        "数据超时：LIO 时延 %.2fs（%.1f Hz），LP 时延 %.2fs（%.1f Hz），此时差值不可信",
        lio_age, lio_hz, lp_age, lp_hz);
      return;
    }

    const double lio_yaw = YawFromQuat(q_ned) * kRadToDeg;
    const double lp_yaw = static_cast<double>(lp_.heading) * kRadToDeg;
    const double yaw_diff = WrapDeg(lio_yaw - lp_yaw);

    const std::array<double, 3> delta{
      lio_ned[0] - static_cast<double>(lp_.x),
      lio_ned[1] - static_cast<double>(lp_.y),
      lio_ned[2] - static_cast<double>(lp_.z)};
    const double delta_norm =
      std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);

    // EKF 的局部原点与 LIO 原点允许存在常量偏置，扣掉首帧偏置后
    // 剩下的才是“轨迹是否跟随”。
    if (!offset_.has_value()) {
      offset_ = delta;
      RCLCPP_INFO(
        get_logger(), "记录初始偏置：NED (%.3f, %.3f, %.3f) m",
        delta[0], delta[1], delta[2]);
    }
    const double drift[3] = {
      delta[0] - (*offset_)[0],
      delta[1] - (*offset_)[1],
      delta[2] - (*offset_)[2]};
    const double drift_norm = std::sqrt(
      drift[0] * drift[0] + drift[1] * drift[1] + drift[2] * drift[2]);

    char buf[512];
    std::snprintf(
      buf, sizeof(buf),
      "LIO_NED=(%+.3f,%+.3f,%+.3f) LP=(%+.3f,%+.3f,%+.3f) "
      "Δ=(%+.3f,%+.3f,%+.3f) |Δ|=%.3fm "
      "去初偏置漂移=(%+.3f,%+.3f,%+.3f) |漂移|=%.3fm "
      "yaw LIO=%+.1f° LP=%+.1f° Δyaw=%+.1f° "
      "| 频率 %.1f/%.1f Hz 时延 %.2f/%.2f s 到达差 %+.3f s |v|=%.2f m/s%s",
      lio_ned[0], lio_ned[1], lio_ned[2],
      static_cast<double>(lp_.x), static_cast<double>(lp_.y),
      static_cast<double>(lp_.z),
      delta[0], delta[1], delta[2], delta_norm,
      drift[0], drift[1], drift[2], drift_norm,
      lio_yaw, lp_yaw, yaw_diff,
      lio_hz, lp_hz, lio_age, lp_age, recv_gap,
      lio_speed,
      (lp_.xy_valid && lp_.z_valid) ? "" : " [LP 位置无效标志]");

    if (delta_norm > 1.0 || drift_norm > 1.0) {
      RCLCPP_WARN(get_logger(), "%s", buf);
    } else {
      RCLCPP_INFO(get_logger(), "%s", buf);
    }
  }

  double rate_hz_{1.0};
  double stale_timeout_{1.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lp_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry lio_;
  px4_msgs::msg::VehicleLocalPosition lp_;
  std::optional<rclcpp::Time> lio_recv_;
  std::optional<rclcpp::Time> lp_recv_;
  std::optional<std::array<double, 3>> prev_lio_ned_;
  std::optional<rclcpp::Time> prev_lio_recv_;
  std::optional<std::array<double, 3>> offset_;

  rclcpp::Time rate_window_start_{0, 0, RCL_ROS_TIME};
  uint64_t lio_count_{0};
  uint64_t lp_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomLpCompare>());
  rclcpp::shutdown();
  return 0;
}
