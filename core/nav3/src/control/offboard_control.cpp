#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

class OffboardControl:public rclcpp::Node
{
public:
  OffboardControl()
  : Node("offboard_control_node")
  {
    auto_offboard_ = this->declare_parameter<bool>("auto_offboard", true);
    auto_arm_ = this->declare_parameter<bool>("auto_arm", true);
    // use_target_yaw_ = this->declare_parameter<bool>("use_target_yaw", false);

    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", px4Qos());
            
    setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", px4Qos());

    command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", px4Qos());

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { TargetPoseCallback(msg); });

    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4Qos(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                local_pos_ = *msg;
                has_local_pos_ = true;
            });

    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4Qos(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { VehicleStatusCallback(msg); });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),   // 50 Hz
           [this]() { TimerCallback(); });
           
    RCLCPP_INFO(
        get_logger(), "向 %s 发布 ENU 目标位姿即可开始控制",
        target_sub_->get_topic_name());
  }

private:
  static rclcpp::QoS px4Qos()
  {
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  }

  void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    has_status_ = true;
    // check fmu status
    if (msg->nav_state == last_nav_state_ && msg->arming_state == last_arming_state_) return;
    last_nav_state_ = msg->nav_state;
    last_arming_state_ = msg->arming_state;

    if (status_.pre_flight_checks_pass != true) {
      RCLCPP_INFO(
          get_logger(), "飞控状态: nav_state=%u, arming_state=%u (OFFBOARD=%u, ARMED=%u)",
          msg->nav_state, msg->arming_state,
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
          px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }
  }

  void TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // // ENU -> NED: 北=ENU.y, 东=ENU.x, 地=-ENU.z
    // setpoint_[0] = static_cast<float>(msg->pose.position.y);
    // setpoint_[1] = static_cast<float>(msg->pose.position.x);
    // setpoint_[2] = static_cast<float>(-msg->pose.position.z);

    // // ENU(FLU) 偏航角 -> NED 航向角 (以正北为 0, 顺时针为正)
    // const auto & q = msg->pose.orientation;
    // const double yaw_enu = std::atan2(
    //     2.0 * (q.w * q.z + q.x * q.y),
    //     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // setpoint_yaw_ = normalizeAngle(kPi / 2.0 - yaw_enu);
  }

  void TimerCallback(void)
  {
    if (!has_local_pos_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "等待 PX4 本地位置, 暂不发送 setpoint");
      return;
    }

    PublishOffboardControlMode();
    PublishSetpoint();

    if (auto_offboard_ && !offboard_requested_ && counter_ > 50) {
        SetOffboardMode();
        offboard_requested_ = true;
    }
    if (auto_arm_ && offboard_requested_ && !arm_requested_ && counter_ > 100) {
      if (!has_status_) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000, "等待飞控状态, 暂不解锁");
      } else if (!status_.pre_flight_checks_pass) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "飞控预检查未通过, 暂不解锁 (详见 QGC 的 Overall Status)");
      } else {
          Arm();
          arm_requested_ = true;
      }
    }

    ++counter_;
  }

  void PublishOffboardControlMode(void)
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = NowUs();
    msg.position = true;   // 使用位置控制
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    // msg.actuator = false;
    offboard_mode_pub_->publish(msg);
  }

  void PublishSetpoint(void)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = NowUs();
    // msg.position = {setpoint_[0], setpoint_[1], setpoint_[2]};
    msg.position = {0, 0, -1.5};
    // 未参与控制的维度置 NaN, 由 PX4 自行保持
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};
    // msg.yaw = use_target_yaw_ ? static_cast<float>(setpoint_yaw_) : nan;
    msg.yaw = 3.14;
    msg.yawspeed = nan;
    setpoint_pub_->publish(msg);
  }

  void SetOffboardMode(void)
  {
    // param1 = 1 (CUSTOM_MODE_ENABLED), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "已发送切换 OFFBOARD 模式指令");
  }

  void Arm(void)
  {
    PublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM));
    RCLCPP_INFO(get_logger(), "已发送解锁指令");
  }

  void PublishVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = NowUs();
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  uint64_t NowUs(void) const{return static_cast<uint64_t>(this->now().nanoseconds() / 1000);}
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  px4_msgs::msg::VehicleLocalPosition local_pos_{};
  px4_msgs::msg::VehicleStatus status_{};
  bool has_local_pos_ = false;
  bool has_status_ = false;

  uint8_t last_nav_state_ = 255;
  uint8_t last_arming_state_ = 255;
  uint32_t counter_ = 0;
  bool auto_offboard_;
  bool auto_arm_;
  bool offboard_requested_ = false;
  bool arm_requested_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
