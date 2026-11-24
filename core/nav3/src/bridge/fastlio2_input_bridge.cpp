// 合并原 px4_imu_bridge.py + gz_lidar_bridge.py：
// 1) PX4 SensorCombined -> /livox/imu
// 2) Gazebo Mid-360 PointCloudPacked -> /livox/lidar
// 3) 发布静态 TF：
//    - body -> base_link      （单位变换，桥接 FAST-LIO 与机体）
//    - base_link -> livox_link（点云传感器原点，含连杆与传感器局部偏移）

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace
{

constexpr const char * kLivoxFrame = "livox_link";
constexpr const char * kBaseFrame = "base_link";
constexpr const char * kBodyFrame = "body";
// PointCloudPacked 以传感器坐标输出：link 相对 base 为 0.28 m，sensor 再沿
// z 轴偏移 0.035 m。主雷达水平安装（0° 俯仰），多看竖直结构给 FAST-LIO
// 提供稳定特征。
constexpr double kLivoxTiltRad = 0.0;           // 0°，水平安装
constexpr double kLivoxMountX = 0.0;            // 0.035 * sin(0°)
constexpr double kLivoxMountZ = 0.315;          // 0.28 + 0.035 * cos(0°)

// Gazebo Field.datatype -> sensor_msgs/PointField.datatype（整体差 1）
uint8_t RosDatatype(uint32_t gz_value)
{
  switch (gz_value) {
    case 0: return sensor_msgs::msg::PointField::INT8;
    case 1: return sensor_msgs::msg::PointField::UINT8;
    case 2: return sensor_msgs::msg::PointField::INT16;
    case 3: return sensor_msgs::msg::PointField::UINT16;
    case 4: return sensor_msgs::msg::PointField::INT32;
    case 5: return sensor_msgs::msg::PointField::UINT32;
    case 6: return sensor_msgs::msg::PointField::FLOAT32;
    case 7: return sensor_msgs::msg::PointField::FLOAT64;
    default: return sensor_msgs::msg::PointField::FLOAT32;
  }
}

rclcpp::QoS Px4OutQos()
{
  rclcpp::QoS qos(20);
  qos.best_effort();
  qos.keep_last(20);
  return qos;
}

}  // namespace

class FastLio2InputBridge : public rclcpp::Node
{
public:
  FastLio2InputBridge()
  : Node("fastlio2_input_bridge")
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 50);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 5);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PublishMountTf();

    // 必须与 fast_lio 的 preprocess.scan_rate 一致（1/scan_rate）。
    scan_period_sec_ = declare_parameter<double>("scan_period", 0.1);

    imu_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined",
      Px4OutQos(),
      std::bind(&FastLio2InputBridge::OnImu, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "订阅 /fmu/out/sensor_combined -> 发布 /livox/imu");

    // 雷达话题要等 Gazebo 完全加载并 spawn 完模型才会出现；启动阶段可能还没就绪，
    // 所以首次失败后周期性重试，直到订阅成功（否则 FAST-LIO 永远收不到点云）。
    if (!SetupLidar()) {
      RCLCPP_WARN(get_logger(), "尚未找到 Mid-360 点云话题，将每 2s 重试");
      lidar_retry_timer_ = create_wall_timer(
          std::chrono::seconds(2), [this]() {
            if (SetupLidar()) {
              RCLCPP_INFO(get_logger(), "已成功订阅 Mid-360 点云话题");
              lidar_retry_timer_->cancel();
            }
          });
    }

    // 把 Gazebo 仿真时钟桥接成 ROS /clock，供 use_sim_time 节点使用：
    // 仿真 RTF≈0.37，若用墙钟，FAST-LIO 的 IMU 预积分 dt 会被放大 ~2.7 倍，
    // 正是"静止没事、一动就发散"的根因。
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    gz_node_.Subscribe("/clock", &FastLio2InputBridge::OnClock, this);
  }

  void OnClock(const gz::msgs::Clock &msg) {
    const int64_t ns =
        static_cast<int64_t>(msg.sim().sec()) * 1000000000LL +
        static_cast<int64_t>(msg.sim().nsec());
    sim_time_ns_.store(ns, std::memory_order_relaxed);

    rosgraph_msgs::msg::Clock out;
    out.clock.sec = static_cast<int32_t>(msg.sim().sec());
    out.clock.nanosec = static_cast<uint32_t>(msg.sim().nsec());
    clock_pub_->publish(out);
  }

  ~FastLio2InputBridge() override
  {
    stopping_.store(true);
    if (!gz_topic_.empty()) {
      gz_node_.Unsubscribe(gz_topic_);
    }
  }

private:
  void PublishMountTf()
  {
    const auto stamp = get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(2);

    // FAST-LIO 的 body 与机体 IMU 原点重合：单位变换即可
    geometry_msgs::msg::TransformStamped body_to_base;
    body_to_base.header.stamp = stamp;
    body_to_base.header.frame_id = kBodyFrame;
    body_to_base.child_frame_id = kBaseFrame;
    body_to_base.transform.rotation.w = 1.0;
    tfs.push_back(body_to_base);

    // 雷达相对机体安装外参
    geometry_msgs::msg::TransformStamped base_to_livox;
    base_to_livox.header.stamp = stamp;
    base_to_livox.header.frame_id = kBaseFrame;
    base_to_livox.child_frame_id = kLivoxFrame;
    base_to_livox.transform.translation.x = kLivoxMountX;
    base_to_livox.transform.translation.z = kLivoxMountZ;
    // 绕 y 轴下俯 20°：q = (w, x, y, z) = (cos(10°), 0, sin(10°), 0)
    base_to_livox.transform.rotation.w = std::cos(0.5 * kLivoxTiltRad);
    base_to_livox.transform.rotation.y = std::sin(0.5 * kLivoxTiltRad);
    tfs.push_back(base_to_livox);

    for (const auto & tf : tfs) {
      tf_broadcaster_->sendTransform(tf);
    }
    RCLCPP_INFO(
      get_logger(),
      "静态 TF: %s -> %s (I), %s -> %s (x=%.3f z=%.3f, 下俯 %.1f°)",
      kBodyFrame, kBaseFrame, kBaseFrame, kLivoxFrame, kLivoxMountX,
      kLivoxMountZ, kLivoxTiltRad * 180.0 / M_PI);
  }

  bool SetupLidar()
  {
    if (lidar_subscribed_) return true;
    std::vector<std::string> topics;
    gz_node_.TopicList(topics);
    for (const auto & topic : topics) {
      // 必须匹配到雷达本体（sensor/livox_mid360_lidar），不能只匹配模型名。
      if (topic.find("livox_mid360_lidar") != std::string::npos &&
        topic.size() >= 7 &&
        topic.compare(topic.size() - 7, 7, "/points") == 0)
      {
        gz_topic_ = topic;
        break;
      }
    }
    if (gz_topic_.empty()) {
      return false;
    }

    const bool ok = gz_node_.Subscribe(
      gz_topic_,
      &FastLio2InputBridge::OnCloud,
      this);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "订阅 Gazebo 话题失败: %s", gz_topic_.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Gazebo 话题: %s", gz_topic_.c_str());
    RCLCPP_INFO(get_logger(), "ROS 2 话题: /livox/lidar");
    lidar_subscribed_ = true;
    return true;
  }

  void OnImu(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out;
    // 统一用 Gazebo 仿真时钟打戳：IMU 和点云同一时基，FAST-LIO 预积分 dt
    // 与仿真物理时间一致。仿真时钟还没到时先不发。
    const int64_t sim_ns = sim_time_ns_.load(std::memory_order_relaxed);
    if (sim_ns <= 0) return;
    out.header.stamp.sec = static_cast<int32_t>(sim_ns / 1000000000LL);
    out.header.stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000LL);

    out.header.frame_id = kBaseFrame;

    // FRD -> FLU
    out.linear_acceleration.x = msg->accelerometer_m_s2[0];
    out.linear_acceleration.y = -msg->accelerometer_m_s2[1];
    out.linear_acceleration.z = -msg->accelerometer_m_s2[2];

    out.angular_velocity.x = msg->gyro_rad[0];
    out.angular_velocity.y = -msg->gyro_rad[1];
    out.angular_velocity.z = -msg->gyro_rad[2];

    out.orientation_covariance[0] = -1.0;
    out.angular_velocity_covariance[0] = 0.01;
    out.linear_acceleration_covariance[0] = 0.1;

    imu_pub_->publish(out);
    ++imu_frames_;
    if (imu_frames_ == 1 || imu_frames_ % 200 == 0) {
      RCLCPP_INFO(get_logger(), "已转发 IMU %lu 帧", static_cast<unsigned long>(imu_frames_));
    }
  }

  void OnCloud(const gz::msgs::PointCloudPacked & msg)
  {
    if (stopping_.load()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    // 与 IMU 用同一套仿真时钟，避免 RTF 不为 1 时 dt 被放大。
    const int64_t sim_ns = sim_time_ns_.load(std::memory_order_relaxed);
    if (sim_ns <= 0) return;
    // Gazebo 的 gpu_lidar 是瞬时快照，msg.header().stamp() 就是这一帧的采样时刻。
    // 注意：default_handler 给每个点的 curvature 都是 0，FAST-LIO 的
    // sync_packages 会走 else 分支，令 lidar_end_time = lidar_beg_time（帧头时间）。
    // 也就是说 FAST-LIO 认为这一帧的位姿时刻就是帧头，不会再额外 +scan_period。
    // 所以这里必须原样使用采样时刻，任何偏移都会让位姿与点云错开一个时间量。
    const int64_t gz_ns =
        static_cast<int64_t>(msg.header().stamp().sec()) * 1000000000LL +
        static_cast<int64_t>(msg.header().stamp().nsec());
    const int64_t stamp_ns = (gz_ns > 0) ? gz_ns : sim_ns;
    out.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    out.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    have_imu_stamp_ = true;
    out.header.frame_id = kLivoxFrame;
    out.height = msg.height();
    out.width = msg.width();
    out.is_bigendian = msg.is_bigendian();
    out.point_step = msg.point_step();
    out.row_step = msg.row_step();
    out.is_dense = msg.is_dense();

    const auto & data = msg.data();
    out.data.assign(data.begin(), data.end());

    out.fields.clear();
    out.fields.reserve(static_cast<size_t>(msg.field_size()));
    for (int i = 0; i < msg.field_size(); ++i) {
      const auto & f = msg.field(i);
      sensor_msgs::msg::PointField pf;
      pf.name = f.name();
      pf.offset = f.offset();
      pf.datatype = RosDatatype(f.datatype());
      pf.count = f.count();
      out.fields.push_back(pf);
    }

    {
      std::lock_guard<std::mutex> lock(cloud_pub_mutex_);
      if (stopping_.load()) {
        return;
      }
      cloud_pub_->publish(out);
    }

    ++cloud_frames_;
    if (cloud_frames_ % 50 == 1) {
      std::string fields;
      int zoff = -1;
      const uint32_t step = msg.point_step();
      for (int i = 0; i < msg.field_size(); ++i) {
        if (i > 0) {
          fields += ",";
        }
        fields += msg.field(i).name();
        if (msg.field(i).name() == "z") zoff = msg.field(i).offset();
      }
      // 传感器系下的 z 范围：用来确认安装俯仰方向（下俯时最低点会明显更深）。
      double zmin = std::numeric_limits<double>::infinity();
      double zmax = -std::numeric_limits<double>::infinity();
      if (zoff >= 0) {
        const size_t n = data.size() / step;
        for (size_t k = 0; k < n; ++k) {
          float zv = 0.0f;
          std::memcpy(&zv, data.data() + k * step + zoff, sizeof(float));
          if (!std::isfinite(zv)) continue;
          zmin = std::min(zmin, static_cast<double>(zv));
          zmax = std::max(zmax, static_cast<double>(zv));
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "已转发 %lu 帧 | 每帧 %u 点 | z∈[%.1f, %.1f] | %u 字节/点 | 字段 [%s] | frame_id=%s",
        static_cast<unsigned long>(cloud_frames_),
        msg.width() * msg.height(),
        zmin, zmax,
        msg.point_step(),
        fields.c_str(),
        out.header.frame_id.c_str());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  gz::transport::Node gz_node_;
  std::string gz_topic_;
  std::mutex cloud_pub_mutex_;
  std::atomic<bool> stopping_{false};
  std::atomic<int64_t> sim_time_ns_{0};

  bool have_imu_offset_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  bool have_imu_stamp_{false};
  double imu_time_offset_{0.0};
  // = 1 / fast_lio preprocess.scan_rate
  double scan_period_sec_{0.1};
  uint64_t imu_frames_{0};
  uint64_t cloud_frames_{0};
  bool lidar_subscribed_{false};
  rclcpp::TimerBase::SharedPtr lidar_retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLio2InputBridge>();
  // 点云回调（几十万字节的拷贝）与 IMU 回调必须并行，否则 IMU 时间戳会被拖后。
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
