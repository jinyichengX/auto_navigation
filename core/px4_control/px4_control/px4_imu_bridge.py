#!/usr/bin/env python3
"""把 PX4 飞控 IMU 转成 ROS 2 的 sensor_msgs/Imu，给 FAST-LIO 用。

订阅：/fmu/out/sensor_combined
发布：/livox/imu

说明：
1. PX4 的加速度/角速度是 FRD（前-右-下），ROS/FAST-LIO 习惯 FLU（前-左-上）。
   这里按常见约定做 FRD→FLU：y、z 取反。
2. 时间戳直接用系统时间，和 gz_lidar_bridge 保持一致，方便后面时间同步。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined


class Px4ImuBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        self.pub = self.create_publisher(Imu, '/livox/imu', 50)
        self.frames = 0

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.on_imu,
            qos_out,
        )
        self.get_logger().info('订阅 /fmu/out/sensor_combined -> 发布 /livox/imu')

    def on_imu(self, msg: SensorCombined):
        out = Imu()
        # 和点云桥一样用系统时间，避免和 Gazebo sim time / PX4 boot time 混用
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        # FRD -> FLU
        out.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        out.linear_acceleration.y = float(-msg.accelerometer_m_s2[1])
        out.linear_acceleration.z = float(-msg.accelerometer_m_s2[2])

        out.angular_velocity.x = float(msg.gyro_rad[0])
        out.angular_velocity.y = float(-msg.gyro_rad[1])
        out.angular_velocity.z = float(-msg.gyro_rad[2])

        # 没有姿态，协方差先标成未知
        out.orientation_covariance[0] = -1.0
        out.angular_velocity_covariance[0] = 0.01
        out.linear_acceleration_covariance[0] = 0.1

        self.pub.publish(out)
        self.frames += 1
        if self.frames == 1 or self.frames % 200 == 0:
            self.get_logger().info(f'已转发 IMU {self.frames} 帧')


def main():
    rclpy.init()
    node = Px4ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
