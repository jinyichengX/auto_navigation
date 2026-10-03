#!/usr/bin/env python3
"""把 FAST-LIO 的 /Odometry 转成 PX4 的 /fmu/in/vehicle_visual_odometry。

输入：nav_msgs/Odometry（ENU 世界系 + FLU 机体系，只有位姿）
输出：px4_msgs/VehicleOdometry（NED 世界系 + FRD 机体系）

转换和 PX4 GZBridge::odometryCallback / rotateQuaternion 对齐：
1. 位置 ENU -> NED: (x_n,y_e,z_d) = (y_e, x_n, -z_u)
2. 姿态 FLU->ENU 转成 FRD->NED:
   q_nb = q_ENU_to_NED * q_flu_enu * q_FLU_to_FRD^{-1}
3. FAST-LIO 默认不发速度，这里用相邻两帧位置差分估 ENU 速度，再转成 NED。
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def _quat_mul(q1, q2):
    """Hamilton 乘积，q = [w, x, y, z]。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _quat_conj(q):
    return [q[0], -q[1], -q[2], -q[3]]


def _flu_enu_to_frd_ned(q_flu_enu):
    # 和 GZBridge::rotateQuaternion 一致
    q_flu_to_frd = [0.0, 1.0, 0.0, 0.0]
    q_enu_to_ned = [0.0, 0.70711, 0.70711, 0.0]
    return _quat_mul(q_enu_to_ned, _quat_mul(q_flu_enu, _quat_conj(q_flu_to_frd)))


class FastLioOdomBridge(Node):
    def __init__(self):
        super().__init__('fastlio_odom_bridge')

        qos_px4_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_px4_in,
        )

        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos_odom)

        self._last_t = None
        self._last_pos_enu = None
        self._last_pub_pos_ned = None
        self._frames = 0
        self._dropped = 0
        # 井下巡航速度通常 < 2m/s；跳变过大基本是 FAST-LIO 发散/重置
        self._max_jump_m = 2.5
        self._max_speed_mps = 8.0
        self.get_logger().info('订阅 /Odometry -> 发布 /fmu/in/vehicle_visual_odometry')

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pos_enu = (float(p.x), float(p.y), float(p.z))
        q_flu_enu = [float(q.w), float(q.x), float(q.y), float(q.z)]

        if not all(math.isfinite(v) for v in pos_enu + tuple(q_flu_enu)):
            self._dropped += 1
            return

        # ENU -> NED
        pos_ned = (pos_enu[1], pos_enu[0], -pos_enu[2])

        # 没有速度时，用相邻位置差分估 ENU 世界速度
        vel_enu = (math.nan, math.nan, math.nan)
        dt = None
        if self._last_t is not None and self._last_pos_enu is not None:
            dt = t - self._last_t
            if 1e-4 < dt < 1.0:
                vel_enu = (
                    (pos_enu[0] - self._last_pos_enu[0]) / dt,
                    (pos_enu[1] - self._last_pos_enu[1]) / dt,
                    (pos_enu[2] - self._last_pos_enu[2]) / dt,
                )

        # 过滤异常跳变：避免把发散位姿灌进 EKF，一起飞就倾覆
        if self._last_pub_pos_ned is not None and dt is not None and dt > 1e-4:
            jump = math.sqrt(
                (pos_ned[0] - self._last_pub_pos_ned[0]) ** 2
                + (pos_ned[1] - self._last_pub_pos_ned[1]) ** 2
                + (pos_ned[2] - self._last_pub_pos_ned[2]) ** 2
            )
            speed = jump / dt
            if jump > self._max_jump_m or speed > self._max_speed_mps:
                self._dropped += 1
                if self._dropped == 1 or self._dropped % 20 == 0:
                    self.get_logger().warn(
                        f'丢弃异常视觉里程计: jump={jump:.2f}m speed={speed:.1f}m/s '
                        f'pos_ned=({pos_ned[0]:.2f},{pos_ned[1]:.2f},{pos_ned[2]:.2f}) '
                        f'dropped={self._dropped}'
                    )
                # 不更新 last_pub；允许后续重新对齐。但更新 last_t/pos 以免差分爆炸
                self._last_t = t
                self._last_pos_enu = pos_enu
                return

        self._last_t = t
        self._last_pos_enu = pos_enu
        self._last_pub_pos_ned = pos_ned

        now_us = int(self.get_clock().now().nanoseconds / 1000)
        out = VehicleOdometry()
        out.timestamp = now_us
        out.timestamp_sample = now_us

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position[0] = float(pos_ned[0])
        out.position[1] = float(pos_ned[1])
        out.position[2] = float(pos_ned[2])

        q_nb = _flu_enu_to_frd_ned(q_flu_enu)
        out.q[0] = float(q_nb[0])
        out.q[1] = float(q_nb[1])
        out.q[2] = float(q_nb[2])
        out.q[3] = float(q_nb[3])

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        if math.isfinite(vel_enu[0]):
            out.velocity[0] = float(vel_enu[1])   # north = enu.y
            out.velocity[1] = float(vel_enu[0])   # east  = enu.x
            out.velocity[2] = float(-vel_enu[2])  # down  = -enu.z
        else:
            out.velocity[0] = math.nan
            out.velocity[1] = math.nan
            out.velocity[2] = math.nan

        out.angular_velocity[0] = math.nan
        out.angular_velocity[1] = math.nan
        out.angular_velocity[2] = math.nan

        # 协方差：有就尽量用，没有就标未知
        cov = msg.pose.covariance
        if len(cov) >= 36 and math.isfinite(cov[0]):
            out.position_variance[0] = float(cov[7])   # y
            out.position_variance[1] = float(cov[0])   # x
            out.position_variance[2] = float(cov[14])  # z
            out.orientation_variance[0] = float(cov[21])
            out.orientation_variance[1] = float(cov[28])
            out.orientation_variance[2] = float(cov[35])
        else:
            out.position_variance[0] = math.nan
            out.orientation_variance[0] = math.nan

        out.velocity_variance[0] = math.nan
        out.reset_counter = 0
        out.quality = 0

        self.pub.publish(out)
        self._frames += 1
        if self._frames == 1 or self._frames % 50 == 0:
            self.get_logger().info(
                f'已转发 {self._frames} 帧: '
                f'NED pos=({out.position[0]:.2f},{out.position[1]:.2f},{out.position[2]:.2f})'
            )


def main():
    rclpy.init()
    node = FastLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
