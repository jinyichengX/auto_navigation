#!/usr/bin/env python3
"""键盘遥控（与 offboard_control 配合工作）

启动清单里的 offboard_control 负责：切 OFFBOARD、解锁、并把 /target_pose
换算成飞控位置指令。本节点只做一件事——把按键换算成 /target_pose 目标点，
不改飞控模式、不解锁、不直接发飞控指令，因此两者不会抢同一个输入口。

按键：
  w / s   机头前进 / 后退
  a / d   机头左移 / 右移
  q / e   左旋 / 右旋（偏航）
  r / f   上升 / 下降
  空格    立刻停住（含偏航）

按住不放会持续飞（靠键盘自动重复），松开约 0.5 秒后自动停下。
目标点按当前机头朝向积分，水平速度不再固定世界轴。
"""
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

NAN = float('nan')
PUB_HZ = 20
TICK = 1.0 / PUB_HZ

SPEED = 2.0           # m/s，按键时给的目标速度（机体系）
YAW_RATE = 0.8        # rad/s，q/e 偏航角速度
DECAY = 0.88          # 每个 tick 衰减系数：松手后约 0.5 秒停住

BANNER = """
==================== 键盘遥控 ====================
  w / s   机头前进 / 后退    a / d   机头左移 / 右移
  q / e   左旋 / 右旋        r / f   上升 / 下降
  空格    立刻停住
本节点只发 /target_pose，切模式/解锁由 offboard_control 负责。
按住不放持续飞，松开自动停。结束按 Ctrl+C。
==================================================
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # offboard_control 订阅 /target_pose 并负责真正控制飞控
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self.on_local_pos, qos_out)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, qos_out)
        self.create_subscription(Odometry, '/Odometry', self.on_flio_odom, 10)

        self.n = 0
        self.body_vel = [0.0, 0.0, 0.0]  # 机体系速度指令：前/右/下（FRD）
        self.yawspeed = 0.0             # NED 偏航角速度：正=右转

        # 当前位姿（PX4 local，NED）
        self.x = NAN
        self.y = NAN
        self.z = NAN
        self.heading = 0.0
        self.heading_valid = False
        self.have_pos = False
        self.arming_state = -1
        self.nav_state = -1

        # 目标点（ENU：东/北/上）
        self.tgt_east = 0.0
        self.tgt_north = 0.0
        self.tgt_up = 0.0
        self.tgt_yaw_ned = 0.0
        self.target_ready = False

        # FAST-LIO2 里程计（ENU：x 东、y 北、z 上）
        self.flio_x = NAN
        self.flio_y = NAN
        self.flio_z = NAN
        self.flio_yaw = NAN
        self.flio_count = 0

        self.create_timer(TICK, self.tick)

    # ---------- 主循环 ----------
    def tick(self):
        self.n += 1
        elapsed = self.n * TICK

        if self.have_pos and not self.target_ready:
            # 用当前位姿初始化目标点，避免第一帧把飞机拉去 (0,0,0)
            self.tgt_east = self.y            # NED y = 东
            self.tgt_north = self.x           # NED x = 北
            self.tgt_up = -self.z             # NED z 向下，取负得向上
            self.tgt_yaw_ned = self.heading
            self.target_ready = True

        if self.target_ready:
            # 按键速度 → 目标点积分（机身系 → NED）
            c = math.cos(self.tgt_yaw_ned)
            s = math.sin(self.tgt_yaw_ned)
            vx, vy, vz = self.body_vel
            dn = vx * c - vy * s          # 北向
            de = vx * s + vy * c          # 东向
            self.tgt_north += dn * TICK
            self.tgt_east += de * TICK
            self.tgt_up -= vz * TICK      # NED z 向下，速度正=下降
            self.tgt_yaw_ned += self.yawspeed * TICK
            self.publish_target()

        # 松手衰减
        self.body_vel = [v * DECAY for v in self.body_vel]
        self.yawspeed *= DECAY

        if self.n % (PUB_HZ * 2) == 0:
            yaw_deg = math.degrees(self.heading) if self.heading_valid else float('nan')
            self.get_logger().info(
                f'[ {elapsed:5.0f}s ] '
                f'位置 x={self.x:5.2f} y={self.y:5.2f} 高度={-self.z:4.2f}m | '
                f'机头={yaw_deg:6.1f}deg | '
                f'机体系 vx={self.body_vel[0]:5.2f} vy={self.body_vel[1]:5.2f} vz={self.body_vel[2]:5.2f} | '
                f'目标 东={self.tgt_east:5.2f} 北={self.tgt_north:5.2f} 高={self.tgt_up:4.2f} '
                f'armed={self.arming_state} nav={self.nav_state}'
            )
            if self.flio_count > 0:
                d_east = self.flio_x - self.y      # LIO 东 - 飞控东(NED y)
                d_north = self.flio_y - self.x     # LIO 北 - 飞控北(NED x)
                d_up = self.flio_z - (-self.z)     # LIO 高 - 飞控高
                self.get_logger().info(
                    f'    LIO x={self.flio_x:6.2f} y={self.flio_y:6.2f} z={self.flio_z:5.2f} '
                    f'yaw={math.degrees(self.flio_yaw):6.1f}deg '
                    f'| 与飞控差 东={d_east:5.2f} 北={d_north:5.2f} 高={d_up:5.2f} '
                    f'| LIO帧数={self.flio_count}'
                )
            else:
                self.get_logger().warn('    LIO 未收到 /Odometry（FAST-LIO2 是否在运行？）')

    def publish_target(self):
        """发 ENU 目标位姿给 offboard_control（它自己转 NED）。"""
        msg = PoseStamped()
        msg.header.frame_id = 'camera_init'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.tgt_east)
        msg.pose.position.y = float(self.tgt_north)
        msg.pose.position.z = float(self.tgt_up)
        yaw_enu = math.pi / 2.0 - self.tgt_yaw_ned
        msg.pose.orientation.z = math.sin(0.5 * yaw_enu)
        msg.pose.orientation.w = math.cos(0.5 * yaw_enu)
        self.target_pub.publish(msg)

    # ---------- 键盘 ----------
    def on_key(self, ch):
        if ch not in (' ', 'w', 's', 'a', 'd', 'q', 'e', 'r', 'f'):
            return
        if ch == ' ':
            self.body_vel = [0.0, 0.0, 0.0]
            self.yawspeed = 0.0
        elif ch == 'w':
            self.body_vel[0] = SPEED            # 机头前
        elif ch == 's':
            self.body_vel[0] = -SPEED           # 机头后
        elif ch == 'd':
            self.body_vel[1] = SPEED            # 机头右
        elif ch == 'a':
            self.body_vel[1] = -SPEED           # 机头左
        elif ch == 'q':
            self.yawspeed = -YAW_RATE           # 左转（NED 负）
        elif ch == 'e':
            self.yawspeed = YAW_RATE            # 右转（NED 正）
        elif ch == 'r':
            self.body_vel[2] = -SPEED           # 上升
        elif ch == 'f':
            self.body_vel[2] = SPEED            # 下降

    # ---------- 回调 ----------
    def on_local_pos(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        if getattr(msg, 'heading_good_for_control', False) or not math.isnan(msg.heading):
            self.heading = float(msg.heading)
            self.heading_valid = not math.isnan(self.heading)
        self.have_pos = True

    def on_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def on_flio_odom(self, msg):
        p = msg.pose.pose.position
        self.flio_x = p.x
        self.flio_y = p.y
        self.flio_z = p.z
        q = msg.pose.pose.orientation
        self.flio_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.flio_count += 1


def main():
    rclpy.init()
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error('必须在交互式终端里运行（需要读键盘），请直接在终端执行 ros2 run px4_control keyboard_teleop')
        node.destroy_node()
        rclpy.shutdown()
        return

    print(BANNER, flush=True)

    # 把终端切成"不需要回车就能读单键"的模式；无论怎么退出都要还原，否则终端会坏掉
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=TICK)
            while select.select([sys.stdin], [], [], 0)[0]:
                node.on_key(sys.stdin.read(1))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    print('\n遥控结束')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
