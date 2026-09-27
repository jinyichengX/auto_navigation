#!/usr/bin/env python3
"""键盘遥控飞行（Offboard 速度控制）

按键：
  w / s   前进 / 后退
  a / d   左移 / 右移
  r / f   上升 / 下降
  空格    立刻停住
  l       降落并上锁，然后退出

按住不放会持续飞（靠键盘自动重复），松开约 0.5 秒后自动停下。
活动范围限制在水平 ±3.5 米、高度 0.4~3.0 米（walls 世界的墙在 4.5 米处）。
"""
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleCommandAck,
    VehicleLocalPosition,
)

NAN = float('nan')
PUB_HZ = 20
TICK = 1.0 / PUB_HZ

SPEED = 2.0           # m/s，按键时给的目标速度
DECAY = 0.88          # 每个 tick 衰减系数：松手后约 0.5 秒停住
LIMIT_XY = 3.5        # 水平活动范围 ±3.5 米
LIMIT_ALT_MAX = 3.0   # 最高 3 米
LIMIT_ALT_MIN = 0.4   # 最低 0.4 米
TAKEOFF_SPEED = 0.5   # 起飞爬升速度 m/s
TAKEOFF_ALT = 1.5     # 爬到该高度后转入遥控
LAND_TIMEOUT = 12.0   # 发降落命令后等多久再上锁

RESULT_NAME = {0: 'ACCEPTED 已接受', 1: 'TEMPORARILY_REJECTED 暂时拒绝',
               2: 'DENIED 拒绝', 3: 'UNSUPPORTED 不支持此命令', 4: 'FAILED 执行失败'}

BANNER = """
==================== 键盘遥控 ====================
  w / s   前进 / 后退        a / d   左移 / 右移
  r / f   上升 / 下降        空格    立刻停住
  l       降落并上锁退出
按住不放持续飞，松开自动停。结束请按 l，不要按 Ctrl+C。
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

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_in)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_in)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_in)

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos_out)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack, qos_out)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self.on_local_pos, qos_out)

        self.n = 0
        self.phase = 'preheat'          # preheat -> takeoff -> teleop -> landing
        self.vel = [0.0, 0.0, 0.0]      # 当前速度指令（NED）
        self.arming_state = -1
        self.nav_state = -1
        self.x = NAN
        self.y = NAN
        self.z = NAN
        self.land_tick = None
        self.disarm_sent = False
        self.done = False

        self.create_timer(TICK, self.tick)

    # ---------- 主循环 ----------
    def tick(self):
        self.n += 1
        elapsed = self.n * TICK

        if self.phase == 'preheat' and elapsed >= 2.0:
            self.get_logger().info('切换到 OFFBOARD 模式，并解锁，开始爬升')
            self.send_command(100001, 14.0)      # VEHICLE_CMD_SET_NAV_STATE, nav_state=14
            self.send_command(400, 1.0)          # 解锁
            self.phase = 'takeoff'

        if self.phase == 'takeoff':
            self.vel = [0.0, 0.0, -TAKEOFF_SPEED]     # NED: z 负 = 上升
            if not math.isnan(self.z) and -self.z >= TAKEOFF_ALT:
                self.phase = 'teleop'
                self.vel = [0.0, 0.0, 0.0]
                self.get_logger().info(f'已升到 {-self.z:.2f} 米，现在可以遥控了')

        elif self.phase == 'teleop':
            self.vel = [v * DECAY for v in self.vel]  # 松手衰减，实现"松开就停"
            self.clamp_vel()

        elif self.phase == 'landing':
            self.vel = [0.0, 0.0, 0.0]
            if not self.disarm_sent and self.land_tick is not None \
                    and (self.n - self.land_tick) * TICK >= LAND_TIMEOUT:
                self.disarm_sent = True
                self.get_logger().info('发送上锁命令')
                self.send_command(400, 0.0, param2=21196.0)
                self.done = True

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.n % (PUB_HZ * 2) == 0:
            self.get_logger().info(
                f'[ {elapsed:5.0f}s | {self.phase:8s} ] '
                f'位置 x={self.x:5.2f} y={self.y:5.2f} 高度={-self.z:4.2f}m | '
                f'速度 vx={self.vel[0]:5.2f} vy={self.vel[1]:5.2f} vz={self.vel[2]:5.2f}'
                f'  armed={self.arming_state} nav={self.nav_state}'
            )

    def clamp_vel(self):
        """别飞出安全范围，也别撞墙"""
        vx, vy, vz = self.vel
        if (self.x > LIMIT_XY and vx > 0) or (self.x < -LIMIT_XY and vx < 0):
            vx = 0.0
        if (self.y > LIMIT_XY and vy > 0) or (self.y < -LIMIT_XY and vy < 0):
            vy = 0.0
        alt = -self.z
        if (alt > LIMIT_ALT_MAX and vz < 0) or (alt < LIMIT_ALT_MIN and vz > 0):
            vz = 0.0
        self.vel = [vx, vy, vz]

    # ---------- 键盘 ----------
    def on_key(self, ch):
        if ch == 'l':
            self.get_logger().info('收到降落指令')
            self.send_command(21, 0.0)           # VEHICLE_CMD_NAV_LAND
            self.phase = 'landing'
            self.land_tick = self.n
            return

        if self.phase != 'teleop':
            return

        if ch == ' ':
            self.vel = [0.0, 0.0, 0.0]
        elif ch == 'w':
            self.vel[0] = SPEED
        elif ch == 's':
            self.vel[0] = -SPEED
        elif ch == 'd':
            self.vel[1] = SPEED
        elif ch == 'a':
            self.vel[1] = -SPEED
        elif ch == 'r':
            self.vel[2] = -SPEED               # NED 里 z 负方向是上升
        elif ch == 'f':
            self.vel[2] = SPEED

    # ---------- 发布 ----------
    def publish_offboard_control_mode(self):
        """告诉 PX4：我这一帧控制的是速度"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = True
        self.mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """速度控制：位置填 NaN 表示不控制位置"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [NAN, NAN, NAN]
        msg.velocity = [float(v) for v in self.vel]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = NAN
        msg.yawspeed = NAN
        self.sp_pub.publish(msg)

    def send_command(self, command, param1, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    # ---------- 回调 ----------
    def on_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def on_local_pos(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def on_ack(self, msg):
        name = RESULT_NAME.get(msg.result, f'未知({msg.result})')
        self.get_logger().info(f'命令回执 -> command={msg.command} result={msg.result} ({name})')


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
        while rclpy.ok() and not node.done:
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
