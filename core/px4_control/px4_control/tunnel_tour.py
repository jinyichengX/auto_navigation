#!/usr/bin/env python3
"""十字巷道航点遍历：起飞后按顺序逛完四条巷道，再回原点降落。

坐标系是飞控 local position（NED）：
  x 向北，y 向东，z 向下（所以高度 1.2 米对应 z=-1.2）

航线：
  原点起飞 -> 北端 -> 中心 -> 东端 -> 中心 -> 南端 -> 中心 -> 西端 -> 原点 -> 降落
"""
import math
import time

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
MAX_SPEED = 0.8          # 目标点平滑推进速度 m/s，巷道里慢一点更稳
ARRIVE_DIST = 0.45       # 离目标点这么近就算到了
HOLD_SEC = 1.0           # 到点后悬停多久再去下一点
FLY_ALT = -1.2           # 飞行高度：离地 1.2 米（顶板约 3 米）
ARM_REACH = 7.0          # 四条巷道端点，留出离端头墙的安全距离

# 到点才切换，不靠固定时间表
WAYPOINTS = [
    ((0.0, 0.0, FLY_ALT), '起飞到交叉口上方'),
    ((ARM_REACH, 0.0, FLY_ALT), '去北端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((0.0, ARM_REACH, FLY_ALT), '去东端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((-ARM_REACH, 0.0, FLY_ALT), '去南端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((0.0, -ARM_REACH, FLY_ALT), '去西端'),
    ((0.0, 0.0, FLY_ALT), '回原点准备降落'),
]

RESULT_NAME = {
    0: 'ACCEPTED 已接受',
    1: 'TEMPORARILY_REJECTED 暂时拒绝',
    2: 'DENIED 拒绝',
    3: 'UNSUPPORTED 不支持此命令',
    4: 'FAILED 执行失败',
}


def step_toward(cur, tgt, max_step):
    d = [tgt[i] - cur[i] for i in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    if dist <= max_step or dist == 0.0:
        return [float(v) for v in tgt], [0.0, 0.0, 0.0]
    s = max_step / dist
    new_sp = [cur[i] + d[i] * s for i in range(3)]
    ff_vel = [d[i] / dist * MAX_SPEED for i in range(3)]
    return new_sp, ff_vel


class TunnelTour(Node):
    def __init__(self):
        super().__init__('tunnel_tour')

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
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.on_local_pos,
            qos_out,
        )

        self.n = 0
        self.wp_index = 0
        self.target = None
        self.sp = [0.0, 0.0, 0.0]
        self.sp_vel = [0.0, 0.0, 0.0]
        self.sp_init = False
        self.hold_ticks = 0
        self.phase = 'preheat'   # preheat -> fly -> land -> disarm -> done
        self.armed_sent = False
        self.land_sent = False
        self.disarm_sent = False
        self.done = False
        self.land_tick = None
        self.arming_state = -1
        self.nav_state = -1
        self.x = NAN
        self.y = NAN
        self.z = NAN

        self.create_timer(TICK, self.tick)
        self.get_logger().info(
            f'开始巷道遍历：四臂各到 {ARM_REACH:.0f} m，飞行高度 {-FLY_ALT:.1f} m，到点后悬停 {HOLD_SEC:.0f} s'
        )

    def tick(self):
        if self.done:
            return

        self.n += 1
        elapsed = self.n * TICK

        if not self.sp_init:
            if math.isnan(self.z):
                return
            self.sp = [self.x, self.y, self.z]
            self.sp_init = True

        if self.phase == 'preheat':
            if not self.armed_sent and elapsed >= 2.0:
                self.armed_sent = True
                self.get_logger().info('切换到 OFFBOARD 模式，并解锁')
                self.send_command(100001, 14.0)
                self.send_command(400, 1.0)
                self._set_waypoint(0)
                self.phase = 'fly'

        elif self.phase == 'fly':
            if self.target is not None:
                self.sp, self.sp_vel = step_toward(self.sp, self.target, MAX_SPEED * TICK)
                remain = math.dist([self.x, self.y, self.z], self.target)
                if remain <= ARRIVE_DIST:
                    self.hold_ticks += 1
                    self.sp_vel = [0.0, 0.0, 0.0]
                    if self.hold_ticks >= int(HOLD_SEC / TICK):
                        next_idx = self.wp_index + 1
                        if next_idx < len(WAYPOINTS):
                            self._set_waypoint(next_idx)
                        else:
                            self.get_logger().info('航点全部完成，准备降落')
                            self.phase = 'land'
                else:
                    self.hold_ticks = 0

        elif self.phase == 'land':
            if not self.land_sent:
                self.land_sent = True
                self.land_tick = self.n
                self.get_logger().info('发送降落命令')
                self.send_command(21, 0.0)
            elif self.n - self.land_tick >= int(12.0 / TICK):
                self.phase = 'disarm'

        elif self.phase == 'disarm':
            if not self.disarm_sent:
                self.disarm_sent = True
                self.get_logger().info('发送上锁命令')
                self.send_command(400, 0.0, param2=21196.0)
            self.done = True
            self.get_logger().info('遍历结束')

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.n % (PUB_HZ * 2) == 0:
            remain = NAN
            if self.target is not None and not math.isnan(self.x):
                remain = math.dist([self.x, self.y, self.z], self.target)
            self.get_logger().info(
                f'[ {elapsed:4.0f}s | {self.phase:6s} ] '
                f'wp={self.wp_index + 1}/{len(WAYPOINTS)} '
                f'位置 x={self.x:.2f} y={self.y:.2f} 高度={-self.z:.2f}m '
                f'| 距目标 {remain:.2f}m'
            )

    def _set_waypoint(self, index):
        self.wp_index = index
        self.target = WAYPOINTS[index][0]
        self.hold_ticks = 0
        self.get_logger().info(
            f'-> 航点 {index + 1}/{len(WAYPOINTS)}: {self.target} （{WAYPOINTS[index][1]}）'
        )

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(v) for v in self.sp]
        msg.velocity = [float(v) for v in self.sp_vel]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = 0.0
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
    node = TunnelTour()
    # 最长兜底 8 分钟，正常会提前结束
    end_time = time.time() + 480.0
    while rclpy.ok() and time.time() < end_time and not node.done:
        rclpy.spin_once(node, timeout_sec=TICK)
    if not node.done:
        node.get_logger().warn('超时退出，请检查飞机状态')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
