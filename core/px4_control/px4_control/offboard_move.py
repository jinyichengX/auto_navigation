#!/usr/bin/env python3
"""Offboard 方形航线（目标点平滑过渡版）

航线：原地起飞 -> 向北 3 米 -> 向东 3 米 -> 向南 3 米 -> 向西飞回起点 -> 降落 -> 上锁
NED 坐标系：x 向正北，y 向正东，z 向下为正（所以 z=-2.0 表示离地 2 米）

注意：walls 世界里墙在 x=4.5 和 y=4.5，所以方形边长不能超过 4 米。
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
PUB_HZ = 20                      # 20Hz 持续发 setpoint，断流 PX4 会触发失效保护
TICK = 1.0 / PUB_HZ
MAX_SPEED = 1.0                  # 目标点移动的最大速度 (m/s)，越小越平缓

T_PREHEAT = int(2.0 / TICK)      # 第 2 秒：切 offboard + 解锁

# 航线：(触发时间秒, 目标点 xyz, 说明)
WAYPOINTS = [
    (2.0,  (0.0, 0.0, -2.0), '飞到起点上方 2 米'),
    (6.0,  (3.0, 0.0, -2.0), '向北飞 3 米'),
    (10.0, (3.0, 3.0, -2.0), '向东飞 3 米'),
    (14.0, (0.0, 3.0, -2.0), '向南飞 3 米'),
    (18.0, (0.0, 0.0, -2.0), '向西飞回起点'),
]
LAND_AT = 23.0                   # 第 23 秒：降落
DISARM_AT = 35.0                 # 第 35 秒：上锁
TOTAL_SECONDS = 38.0

RESULT_NAME = {0: 'ACCEPTED 已接受', 1: 'TEMPORARILY_REJECTED 暂时拒绝',
               2: 'DENIED 拒绝', 3: 'UNSUPPORTED 不支持此命令', 4: 'FAILED 执行失败'}


def step_toward(cur, tgt, max_step):
    """把当前 setpoint 朝目标推进最多 max_step 米，返回 (新 setpoint, 前馈速度)。

    这样目标点是渐变过去的，位置控制器不会因为目标突跳而要求速度突变。
    """
    d = [tgt[i] - cur[i] for i in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    if dist <= max_step or dist == 0.0:
        return [float(v) for v in tgt], [0.0, 0.0, 0.0]
    s = max_step / dist
    new_sp = [cur[i] + d[i] * s for i in range(3)]
    ff_vel = [d[i] / dist * MAX_SPEED for i in range(3)]
    return new_sp, ff_vel


class OffboardMove(Node):
    def __init__(self):
        super().__init__('offboard_move')

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
        self.wp_index = 0
        self.target = None
        self.sp = [0.0, 0.0, 0.0]        # 当前的（平滑后的）目标点
        self.sp_vel = [0.0, 0.0, 0.0]    # 前馈速度
        self.sp_init = False             # setpoint 是否已按飞机的实际位置初始化
        self.armed_sent = False
        self.land_sent = False
        self.disarm_sent = False
        self.arming_state = -1
        self.nav_state = -1
        self.x = NAN
        self.y = NAN
        self.z = NAN

        self.create_timer(TICK, self.tick)
        self.get_logger().info('开始：原地起飞 -> 方形航线 3x3 米 -> 降落 -> 上锁（目标点平滑过渡）')

    def tick(self):
        self.n += 1
        elapsed = self.n * TICK

        # 第一次拿到位置估计时，把 setpoint 设成飞机的实际位置。
        # 这样不管上次降落在哪里，飞机都是原地垂直起飞，不会先斜着飞回去。
        if not self.sp_init:
            if math.isnan(self.z):
                return
            self.sp = [self.x, self.y, self.z]
            self.sp_init = True

        # 按时间推进航点
        if self.wp_index < len(WAYPOINTS) and elapsed >= WAYPOINTS[self.wp_index][0]:
            _, wp, desc = WAYPOINTS[self.wp_index]
            self.target = wp
            self.get_logger().info(f'-> 新目标 {wp} （{desc}）')
            self.wp_index += 1

        # 目标点平滑推进
        if self.target is not None:
            self.sp, self.sp_vel = step_toward(self.sp, self.target, MAX_SPEED * TICK)

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 第 2 秒：切 OFFBOARD 模式 + 解锁
        if not self.armed_sent and elapsed >= 2.0:
            self.armed_sent = True
            self.get_logger().info('切换到 OFFBOARD 模式，并解锁')
            self.send_command(100001, 14.0)      # VEHICLE_CMD_SET_NAV_STATE, nav_state=14
            self.send_command(400, 1.0)          # VEHICLE_CMD_COMPONENT_ARM_DISARM, 1=解锁

        # 降落
        if not self.land_sent and elapsed >= LAND_AT:
            self.land_sent = True
            self.get_logger().info('发送降落命令')
            self.send_command(21, 0.0)           # VEHICLE_CMD_NAV_LAND

        # 落地后上锁（param2=21196 是 PX4 的"强制"标记，跳过飞行中保护检查）
        if not self.disarm_sent and elapsed >= DISARM_AT:
            self.disarm_sent = True
            self.get_logger().info('发送上锁命令')
            self.send_command(400, 0.0, param2=21196.0)

        if self.n % (PUB_HZ * 2) == 0:           # 每 2 秒打一行进度
            remain = NAN
            if self.target is not None:
                remain = math.dist(self.sp, self.target)
            self.get_logger().info(
                f'[ {elapsed:4.0f}s ] arming_state={self.arming_state} '
                f'nav_state={self.nav_state} '
                f'位置 x={self.x:.2f} y={self.y:.2f} 高度={-self.z:.2f}m '
                f'| setpoint 剩余 {remain:.2f}m'
            )

    def publish_offboard_control_mode(self):
        """告诉 PX4：我这一帧控制的是位置"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """发平滑后的目标点 + 前馈速度"""
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
    node = OffboardMove()
    end_time = time.time() + TOTAL_SECONDS
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=TICK)
    node.get_logger().info('脚本结束')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
