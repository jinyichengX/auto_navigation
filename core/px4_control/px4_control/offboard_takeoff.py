#!/usr/bin/env python3
"""Offboard 起飞：升到 2 米悬停一段时间，然后自动降落。

流程：先持续发 setpoint 预热 -> 切 OFFBOARD 模式 -> 解锁 -> 悬停 -> 降落
"""
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
TARGET_Z = -2.0          # NED 坐标系里 z 轴向下为正，所以 -2.0 = 离地 2 米
PUB_HZ = 20              # 20Hz 持续发 setpoint，断了 PX4 会触发失效保护
TICK = 1.0 / PUB_HZ

T_PREHEAT = int(2.0 / TICK)      # 第 2 秒：切 offboard + 解锁
T_LAND = int(15.0 / TICK)        # 第 15 秒：降落
TOTAL_SECONDS = 32.0             # 脚本总运行时长（秒）

RESULT_NAME = {0: 'ACCEPTED 已接受', 1: 'TEMPORARILY_REJECTED 暂时拒绝',
               2: 'DENIED 拒绝', 3: 'UNSUPPORTED 不支持此命令', 4: 'FAILED 执行失败'}


class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')

        # 往 /fmu/in/* 发，用这个 QoS
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 收 /fmu/out/* ，用这个 QoS
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
        self.arming_state = -1
        self.nav_state = -1
        self.altitude = NAN
        self.create_timer(TICK, self.tick)
        self.get_logger().info(f'开始，目标高度 {-TARGET_Z:.0f} 米。前 2 秒预热，之后起飞。')

    # ---------- 每 20Hz 执行 ----------
    def tick(self):
        self.n += 1
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.n == T_PREHEAT:
            self.get_logger().info('切换到 OFFBOARD 模式，并解锁')
            self.send_command(100001, 14.0)      # VEHICLE_CMD_SET_NAV_STATE, nav_state=14(OFFBOARD)
            self.send_command(400, 1.0)          # VEHICLE_CMD_COMPONENT_ARM_DISARM, 1=解锁

        if self.n == T_LAND:
            self.get_logger().info('发送降落命令')
            self.send_command(21, 0.0)           # VEHICLE_CMD_NAV_LAND

        if self.n % (PUB_HZ * 2) == 0:           # 每 2 秒打一行进度
            self.get_logger().info(
                f'[ {self.n * TICK:4.0f}s ] arming_state={self.arming_state} '
                f'nav_state={self.nav_state} 高度={-self.altitude:.2f}m'
            )

    def publish_offboard_control_mode(self):
        """告诉 PX4：我这一帧控制的是位置"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """目标点：原点正上方 2 米，机头朝北。速度/加速度填 NaN 表示不约束"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [0.0, 0.0, TARGET_Z]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = 0.0
        msg.yawspeed = NAN
        self.sp_pub.publish(msg)

    def send_command(self, command, param1):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
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
        self.altitude = msg.z

    def on_ack(self, msg):
        name = RESULT_NAME.get(msg.result, f'未知({msg.result})')
        self.get_logger().info(f'命令回执 -> command={msg.command} result={msg.result} ({name})')


def main():
    rclpy.init()
    node = OffboardTakeoff()
    # 用真实时间控制总时长：spin_once 一有回调就返回，用循环次数控制会提前跑完
    end_time = time.time() + TOTAL_SECONDS
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=TICK)
    node.get_logger().info('脚本结束')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
