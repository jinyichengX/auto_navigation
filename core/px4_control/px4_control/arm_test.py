#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleCommand


class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test')
        # QoS 必须和 PX4 侧订阅 /fmu/in/* 的设置一致，否则指令收不到
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

    def send_arm(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 400        # VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0         # 1 = 解锁，0 = 上锁
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmTest()
    for _ in range(20):          # 连发 20 次，避免丢包
        node.send_arm()
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info('解锁指令已发送')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
