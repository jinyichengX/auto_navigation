#!/usr/bin/python3
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info('你好 python node')
    node.get_logger().warn('你好 python node')
    rclpy.spin(node)
    rclpy.shutdown()