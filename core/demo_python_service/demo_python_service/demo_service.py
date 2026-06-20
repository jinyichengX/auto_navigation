from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node



def main():
    rclpy.init()
    node = Node('demo_service')
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()