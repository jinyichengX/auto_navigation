import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 导入静态变换广播器
from geometry_msgs.msg import TransformStamped #消息接口
import math


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角(弧度)转四元数，返回(x,y,z,w)，不依赖任何第三方库"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        # 欧拉角转四元数（自实现，零依赖）
        rotation_quat = euler_to_quaternion(math.radians(180), 0.0, 0.0)
        transform.transform.rotation.x = rotation_quat[0]
        transform.transform.rotation.y = rotation_quat[1]
        transform.transform.rotation.z = rotation_quat[2]
        transform.transform.rotation.w = rotation_quat[3]
        # 发布静态变换
        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(f'发布静态变换: {transform}')

def main():
    rclpy.init()
    static_tf_broadcaster = StaticTFBroadcaster()
    rclpy.spin(static_tf_broadcaster)
    rclpy.shutdown()