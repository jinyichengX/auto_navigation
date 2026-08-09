import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster  # 动态变换广播器
from geometry_msgs.msg import TransformStamped
import math
import time

from demo_python_tf.static_tf_broadcaster import euler_to_quaternion  # 复用已有的转换函数


class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)  # 动态广播器
        self.timer = self.create_timer(0.1, self.publish_tf)  # 10Hz 定时发布
        self.start_time = time.time()
        self.get_logger().info('动态 TF 广播器已启动 (camera_link -> bottle_link)')

    def publish_tf(self):
        # 模拟瓶子在相机前方做圆周运动
        elapsed = time.time() - self.start_time
        radius = 0.3                     # 半径 30cm
        angle = elapsed * 0.5            # 角速度 0.5 rad/s

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'       # 父：相机
        t.child_frame_id = 'bottle_link'         # 子：瓶子
        t.transform.translation.x = 1.0          # 相机前方 1 米
        t.transform.translation.y = radius * math.cos(angle)  # 左右摆动
        t.transform.translation.z = radius * math.sin(angle)  # 上下摆动
        # 瓶子朝向不变
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'发布静态变换: {t}')

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

