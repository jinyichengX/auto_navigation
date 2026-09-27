#!/usr/bin/env python3
"""把 Gazebo 里 Livox Mid-360 的点云转成 ROS 2 的 PointCloud2

用法：先启动 PX4 SITL（机型 gz_x500_livox_mid360），再运行本节点。
节点会自动在 Gazebo 的话题列表里找点云话题，不需要手填名字。

发布：/livox/lidar   (sensor_msgs/PointCloud2)

为什么不用 ros_gz_bridge：Humble 自带的 ros_gz 是 Fortress 版，和你的 Harmonic
不匹配；而装 Harmonic 版会卸载 Fortress 版，影响 FishBot 项目。Gazebo 自带
gz-transport 的 Python 绑定正好够用，所以这里自己转，一个系统包都不动。
"""
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from gz.transport13 import Node as GzNode
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

# Gazebo 和 ROS 的 PointField 数据类型枚举【数值不一致】，必须按名字映射，不能照搬数字：
#   Gazebo: INT8=0 UINT8=1 INT16=2 UINT16=3 INT32=4 UINT32=5 FLOAT32=6 FLOAT64=7
#   ROS:    INT8=1 UINT8=2 INT16=3 UINT16=4 INT32=5 UINT32=6 FLOAT32=7 FLOAT64=8
# 整体差 1。之前直接照搬导致了把 FLOAT32 声明成 UINT32，点云在 RViz 里直接废掉。
_GZ_DATATYPE_ENUM = (
    PointCloudPacked.DESCRIPTOR.fields_by_name['field']
    .message_type.fields_by_name['datatype'].enum_type
)


def ros_datatype(gz_value):
    """把 Gazebo 的 Field.datatype 数值翻成 ROS sensor_msgs/PointField 的数值"""
    try:
        name = _GZ_DATATYPE_ENUM.values_by_number[gz_value].name
        return getattr(PointField, name)
    except (KeyError, AttributeError):
        return PointField.FLOAT32


DEFAULT_FRAME = 'livox_link'
PARENT_FRAME = 'base_link'
LIVOX_MOUNT_Z = 0.28        # 雷达相对 base_link 的安装高度，和 SDF 里的 <pose relative_to="base_link"> 必须一致


class GzLidarBridge(Node):
    def __init__(self):
        super().__init__('gz_lidar_bridge')
        self.pub = self.create_publisher(PointCloud2, '/livox/lidar', 5)
        self.frames = 0
        self.ok = False
        self.stopping = False
        self.lock = threading.Lock()
        self.topic = None

        # 发布固定的安装变换 base_link -> livox_link。
        # ROS 2 里 TF 不会自动产生，必须有人发布；没有它 RViz 的 Fixed Frame 会一个都选不到。
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_mount_tf()

        self.gz = GzNode()
        topic = self.find_topic()
        if topic is None:
            self.get_logger().error(
                '在 Gazebo 话题列表里没找到 Mid-360 的点云话题。'
                '请确认 PX4 仿真是以 gz_x500_livox_mid360 机型启动的。')
            return

        self.get_logger().info(f'Gazebo 话题: {topic}')
        self.get_logger().info('ROS 2 话题: /livox/lidar')
        self.topic = topic
        self.gz.subscribe(PointCloudPacked, topic, self.on_cloud)
        self.ok = True

    def publish_mount_tf(self):
        """雷达相对机身的固定平移，旋转为零（雷达朝向和机身一致）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id = DEFAULT_FRAME
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = LIVOX_MOUNT_Z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def stop(self):
        """先禁止 gz 回调再发布，然后退订，避免节点销毁后回调还在用 publisher"""
        with self.lock:
            self.stopping = True
        if self.topic is not None:
            self.gz.unsubscribe(self.topic)

    def find_topic(self):
        """话题名形如 /world/<world>/model/<model>/link/livox_link/sensor/livox_mid360_lidar/scan/points"""
        for t in self.gz.topic_list():
            if 'livox_mid360' in t and t.endswith('/points'):
                return t
        return None

    def on_cloud(self, msg):
        out = PointCloud2()

        # 时间戳统一用系统时间（节点时钟），不要用 Gazebo 的 header.stamp。
        # 原因：Gazebo 仿真时间是从世界启动开始从 0 计的（比如 1302 秒 = 1970 年），
        # 而 PX4 通过 uXRCE-DDS 发的 /fmu/out/* 时间戳是系统时间（约 2026 年）。
        # 两者混用会导致 RViz 查不到变换，后面接 FAST-LIO2 做雷达-IMU 融合时
        # 时间基准不一致会直接出问题，所以这里统一到系统时间。
        out.header.stamp = self.get_clock().now().to_msg()

        out.header.frame_id = DEFAULT_FRAME

        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = bytes(msg.data)

        for f in msg.field:
            out.fields.append(PointField(
                name=f.name,
                offset=f.offset,
                datatype=ros_datatype(f.datatype),
                count=f.count,
            ))

        with self.lock:
            if self.stopping:
                return
            self.pub.publish(out)

        self.frames += 1
        if self.frames % 10 == 1:          # 每 10 帧打一行，避免刷屏
            self.get_logger().info(
                f'已转发 {self.frames} 帧 | 每帧 {msg.width * msg.height} 点 | '
                f'{msg.point_step} 字节/点 | 字段 {[f.name for f in msg.field]} | '
                f'frame_id={out.header.frame_id}')


def main():
    rclpy.init()
    node = GzLidarBridge()
    if node.ok:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.stop()
    node.destroy_node()
    if rclpy.ok():          # rclpy 自己的 SIGINT 处理可能已经关过了
        rclpy.shutdown()


if __name__ == '__main__':
    main()
