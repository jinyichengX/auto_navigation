#!/usr/bin/env python3
"""把 .las 点云文件发布成 sensor_msgs/PointCloud2 话题。

用法示例:
  python3 las_to_pointcloud2.py --file shuju.las --voxel 0.2 --topic /map_cloud
"""
#python3 las_to_pointcloud2.py --file shuju.las --topic /map_cloud --voxel 0 --chunk 1000000 --rate 20 --period 0
#- 发了 ：`x``y``z` （float32，已减去原点）、`intensity` （float32，保留 0~227 原值）、`rgb` （打包成 float32 的 8 位 RGB）
#- 没发 ：回波标志位（Return Flags）、分类（Classification）、扫描角（Scan Angle Rank）、用户数据（User Data）、航带编号（Point Source ID）、GPS 时间（GPS Time）
import argparse
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

# LAS 点记录布局（只列到 RGB 需要的偏移）
RECORD_LAYOUT = {
    0: dict(size=20, rgb=None),
    1: dict(size=28, rgb=None),
    2: dict(size=26, rgb=20),
    3: dict(size=34, rgb=28),
    4: dict(size=57, rgb=28),
    5: dict(size=63, rgb=28),
}


def read_las_header(path):
    with open(path, "rb") as f:
        h = f.read(375)
    if h[:4] != b"LASF":
        raise RuntimeError("不是合法的 LAS 文件")
    header_size = struct.unpack("<H", h[94:96])[0]
    data_offset = struct.unpack("<I", h[96:100])[0]
    point_format = h[104] & 0x3F
    record_len = struct.unpack("<H", h[105:107])[0]
    n_points = struct.unpack("<I", h[107:111])[0]
    legacy = n_points
    if h[24] == 1 and h[25] >= 4 or h[24] > 1:
        n_points = struct.unpack("<Q", h[247:255])[0] or legacy
    scale = np.array(struct.unpack("<3d", h[131:155]))
    offset = np.array(struct.unpack("<3d", h[155:179]))
    return dict(header_size=header_size, data_offset=data_offset,
                point_format=point_format, record_len=record_len,
                n_points=n_points, scale=scale, offset=offset)


class LasCloud(Node):
    def __init__(self, args, header):
        super().__init__("las_to_pointcloud2")
        self.args = args
        self.header = header
        qos = QoSProfile(depth=5,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)

    def load(self):
        args, hd = self.args, self.header
        layout = RECORD_LAYOUT.get(hd["point_format"])
        if layout is None:
            raise RuntimeError(f"暂不支持的 point format: {hd['point_format']}")

        rgb_off = layout["rgb"]
        use_color = rgb_off is not None and not args.no_color

        names, formats, offsets = ["x", "y", "z", "intensity"], \
            ["<i4", "<i4", "<i4", "<u2"], [0, 4, 8, 12]
        if use_color:
            names += ["r", "g", "b"]
            formats += ["<u2", "<u2", "<u2"]
            offsets += [rgb_off, rgb_off + 2, rgb_off + 4]
        dt = np.dtype({"names": names, "formats": formats,
                       "offsets": offsets, "itemsize": hd["record_len"]})
        raw = np.memmap(args.file, dtype=dt, mode="r",
                        offset=hd["data_offset"], shape=(hd["n_points"],))

        n = hd["n_points"]
        xyz = np.empty((n, 3), dtype=np.float32)
        inten = np.empty(n, dtype=np.float32)
        color = np.empty((n, 3), dtype=np.uint8) if use_color else None

        step = 2_000_000
        for s in range(0, n, step):
            e = min(s + step, n)
            chunk = raw[s:e]
            p = np.stack([chunk["x"], chunk["y"], chunk["z"]], 1).astype(np.float64)
            p = p * hd["scale"] + hd["offset"]
            p -= args.origin
            xyz[s:e] = p.astype(np.float32)
            inten[s:e] = chunk["intensity"]
            if use_color:
                color[s:e, 0] = chunk["r"] >> 8
                color[s:e, 1] = chunk["g"] >> 8
                color[s:e, 2] = chunk["b"] >> 8
            self.get_logger().info(f"读取 {e}/{n}")

        del raw
        return xyz, inten, color

    @staticmethod
    def voxel_filter(xyz, inten, color, voxel):
        if voxel <= 0.0:
            return xyz, inten, color
        ijk = np.floor(xyz.astype(np.float64) / voxel).astype(np.int64)
        ijk -= ijk.min(axis=0)
        dims = ijk.max(axis=0) + 1
        key = (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]
        del ijk
        _, idx = np.unique(key, return_index=True)
        del key
        idx.sort()
        keep_c = None if color is None else color[idx]
        return xyz[idx], inten[idx], keep_c

    def build_msg(self, xyz, inten, color):
        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)]
        if color is not None:
            fields.append(PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1))
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                           ("intensity", "<f4"), ("rgb", "<f4")])
            packed = color[:, 0].astype(np.uint32) << 16 | \
                color[:, 1].astype(np.uint32) << 8 | color[:, 2].astype(np.uint32)
            arr = np.empty(xyz.shape[0], dtype=dt)
            arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            arr["intensity"] = inten
            arr["rgb"] = packed.view(np.float32)
        else:
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
            arr = np.empty(xyz.shape[0], dtype=dt)
            arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            arr["intensity"] = inten
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame
        msg.height = 1
        msg.width = arr.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = dt.itemsize
        msg.row_step = dt.itemsize * arr.shape[0]
        msg.is_dense = True
        msg.data = arr.tobytes()
        return msg

    def publish_all(self, xyz, inten, color):
        n = xyz.shape[0]
        chunk = self.args.chunk
        color_missing = color is None
        for s in range(0, n, chunk):
            e = min(s + chunk, n)
            c = None if color_missing else color[s:e]
            self.pub.publish(self.build_msg(xyz[s:e], inten[s:e], c))
            time.sleep(1.0 / self.args.rate)
        self.get_logger().info(f"已发送 {n} 个点，话题 {self.args.topic}，frame {self.args.frame}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="shuju.las")
    ap.add_argument("--topic", default="/map_cloud")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--voxel", type=float, default=0.0, help="体素降采样边长(米)，0 表示不降采样")
    ap.add_argument("--chunk", type=int, default=200000, help="每条消息的点数")
    ap.add_argument("--rate", type=float, default=5.0, help="消息发送频率(Hz)")
    ap.add_argument("--period", type=float, default=10.0, help="整幅点云重复发布周期(秒)，0 表示只发一次")
    ap.add_argument("--origin", type=float, nargs=3, default=None,
                    help="输出的坐标原点，默认使用 LAS 头部 offset")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()

    hd = read_las_header(args.file)
    if args.origin is None:
        args.origin = hd["offset"]
    args.origin = np.array(args.origin, dtype=np.float64)
    print(f"点数 {hd['n_points']}，格式 {hd['point_format']}，"
          f"输出原点 {args.origin.tolist()}")

    rclpy.init()
    node = LasCloud(args, hd)
    xyz, inten, color = node.load()
    xyz, inten, color = node.voxel_filter(xyz, inten, color, args.voxel)
    node.get_logger().info(f"降采样后点数 {xyz.shape[0]}")
    try:
        while rclpy.ok():
            node.publish_all(xyz, inten, color)
            if args.period <= 0:
                break
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
