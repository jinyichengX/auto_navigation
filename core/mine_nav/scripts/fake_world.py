#!/usr/bin/env python3
"""假世界：占用图 + 假飞控局部位姿，给 mine_nav 四层做节点级冒烟。

支持两种地图：
  - cross：十字巷道（回归旧场景）
  - irregular：斜向巷道 + 不规则空洞 + 偏置障碍（验证任意地图）

位姿默认跟随 /fmu/in/trajectory_setpoint。

用法：
  source /opt/ros/humble/setup.bash
  source ~/auto_navigation/install/setup.bash
  ros2 run mine_nav fake_world.py
  ros2 run mine_nav fake_world.py --ros-args -p map_type:=irregular
"""

from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FakeWorld(Node):
    def __init__(self) -> None:
        super().__init__('fake_world')
        self.declare_parameter('follow_setpoint', True)
        self.declare_parameter('max_speed', 1.2)  # m/s，假飞机追 setpoint 的速度
        self.declare_parameter('map_type', 'irregular')  # cross | irregular
        # 飞到哪揭到哪：把附近 unknown(-1) 变成 free(0)，模拟传感器建图
        self.declare_parameter('reveal_radius', 1.8)

        qos_be_tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_rel = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pos_pub = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', qos_be_tl)
        self.map_pub = self.create_publisher(OccupancyGrid, '/projected_map', qos_rel)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.on_setpoint, qos_be)

        # 单实例保护：同域里已有别人在发 /projected_map，就退出
        # 等 discovery 一下；自己刚创建的 publisher 也算 1，所以 >1 才拒
        time.sleep(0.3)
        try:
            pubs = self.get_publishers_info_by_topic('/projected_map')
            if len(pubs) > 1:
                raise RuntimeError(
                    f'检测到已有 {len(pubs)} 个 /projected_map 发布者，拒绝重复启动')
        except RuntimeError:
            raise
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'单实例检查跳过：{exc}')

        self.follow_setpoint = bool(self.get_parameter('follow_setpoint').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.map_type = str(self.get_parameter('map_type').value).strip().lower()
        self.reveal_radius = float(self.get_parameter('reveal_radius').value)
        self.n = 0.0
        self.e = 0.0
        self.z = -1.2
        self.sp_n = 0.0
        self.sp_e = 0.0
        self.sp_z = -1.2
        self.have_sp = False
        self.dt = 0.1

        self.build_map()
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'假世界已启动：map_type={self.map_type} follow_setpoint={self.follow_setpoint} '
            f'reveal_radius={self.reveal_radius:.2f}')

    def on_setpoint(self, msg: TrajectorySetpoint) -> None:
        # PX4 NED setpoint；NaN 表示该轴不控制，保持原目标
        if math.isfinite(msg.position[0]):
            self.sp_n = float(msg.position[0])
        if math.isfinite(msg.position[1]):
            self.sp_e = float(msg.position[1])
        if math.isfinite(msg.position[2]):
            self.sp_z = float(msg.position[2])
        self.have_sp = True

    def _blank_grid(self, res: float = 0.2, half: float = 10.0):
        width = int((2.0 * half) / res)
        height = width
        ox = -half
        oy = -half
        data = [100] * (width * height)
        return res, width, height, ox, oy, data

    def _setc(self, data, width, height, ox, oy, res, x, y, v) -> None:
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if 0 <= ix < width and 0 <= iy < height:
            data[iy * width + ix] = v

    def build_cross(self):
        res, width, height, ox, oy, data = self._blank_grid()
        for iy in range(height):
            for ix in range(width):
                x = ox + (ix + 0.5) * res
                y = oy + (iy + 0.5) * res
                data[iy * width + ix] = 0 if (abs(x) <= 1.2 or abs(y) <= 1.2) else 100
        # 东巷柱子
        for dx in (-0.3, -0.1, 0.1, 0.3):
            for dy in (-0.3, -0.1, 0.1, 0.3):
                self._setc(data, width, height, ox, oy, res, 3.0 + dx, 0.0 + dy, 100)
        # 北巷远端未知
        for iy in range(height):
            for ix in range(width):
                x = ox + (ix + 0.5) * res
                y = oy + (iy + 0.5) * res
                if abs(x) <= 1.2 and y > 6.0:
                    data[iy * width + ix] = -1
        return res, width, height, ox, oy, data

    def build_irregular(self):
        """斜向主巷 + 侧支 + 不规则空洞，专门打掉轴对齐假设。"""
        res, width, height, ox, oy, data = self._blank_grid()

        def carve_disk(cx, cy, r, v=0):
            for iy in range(height):
                for ix in range(width):
                    x = ox + (ix + 0.5) * res
                    y = oy + (iy + 0.5) * res
                    if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r:
                        data[iy * width + ix] = v

        def carve_band(ax, ay, bx, by, half_w, v=0):
            # 线段带状挖空
            dx = bx - ax
            dy = by - ay
            L = math.hypot(dx, dy)
            if L < 1e-6:
                return
            ux, uy = dx / L, dy / L
            nx, ny = -uy, ux
            for iy in range(height):
                for ix in range(width):
                    x = ox + (ix + 0.5) * res
                    y = oy + (iy + 0.5) * res
                    px, py = x - ax, y - ay
                    t = px * ux + py * uy
                    if t < -0.2 or t > L + 0.2:
                        continue
                    d = abs(px * nx + py * ny)
                    if d <= half_w:
                        data[iy * width + ix] = v

        # 斜向主通道：西南 -> 东北（故意不走轴对齐）
        # 半宽 1.6：给 0.5m 机体半径后仍有约 2.2m 净宽
        carve_band(-7.0, -6.0, 7.0, 6.5, 1.60, 0)
        # 侧支：斜穿后往北偏东，给未知前沿；必须比机体直径宽
        carve_band(0.2, 0.0, 3.8, 7.2, 1.55, 0)
        # 西侧不规则腔体
        carve_disk(-4.5, 1.5, 1.8, 0)
        carve_disk(-3.2, 3.0, 1.2, 0)
        # 东侧偏置障碍（柱/石堆），逼局部任意方向绕；别堵死主通道
        carve_disk(2.8, 0.4, 0.40, 100)
        carve_disk(4.5, 2.0, 0.35, 100)
        # 东北远端留未知，给 explore 找前沿
        for iy in range(height):
            for ix in range(width):
                x = ox + (ix + 0.5) * res
                y = oy + (iy + 0.5) * res
                if data[iy * width + ix] == 0 and y > 5.2 and x > 0.8:
                    data[iy * width + ix] = -1
        # 起点附近保证可站，并连上主通道/侧支
        carve_disk(0.0, 0.0, 1.6, 0)
        carve_band(0.0, 0.0, 2.2, 2.0, 1.40, 0)
        return res, width, height, ox, oy, data

    def build_map(self) -> None:
        if self.map_type == 'cross':
            res, width, height, ox, oy, data = self.build_cross()
        else:
            self.map_type = 'irregular'
            res, width, height, ox, oy, data = self.build_irregular()

        g = OccupancyGrid()
        g.header.frame_id = 'camera_init'
        g.info.resolution = res
        g.info.width = width
        g.info.height = height
        g.info.origin.position.x = ox
        g.info.origin.position.y = oy
        g.info.origin.orientation.w = 1.0
        g.data = data
        self.grid = g

    def step_toward(self) -> None:
        if not (self.follow_setpoint and self.have_sp):
            return
        dn = self.sp_n - self.n
        de = self.sp_e - self.e
        dz = self.sp_z - self.z
        dist = math.sqrt(dn * dn + de * de + dz * dz)
        if dist < 1e-4:
            self.n = self.sp_n
            self.e = self.sp_e
            self.z = self.sp_z
            return
        step = min(self.max_speed * self.dt, dist)
        k = step / dist
        self.n += dn * k
        self.e += de * k
        self.z += dz * k

    def reveal_around(self) -> None:
        """飞机当前位置附近：unknown(-1) -> free(0)。占用(100)保持不动。"""
        if self.reveal_radius <= 1e-6:
            return
        # 飞控 NED -> map(ENU)：map_x = ned_y(=E), map_y = ned_x(=N)
        cx = self.e
        cy = self.n
        res = float(self.grid.info.resolution)
        ox = float(self.grid.info.origin.position.x)
        oy = float(self.grid.info.origin.position.y)
        w = int(self.grid.info.width)
        h = int(self.grid.info.height)
        r = self.reveal_radius
        r2 = r * r
        ix0 = max(0, int((cx - r - ox) / res) - 1)
        ix1 = min(w - 1, int((cx + r - ox) / res) + 1)
        iy0 = max(0, int((cy - r - oy) / res) - 1)
        iy1 = min(h - 1, int((cy + r - oy) / res) + 1)
        data = self.grid.data
        changed = 0
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                x = ox + (ix + 0.5) * res
                y = oy + (iy + 0.5) * res
                if (x - cx) * (x - cx) + (y - cy) * (y - cy) > r2:
                    continue
                idx = iy * w + ix
                if data[idx] < 0:
                    data[idx] = 0
                    changed += 1
        if changed:
            # OccupancyGrid.data 是 list，改完要写回
            self.grid.data = data

    def tick(self) -> None:
        self.step_toward()
        self.reveal_around()

        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.grid)

        msg = VehicleLocalPosition()
        msg.timestamp = int(time.time() * 1e6)
        msg.xy_valid = True
        msg.z_valid = True
        msg.x = float(self.n)
        msg.y = float(self.e)
        msg.z = float(self.z)
        self.pos_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
