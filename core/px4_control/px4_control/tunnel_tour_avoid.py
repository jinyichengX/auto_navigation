#!/usr/bin/env python3
"""带局部避障的巷道航点遍历。

看 /projected_map（占用栅格，不是颜色）：
  - 墙、立柱、红柱，只要占格都算障碍
  - 未知格也当危险，不能当可走
  - 窄巷只做：先侧移 -> 平行越过 -> 再回中心线
  - 禁止斜着切角飞，避免贴侧墙

坐标系：
  - 飞控位置是 NED：x北 y东 z下
  - /projected_map 来自 FAST-LIO 的 camera_init（ENU）
  - 换算：map_x = ned_y ，map_y = ned_x
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleCommandAck,
    VehicleLocalPosition,
)

NAN = float('nan')
PUB_HZ = 20
TICK = 1.0 / PUB_HZ
MAX_SPEED = 0.40
ARRIVE_DIST = 0.40
HOLD_SEC = 1.0
FLY_ALT = -1.2
ARM_REACH = 7.0
SCAN_HOLD_SEC = 6.0
SCAN_HOLD_MAX_SEC = 12.0

# 避障参数
# 仿真 x500 + Mid-360：水平安全半径约 0.55 m（含桨叶余量）
ROBOT_RADIUS = 0.55
OCC_THRESH = 50
# 巷道半宽约 2.0 m；交叉口立柱外表面约 ±1.6 m
# 侧移过大就会贴立柱/侧墙，所以只给小侧移
# 红柱偏东约 0.7 m，西侧留通道：侧移 0.70~0.85 足够绕开
AVOID_LATERALS = (0.70, 0.85)
LOOKAHEAD = 4.0
PASS_AHEAD = 2.0          # 侧移后，沿走廊再往前探这么远
REPLAN_PERIOD = 0.35
CENTER_TOL = 0.20         # 小于这个就当已在中心线附近
# 未知格：近处当危险；远处先放过，否则没扫过的巷道永远进不去
# 真正的墙/柱是 occupied，一律挡路
UNKNOWN_NEAR = 1.8

WAYPOINTS = [
    ((0.0, 0.0, FLY_ALT), '起飞到交叉口上方'),
    ((ARM_REACH, 0.0, FLY_ALT), '去北端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((0.0, ARM_REACH, FLY_ALT), '去东端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((-ARM_REACH, 0.0, FLY_ALT), '去南端'),
    ((0.0, 0.0, FLY_ALT), '回交叉口'),
    ((0.0, -ARM_REACH, FLY_ALT), '去西端'),
    ((0.0, 0.0, FLY_ALT), '回原点准备降落'),
]

RESULT_NAME = {
    0: 'ACCEPTED 已接受',
    1: 'TEMPORARILY_REJECTED 暂时拒绝',
    2: 'DENIED 拒绝',
    3: 'UNSUPPORTED 不支持此命令',
    4: 'FAILED 执行失败',
}


def step_toward(cur, tgt, max_step):
    d = [tgt[i] - cur[i] for i in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    if dist <= max_step or dist == 0.0:
        return [float(v) for v in tgt], [0.0, 0.0, 0.0]
    s = max_step / dist
    new_sp = [cur[i] + d[i] * s for i in range(3)]
    ff_vel = [d[i] / dist * MAX_SPEED for i in range(3)]
    return new_sp, ff_vel


def ned_to_map(x, y):
    # camera_init(ENU) <- NED
    return y, x


def map_to_ned(mx, my, z=FLY_ALT):
    return my, mx, z


class GridMap:
    def __init__(self):
        self.ready = False
        self.res = 0.1
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.data = []

    def update(self, msg: OccupancyGrid):
        self.res = float(msg.info.resolution)
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = list(msg.data)
        self.ready = self.width > 0 and self.height > 0 and len(self.data) == self.width * self.height

    def world_to_idx(self, x, y):
        ix = int((x - self.origin_x) / self.res)
        iy = int((y - self.origin_y) / self.res)
        return ix, iy

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def cell_value(self, x, y):
        if not self.ready:
            return -1
        ix, iy = self.world_to_idx(x, y)
        if not self.in_bounds(ix, iy):
            return -1
        return self.data[iy * self.width + ix]

    def count_cells(self):
        if not self.ready:
            return 0, 0, 0
        occ = free = unk = 0
        for v in self.data:
            if v < 0:
                unk += 1
            elif v > OCC_THRESH:
                occ += 1
            else:
                free += 1
        return occ, free, unk

    def is_blocked_cell(self, x, y, inflate=0.0, treat_unknown=False):
        """占用格一律危险；未知/地图外只在 treat_unknown=True 时算危险。"""
        if not self.ready:
            return True
        r = max(0, int(math.ceil((inflate + ROBOT_RADIUS) / self.res)))
        cx, cy = self.world_to_idx(x, y)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                ix, iy = cx + dx, cy + dy
                if not self.in_bounds(ix, iy):
                    if treat_unknown:
                        return True
                    continue
                v = self.data[iy * self.width + ix]
                if v > OCC_THRESH:
                    return True
                if v < 0 and treat_unknown:
                    return True
        return False

    def segment_blocked(self, x0, y0, x1, y1, step=None):
        """返回 (blocked, hit_xy, reason, stats)。
        occupied（墙/柱/红柱）一律挡；未知只挡近处。
        """
        if not self.ready:
            return True, None, 'no_map', {'occ': 0, 'unk': 0, 'free': 0}
        if step is None:
            step = max(0.1, self.res * 0.8)
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        stats = {'occ': 0, 'unk': 0, 'free': 0}
        if dist < 1e-6:
            samples = [(x0, y0, 0.0)]
        else:
            n = max(1, int(dist / step))
            samples = []
            for i in range(n + 1):
                t = i / n
                samples.append((x0 + dx * t, y0 + dy * t, dist * t))

        for x, y, along in samples:
            v = self.cell_value(x, y)
            if v < 0:
                stats['unk'] += 1
            elif v > OCC_THRESH:
                stats['occ'] += 1
            else:
                stats['free'] += 1

            # 墙/柱等占用：一律挡
            if self.is_blocked_cell(x, y, treat_unknown=False):
                return True, (x, y), 'occupied', stats
            # 未知：只挡近处，避免贴着没扫清的侧墙硬闯
            if along <= UNKNOWN_NEAR and self.is_blocked_cell(x, y, treat_unknown=True):
                return True, (x, y), 'unknown_near', stats
        return False, None, 'clear', stats


class TunnelTourAvoid(Node):
    def __init__(self):
        super().__init__('tunnel_tour_avoid')

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
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_in)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_in)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_in)
        self.path_pub = self.create_publisher(Path, '/avoid_path', 10)

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos_out)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack, qos_out)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.on_local_pos,
            qos_out,
        )
        self.create_subscription(OccupancyGrid, '/projected_map', self.on_map, qos_map)

        self.grid = GridMap()
        self.n = 0
        self.wp_index = 0
        self.goal = None
        self.local_target = None
        self.avoid_active = False
        self.lane_lat = 0.0          # 当前选用的侧移量（NED 侧向坐标）
        self.sp = [0.0, 0.0, 0.0]
        self.sp_vel = [0.0, 0.0, 0.0]
        self.sp_init = False
        self.hold_ticks = 0
        self.phase = 'preheat'
        self.armed_sent = False
        self.land_sent = False
        self.disarm_sent = False
        self.done = False
        self.land_tick = None
        self.scan_start_tick = None
        self.last_replan = 0.0
        self.last_diag = 0.0
        self.arming_state = -1
        self.nav_state = -1
        self.x = NAN
        self.y = NAN
        self.z = NAN

        self.create_timer(TICK, self.tick)
        self.get_logger().info(
            f'开始带避障遍历：四臂各到 {ARM_REACH:.0f} m，高度 {-FLY_ALT:.1f} m；'
            f'墙/柱/未知都算障碍；贴走廊侧移绕障，不斜着切角'
        )

    def _leg_axes(self):
        """当前航段：前进轴 / 侧向轴。返回 ('x'|'y', 'x'|'y')，都是 NED。"""
        if self.goal is None or math.isnan(self.x):
            return 'x', 'y'
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        if abs(dx) >= abs(dy):
            return 'x', 'y'   # 南北巷：前进北，侧移东
        return 'y', 'x'       # 东西巷：前进东，侧移北

    def _get_axis(self, n, e, axis):
        return n if axis == 'x' else e

    def _set_axes(self, fwd_val, lat_val, fwd_axis, lat_axis):
        if fwd_axis == 'x':
            return [fwd_val, lat_val, FLY_ALT]
        return [lat_val, fwd_val, FLY_ALT]

    def tick(self):
        if self.done:
            return

        self.n += 1
        elapsed = self.n * TICK

        if not self.sp_init:
            if math.isnan(self.z):
                return
            self.sp = [self.x, self.y, self.z]
            self.sp_init = True

        if self.phase == 'preheat':
            if not self.armed_sent and elapsed >= 2.0:
                self.armed_sent = True
                self.get_logger().info('切换到 OFFBOARD 模式，并解锁')
                self.send_command(100001, 14.0)
                self.send_command(400, 1.0)
                self._set_waypoint(0)
                self.phase = 'takeoff'

        elif self.phase == 'takeoff':
            self.local_target = self.goal
            self.sp, self.sp_vel = step_toward(self.sp, self.local_target, MAX_SPEED * TICK)
            remain = math.dist([self.x, self.y, self.z], self.goal)
            if remain <= ARRIVE_DIST:
                self.sp_vel = [0.0, 0.0, 0.0]
                if self.scan_start_tick is None:
                    self.scan_start_tick = self.n
                    self.get_logger().info(
                        f'起飞到位，悬停建图 {SCAN_HOLD_SEC:.0f}s（等墙/柱进 /projected_map）'
                    )
                held = (self.n - self.scan_start_tick) * TICK
                occ, free, unk = self.grid.count_cells()
                if self.n % PUB_HZ == 0:
                    self.get_logger().info(
                        f'建图中 {held:.1f}/{SCAN_HOLD_SEC:.0f}s | 地图格 occupied={occ} free={free} unknown={unk}'
                    )
                map_useful = (occ > 0) or (free > 0)
                if held >= SCAN_HOLD_SEC and map_useful:
                    self.get_logger().info(
                        f'建图等待结束（occ={occ}, free={free}），开始带避障遍历'
                    )
                    self.phase = 'fly'
                    self._set_waypoint(1)
                elif held >= SCAN_HOLD_MAX_SEC:
                    self.get_logger().warn(
                        f'建图等待超时（occ={occ}, free={free}, unk={unk}），先继续飞；'
                        f'请确认 octomap / cloud_registered_body 正常'
                    )
                    self.phase = 'fly'
                    self._set_waypoint(1)

        elif self.phase == 'fly':
            now = time.time()
            if now - self.last_replan >= REPLAN_PERIOD:
                self.last_replan = now
                self._replan_local_target()

            if self.local_target is not None:
                self.sp, self.sp_vel = step_toward(self.sp, self.local_target, MAX_SPEED * TICK)
                remain_local = math.dist([self.x, self.y, self.z], self.local_target)
                remain_goal = math.dist([self.x, self.y, self.z], self.goal)

                if self.avoid_active and remain_local <= ARRIVE_DIST:
                    # 到了平行绕障点：继续沿当前车道再规划，不立刻斜着冲回中心
                    self.get_logger().info('绕障段到位，继续沿当前车道检查')
                    self.hold_ticks = 0
                    self._replan_local_target(force=True)
                elif (not self.avoid_active) and remain_goal <= ARRIVE_DIST:
                    self.hold_ticks += 1
                    self.sp_vel = [0.0, 0.0, 0.0]
                    if self.hold_ticks >= int(HOLD_SEC / TICK):
                        next_idx = self.wp_index + 1
                        if next_idx < len(WAYPOINTS):
                            self._set_waypoint(next_idx)
                        else:
                            self.get_logger().info('航点全部完成，准备降落')
                            self.phase = 'land'
                else:
                    self.hold_ticks = 0

        elif self.phase == 'land':
            if not self.land_sent:
                self.land_sent = True
                self.land_tick = self.n
                self.get_logger().info('发送降落命令')
                self.send_command(21, 0.0)
            elif self.n - self.land_tick >= int(12.0 / TICK):
                self.phase = 'disarm'

        elif self.phase == 'disarm':
            if not self.disarm_sent:
                self.disarm_sent = True
                self.get_logger().info('发送上锁命令')
                self.send_command(400, 0.0, param2=21196.0)
            self.done = True
            self.get_logger().info('遍历结束')

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        self.publish_path()

        if self.n % (PUB_HZ * 2) == 0:
            remain = NAN
            if self.goal is not None and not math.isnan(self.x):
                remain = math.dist([self.x, self.y, self.z], self.goal)
            mode = '绕障中' if self.avoid_active else '直飞'
            occ, free, unk = self.grid.count_cells()
            self.get_logger().info(
                f'[ {elapsed:4.0f}s | {self.phase:7s} | {mode} ] '
                f'wp={self.wp_index + 1}/{len(WAYPOINTS)} '
                f'位置 x={self.x:.2f} y={self.y:.2f} 高度={-self.z:.2f}m '
                f'| 距目标 {remain:.2f}m | 地图 occ={occ} free={free} | 车道侧移={self.lane_lat:+.2f}'
            )

    def _set_waypoint(self, index):
        self.wp_index = index
        self.goal = WAYPOINTS[index][0]
        self.local_target = self.goal
        self.avoid_active = False
        self.lane_lat = 0.0
        self.hold_ticks = 0
        self.get_logger().info(
            f'-> 航点 {index + 1}/{len(WAYPOINTS)}: {self.goal} （{WAYPOINTS[index][1]}）'
        )
        self._replan_local_target(force=True)

    def _replan_local_target(self, force=False):
        if self.goal is None or math.isnan(self.x) or not self.grid.ready:
            return

        fwd_axis, lat_axis = self._leg_axes()
        cur_fwd = self._get_axis(self.x, self.y, fwd_axis)
        cur_lat = self._get_axis(self.x, self.y, lat_axis)
        goal_fwd = self._get_axis(self.goal[0], self.goal[1], fwd_axis)
        goal_lat = self._get_axis(self.goal[0], self.goal[1], lat_axis)

        # 航点都在中心线；goal_lat 理论上是 0
        remain_fwd = goal_fwd - cur_fwd
        if abs(remain_fwd) < 1e-3 and abs(cur_lat - goal_lat) < CENTER_TOL:
            self.local_target = self.goal
            self.avoid_active = False
            self.lane_lat = 0.0
            return

        fwd_sign = 1.0 if remain_fwd >= 0 else -1.0
        check_len = min(abs(remain_fwd), LOOKAHEAD)
        if check_len < 0.2:
            check_len = abs(remain_fwd)

        # 当前车道：优先沿已选侧移；没有就用当前实际侧向
        lane = self.lane_lat if self.avoid_active else (cur_lat if abs(cur_lat) > CENTER_TOL else 0.0)

        def sample_on_lane(lane_val, ahead):
            # NED -> map，检查这条平行线
            n0, e0, _ = self._set_axes(cur_fwd, lane_val, fwd_axis, lat_axis)
            n1, e1, _ = self._set_axes(cur_fwd + fwd_sign * ahead, lane_val, fwd_axis, lat_axis)
            mx0, my0 = ned_to_map(n0, e0)
            mx1, my1 = ned_to_map(n1, e1)
            return self.grid.segment_blocked(mx0, my0, mx1, my1)

        blocked, hit, reason, stats = sample_on_lane(lane, check_len)
        now = time.time()
        if force or now - self.last_diag >= 1.0:
            self.last_diag = now
            hit_txt = '无'
            if hit is not None:
                hn, he, _ = map_to_ned(hit[0], hit[1], FLY_ALT)
                hit_txt = f'NED=({hn:.2f},{he:.2f})'
            self.get_logger().info(
                f'前方检查 {check_len:.1f}m | 车道侧移={lane:+.2f} | {reason} | '
                f'occ={stats["occ"]} free={stats["free"]} unk={stats["unk"]} | 命中={hit_txt}'
            )

        def lateral_move_blocked(from_lat, to_lat):
            """纯横移这一段是否撞墙/近处未知。"""
            n0, e0, _ = self._set_axes(cur_fwd, from_lat, fwd_axis, lat_axis)
            n1, e1, _ = self._set_axes(cur_fwd, to_lat, fwd_axis, lat_axis)
            mx0, my0 = ned_to_map(n0, e0)
            mx1, my1 = ned_to_map(n1, e1)
            blocked_lat, _, why_lat, _ = self.grid.segment_blocked(mx0, my0, mx1, my1)
            return blocked_lat, why_lat

        # 当前车道通畅：能安全回中心就回中心，否则继续贴当前车道平行飞
        if not blocked:
            center_blocked, _, _, _ = sample_on_lane(0.0, min(check_len, PASS_AHEAD))
            want_center = abs(lane) <= CENTER_TOL or not center_blocked
            if want_center and abs(cur_lat) > CENTER_TOL:
                lat_blocked, lat_why = lateral_move_blocked(cur_lat, 0.0)
                if lat_blocked:
                    # 横移回中心会撞墙/柱：继续沿当前车道平行飞，绝不斜切
                    ahead = min(abs(remain_fwd), PASS_AHEAD)
                    self.local_target = self._set_axes(
                        cur_fwd + fwd_sign * max(ahead, 1.0), lane, fwd_axis, lat_axis
                    )
                    self.avoid_active = True
                    self.lane_lat = lane
                    if force:
                        self.get_logger().info(
                            f'中心方向暂不通({lat_why})，继续沿侧道平行飞'
                        )
                    return
                # 先横向回到中心线（前进坐标不变），禁止斜着切回
                new_target = self._set_axes(cur_fwd, 0.0, fwd_axis, lat_axis)
                changed = (
                    force
                    or self.local_target is None
                    or math.dist(self.local_target, new_target) > 0.20
                    or abs(self.lane_lat) > 0.05
                )
                self.local_target = new_target
                self.avoid_active = True
                self.lane_lat = 0.0
                if changed:
                    self.get_logger().info('前方已通，先回到中心线')
                return
            if want_center:
                # 已在中心线附近：只沿前进轴冲目标，侧向保持 goal 中心
                self.local_target = self._set_axes(goal_fwd, goal_lat, fwd_axis, lat_axis)
                self.avoid_active = False
                self.lane_lat = 0.0
                return
            # 中心仍不通：继续沿当前车道平行前进
            ahead = min(abs(remain_fwd), PASS_AHEAD)
            self.local_target = self._set_axes(
                cur_fwd + fwd_sign * max(ahead, 1.0), lane, fwd_axis, lat_axis
            )
            self.avoid_active = True
            self.lane_lat = lane
            return

        # 当前车道被挡：找左右平行通道
        candidates = []
        fail_reasons = []
        for lat in AVOID_LATERALS:
            for sign in (1.0, -1.0):
                side = lat * sign
                # 落点本身不能碰墙；近处未知也算危险
                n_side, e_side, _ = self._set_axes(cur_fwd, side, fwd_axis, lat_axis)
                mx_s, my_s = ned_to_map(n_side, e_side)
                if self.grid.is_blocked_cell(mx_s, my_s, treat_unknown=True):
                    fail_reasons.append(f'侧移{side:+.2f}落点危险')
                    continue
                # 横移过去这一段也不能撞墙/近处未知
                blocked_side, why_side = lateral_move_blocked(cur_lat, side)
                if blocked_side:
                    fail_reasons.append(f'侧移{side:+.2f}去路{why_side}')
                    continue
                # 新车道前方
                still_blocked, _, why_fwd, fwd_stats = sample_on_lane(
                    side, min(LOOKAHEAD, max(PASS_AHEAD, check_len))
                )
                score = abs(side)
                if still_blocked and why_fwd == 'occupied':
                    score += 10.0 + 0.05 * fwd_stats.get('occ', 0)
                    fail_reasons.append(f'侧移{side:+.2f}后前方仍{why_fwd}')
                    candidates.append((side, score, True, why_fwd))
                    continue
                candidates.append((side, score, False, 'clear'))

        if not candidates:
            self.local_target = [self.x, self.y, FLY_ALT]
            self.avoid_active = True
            detail = '；'.join(fail_reasons[:4]) if fail_reasons else '无候选'
            self.get_logger().warn(
                f'前方有障({reason})，左右都绕不过，先悬停 | {detail}'
            )
            return

        candidates.sort(key=lambda c: (c[2], c[1]))  # 先能通的，再侧移小的
        side, _, still_blocked, how = candidates[0]
        ahead = min(abs(remain_fwd), PASS_AHEAD)
        prev_target = self.local_target
        prev_lane = self.lane_lat
        # 关键：先横移到侧道；到位后再沿侧道平行前进，禁止斜着切角
        if abs(cur_lat - side) > 0.15:
            self.local_target = self._set_axes(cur_fwd, side, fwd_axis, lat_axis)
            tip = '先侧移'
        else:
            self.local_target = self._set_axes(
                cur_fwd + fwd_sign * max(ahead, 1.0), side, fwd_axis, lat_axis
            )
            tip = '沿侧道前进'
        self.avoid_active = True
        self.lane_lat = side
        changed = (
            force
            or prev_target is None
            or abs(prev_lane - side) > 0.05
            or math.dist(prev_target, self.local_target) > 0.25
        )
        if changed:
            tip2 = '可绕过' if not still_blocked else f'先靠近通道({how})'
            self.get_logger().info(
                f'检测到障碍({reason})，{tip}/{tip2}：侧移 {side:+.2f}m -> '
                f'NED=({self.local_target[0]:.2f},{self.local_target[1]:.2f})'
            )

    def publish_path(self):
        if self.goal is None or math.isnan(self.x):
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        pts = [(self.x, self.y)]
        if self.local_target is not None:
            pts.append((self.local_target[0], self.local_target[1]))
        pts.append((self.goal[0], self.goal[1]))
        for nx, ny in pts:
            mx, my = ned_to_map(nx, ny)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(mx)
            p.pose.position.y = float(my)
            p.pose.position.z = float(-FLY_ALT)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
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

    def on_map(self, msg):
        self.grid.update(msg)

    def on_ack(self, msg):
        name = RESULT_NAME.get(msg.result, f'未知({msg.result})')
        self.get_logger().info(f'命令回执 -> command={msg.command} result={msg.result} ({name})')


def main():
    rclpy.init()
    node = TunnelTourAvoid()
    end_time = time.time() + 600.0
    while rclpy.ok() and time.time() < end_time and not node.done:
        rclpy.spin_once(node, timeout_sec=TICK)
    if not node.done:
        node.get_logger().warn('超时退出，请检查飞机状态')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
