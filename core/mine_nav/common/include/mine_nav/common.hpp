#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mine_nav
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr int kOccThresh = 50;

inline double hypot2(double dx, double dy)
{
  return std::sqrt(dx * dx + dy * dy);
}

// camera_init(ENU) <-> PX4 local NED
// map_x = ned_y , map_y = ned_x
inline std::pair<double, double> ned_to_map(double n, double e)
{
  return {e, n};
}

inline std::pair<double, double> map_to_ned(double mx, double my)
{
  return {my, mx};
}

inline bool point_in_rect(
  double x, double y,
  const geometry_msgs::msg::Point & mn,
  const geometry_msgs::msg::Point & mx)
{
  return x >= mn.x && x <= mx.x && y >= mn.y && y <= mx.y;
}

inline bool point_in_any_nofly(
  double x, double y,
  const std::vector<geometry_msgs::msg::Point> & zones)
{
  // zones: [min0,max0,min1,max1,...]
  if (zones.size() < 2) {
    return false;
  }
  for (size_t i = 0; i + 1 < zones.size(); i += 2) {
    if (point_in_rect(x, y, zones[i], zones[i + 1])) {
      return true;
    }
  }
  return false;
}

struct GridMap
{
  bool ready{false};
  double res{0.1};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;

  void update(const nav_msgs::msg::OccupancyGrid & msg)
  {
    res = msg.info.resolution;
    width = static_cast<int>(msg.info.width);
    height = static_cast<int>(msg.info.height);
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    data = msg.data;
    ready = width > 0 && height > 0 &&
      static_cast<int>(data.size()) == width * height;
  }

  bool world_to_idx(double x, double y, int & ix, int & iy) const
  {
    ix = static_cast<int>(std::floor((x - origin_x) / res));
    iy = static_cast<int>(std::floor((y - origin_y) / res));
    return ix >= 0 && iy >= 0 && ix < width && iy < height;
  }

  int cell_value(double x, double y) const
  {
    if (!ready) {
      return -1;
    }
    int ix = 0;
    int iy = 0;
    if (!world_to_idx(x, y, ix, iy)) {
      return -1;
    }
    return data[iy * width + ix];
  }

  bool is_occupied(double x, double y, double robot_radius) const
  {
    if (!ready) {
      return true;
    }
    const int r = std::max(0, static_cast<int>(std::ceil(robot_radius / res)));
    int cx = 0;
    int cy = 0;
    if (!world_to_idx(x, y, cx, cy)) {
      return true;
    }
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) {
          continue;
        }
        const int ix = cx + dx;
        const int iy = cy + dy;
        if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
          return true;
        }
        if (data[iy * width + ix] > kOccThresh) {
          return true;
        }
      }
    }
    return false;
  }

  bool is_unknown_near(double x, double y, double robot_radius) const
  {
    if (!ready) {
      return true;
    }
    const int r = std::max(0, static_cast<int>(std::ceil(robot_radius / res)));
    int cx = 0;
    int cy = 0;
    if (!world_to_idx(x, y, cx, cy)) {
      return true;
    }
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) {
          continue;
        }
        const int ix = cx + dx;
        const int iy = cy + dy;
        if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
          return true;
        }
        if (data[iy * width + ix] < 0) {
          return true;
        }
      }
    }
    return false;
  }
};

inline geometry_msgs::msg::PoseStamped make_pose(
  const std_msgs::msg::Header & header,
  double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped p;
  p.header = header;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;
  return p;
}

inline nav_msgs::msg::Path densify_path(
  const nav_msgs::msg::Path & in, double step)
{
  nav_msgs::msg::Path out;
  out.header = in.header;
  if (in.poses.empty()) {
    return out;
  }
  out.poses.push_back(in.poses.front());
  for (size_t i = 1; i < in.poses.size(); ++i) {
    const auto & a = out.poses.back().pose.position;
    const auto & b = in.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dist = hypot2(dx, dy);
    if (dist < 1e-6) {
      continue;
    }
    const int n = std::max(1, static_cast<int>(dist / step));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      out.poses.push_back(
        make_pose(in.header, a.x + dx * t, a.y + dy * t, b.z));
    }
  }
  return out;
}

/**
 * 在目标附近找最近可走点。
 * allow_fn(x,y)=true 表示该点允许作为目标（范围/禁飞/占用都过）。
 * 返回 true 时 out_x/out_y 是找到的点；找不到返回 false。
 */
inline bool find_nearest_free(
  double goal_x, double goal_y,
  double search_radius, double step,
  const std::function<bool(double, double)> & allow_fn,
  double & out_x, double & out_y)
{
  if (allow_fn(goal_x, goal_y)) {
    out_x = goal_x;
    out_y = goal_y;
    return true;
  }
  if (step < 1e-3) {
    step = 0.2;
  }
  double best_d = std::numeric_limits<double>::infinity();
  bool found = false;
  const int rings = std::max(1, static_cast<int>(std::ceil(search_radius / step)));
  for (int r = 1; r <= rings; ++r) {
    const double rad = r * step;
    // 每圈采样点数随半径增大，保证覆盖
    const int samples = std::max(8, r * 8);
    for (int i = 0; i < samples; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / samples;
      const double x = goal_x + rad * std::cos(ang);
      const double y = goal_y + rad * std::sin(ang);
      if (!allow_fn(x, y)) {
        continue;
      }
      const double d = hypot2(x - goal_x, y - goal_y);
      if (d < best_d) {
        best_d = d;
        out_x = x;
        out_y = y;
        found = true;
      }
    }
    if (found) {
      return true;  // 优先最近一圈
    }
  }
  return found;
}

}  // namespace mine_nav
