#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav_pkg
{

struct GridMap
{
  bool ready{false};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};
  std::string frame_id{"map"};
  std::vector<int8_t> data;

  void update(const nav_msgs::msg::OccupancyGrid & msg)
  {
    resolution = msg.info.resolution;
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    width = static_cast<int>(msg.info.width);
    height = static_cast<int>(msg.info.height);
    frame_id = msg.header.frame_id;
    data = msg.data;
    ready = width > 0 && height > 0 && resolution > 1e-6 &&
      static_cast<int>(data.size()) == width * height;
  }

  bool world_to_map(double x, double y, int & mx, int & my) const
  {
    if (!ready) {
      return false;
    }
    mx = static_cast<int>(std::floor((x - origin_x) / resolution));
    my = static_cast<int>(std::floor((y - origin_y) / resolution));
    return mx >= 0 && my >= 0 && mx < width && my < height;
  }

  void map_to_world(int mx, int my, double & x, double & y) const
  {
    x = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
    y = origin_y + (static_cast<double>(my) + 0.5) * resolution;
  }

  int8_t cell(int mx, int my) const
  {
    if (!ready || mx < 0 || my < 0 || mx >= width || my >= height) {
      return 100;
    }
    return data[my * width + mx];
  }

  // 占用：>= occ_threshold；未知是否算障碍由 treat_unknown_as_occ 决定
  bool is_occ(int mx, int my, int occ_threshold, bool treat_unknown_as_occ) const
  {
    const int8_t v = cell(mx, my);
    if (v < 0) {
      return treat_unknown_as_occ;
    }
    return v >= occ_threshold;
  }

  bool is_occ_world(
    double x, double y, int occ_threshold, bool treat_unknown_as_occ) const
  {
    int mx = 0;
    int my = 0;
    if (!world_to_map(x, y, mx, my)) {
      return true;
    }
    return is_occ(mx, my, occ_threshold, treat_unknown_as_occ);
  }

  // 线段采样碰撞检测（世界坐标）
  bool segment_free(
    double x0, double y0, double x1, double y1,
    int occ_threshold, bool treat_unknown_as_occ) const
  {
    if (!ready) {
      return false;
    }
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    const double step = std::max(resolution * 0.5, 1e-3);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      const double x = x0 + t * dx;
      const double y = y0 + t * dy;
      if (is_occ_world(x, y, occ_threshold, treat_unknown_as_occ)) {
        return false;
      }
    }
    return true;
  }
};

// 圆形膨胀：把占用格向外扩 radius_m
inline nav_msgs::msg::OccupancyGrid inflate_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid & in,
  double radius_m,
  int occ_threshold,
  bool inflate_unknown)
{
  nav_msgs::msg::OccupancyGrid out = in;
  if (in.info.width == 0 || in.info.height == 0 || in.info.resolution <= 1e-6) {
    return out;
  }

  const int w = static_cast<int>(in.info.width);
  const int h = static_cast<int>(in.info.height);
  const int r_cells = std::max(0, static_cast<int>(std::ceil(radius_m / in.info.resolution)));
  if (r_cells == 0) {
    return out;
  }

  out.data.assign(in.data.size(), 0);
  const int r2 = r_cells * r_cells;

  // 先拷贝未知，再写膨胀占用，避免未知被冲掉后无法区分
  for (size_t i = 0; i < in.data.size(); ++i) {
    if (in.data[i] < 0) {
      out.data[i] = -1;
    }
  }

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int8_t v = in.data[y * w + x];
      const bool seed_occ = (v >= occ_threshold) || (inflate_unknown && v < 0);
      if (!seed_occ) {
        continue;
      }
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          if (dx * dx + dy * dy > r2) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
            continue;
          }
          out.data[ny * w + nx] = 100;
        }
      }
    }
  }

  // 原图 free 且未被膨胀覆盖的保持 0；原占用已被写成 100
  // 原未知若未被膨胀覆盖，保持 -1
  return out;
}

}  // namespace nav_pkg
