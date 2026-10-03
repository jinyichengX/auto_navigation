#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "nav/grid_map.hpp"

namespace nav_pkg
{

struct RRTConfig
{
  double step_size{0.40};
  double goal_bias{0.20};
  double goal_tolerance{0.35};
  int max_iterations{4000};
  int occ_threshold{50};
  bool treat_unknown_as_occ{true};
  // 起点若落在膨胀里，允许向周围搜最近自由格
  double start_snap_radius{1.0};
  double goal_snap_radius{1.5};
};

struct RRTResult
{
  bool ok{false};
  std::string message;
  std::vector<std::pair<double, double>> path;  // world xy
};

class RRTPlanner
{
public:
  explicit RRTPlanner(RRTConfig cfg = {})
  : cfg_(std::move(cfg)), gen_(rd_())
  {
  }

  void set_config(const RRTConfig & cfg) { cfg_ = cfg; }

  RRTResult plan(
    const GridMap & map,
    double start_x, double start_y,
    double goal_x, double goal_y)
  {
    RRTResult res;
    if (!map.ready) {
      res.message = "map not ready";
      return res;
    }

    double sx = start_x;
    double sy = start_y;
    double gx = goal_x;
    double gy = goal_y;

    if (!snap_to_free(map, sx, sy, cfg_.start_snap_radius)) {
      res.message = "start in obstacle and snap failed";
      return res;
    }
    if (!snap_to_free(map, gx, gy, cfg_.goal_snap_radius)) {
      res.message = "goal in obstacle and snap failed";
      return res;
    }

    // 直连优先
    if (map.segment_free(
        sx, sy, gx, gy, cfg_.occ_threshold, cfg_.treat_unknown_as_occ))
    {
      res.ok = true;
      res.message = "direct";
      res.path = {{sx, sy}, {gx, gy}};
      return res;
    }

    struct Node
    {
      double x{0.0};
      double y{0.0};
      int parent{-1};
    };

    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(cfg_.max_iterations) + 2);
    nodes.push_back(Node{sx, sy, -1});

    const double x_min = map.origin_x;
    const double y_min = map.origin_y;
    const double x_max = map.origin_x + map.width * map.resolution;
    const double y_max = map.origin_y + map.height * map.resolution;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> dist_x(x_min, x_max);
    std::uniform_real_distribution<double> dist_y(y_min, y_max);

    int goal_idx = -1;
    for (int it = 0; it < cfg_.max_iterations; ++it) {
      double rx = 0.0;
      double ry = 0.0;
      if (unit(gen_) < cfg_.goal_bias) {
        rx = gx;
        ry = gy;
      } else {
        rx = dist_x(gen_);
        ry = dist_y(gen_);
      }

      int nearest = 0;
      double best_d2 = dist2(nodes[0].x, nodes[0].y, rx, ry);
      for (size_t i = 1; i < nodes.size(); ++i) {
        const double d2 = dist2(nodes[i].x, nodes[i].y, rx, ry);
        if (d2 < best_d2) {
          best_d2 = d2;
          nearest = static_cast<int>(i);
        }
      }

      const double ndx = rx - nodes[nearest].x;
      const double ndy = ry - nodes[nearest].y;
      const double nd = std::hypot(ndx, ndy);
      if (nd < 1e-6) {
        continue;
      }
      const double scale = std::min(cfg_.step_size, nd) / nd;
      const double nx = nodes[nearest].x + ndx * scale;
      const double ny = nodes[nearest].y + ndy * scale;

      if (!map.segment_free(
          nodes[nearest].x, nodes[nearest].y, nx, ny,
          cfg_.occ_threshold, cfg_.treat_unknown_as_occ))
      {
        continue;
      }

      nodes.push_back(Node{nx, ny, nearest});
      const int new_idx = static_cast<int>(nodes.size()) - 1;
      if (std::hypot(nx - gx, ny - gy) <= cfg_.goal_tolerance) {
        // 尝试直接接到真正目标
        if (map.segment_free(
            nx, ny, gx, gy, cfg_.occ_threshold, cfg_.treat_unknown_as_occ))
        {
          nodes.push_back(Node{gx, gy, new_idx});
          goal_idx = static_cast<int>(nodes.size()) - 1;
        } else {
          goal_idx = new_idx;
        }
        break;
      }
    }

    if (goal_idx < 0) {
      res.message = "rrt failed to reach goal";
      return res;
    }

    std::vector<std::pair<double, double>> raw;
    for (int i = goal_idx; i >= 0; i = nodes[i].parent) {
      raw.emplace_back(nodes[i].x, nodes[i].y);
      if (nodes[i].parent < 0) {
        break;
      }
    }
    std::reverse(raw.begin(), raw.end());
    res.path = shortcut(map, raw);
    res.ok = res.path.size() >= 2;
    res.message = res.ok ? "rrt ok" : "path too short";
    return res;
  }

private:
  static double dist2(double x0, double y0, double x1, double y1)
  {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return dx * dx + dy * dy;
  }

  bool snap_to_free(const GridMap & map, double & x, double & y, double radius) const
  {
    if (!map.is_occ_world(x, y, cfg_.occ_threshold, cfg_.treat_unknown_as_occ)) {
      return true;
    }
    const double step = std::max(map.resolution, 0.05);
    const int rings = std::max(1, static_cast<int>(std::ceil(radius / step)));
    for (int r = 1; r <= rings; ++r) {
      const double rad = r * step;
      const int samples = std::max(8, r * 8);
      for (int i = 0; i < samples; ++i) {
        const double ang = 2.0 * M_PI * static_cast<double>(i) / samples;
        const double nx = x + rad * std::cos(ang);
        const double ny = y + rad * std::sin(ang);
        if (!map.is_occ_world(nx, ny, cfg_.occ_threshold, cfg_.treat_unknown_as_occ)) {
          x = nx;
          y = ny;
          return true;
        }
      }
    }
    return false;
  }

  std::vector<std::pair<double, double>> shortcut(
    const GridMap & map,
    const std::vector<std::pair<double, double>> & path) const
  {
    if (path.size() <= 2) {
      return path;
    }
    std::vector<std::pair<double, double>> out;
    out.push_back(path.front());
    size_t i = 0;
    while (i + 1 < path.size()) {
      size_t best = i + 1;
      for (size_t j = path.size() - 1; j > i + 1; --j) {
        if (map.segment_free(
            path[i].first, path[i].second,
            path[j].first, path[j].second,
            cfg_.occ_threshold, cfg_.treat_unknown_as_occ))
        {
          best = j;
          break;
        }
      }
      out.push_back(path[best]);
      i = best;
    }
    return out;
  }

  RRTConfig cfg_;
  std::random_device rd_;
  std::mt19937 gen_;
};

}  // namespace nav_pkg
