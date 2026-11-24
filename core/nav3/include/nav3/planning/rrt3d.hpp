#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace nav3
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double Dist3(const Vec3 & a, const Vec3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct RRT3DConfig
{
  double step_size{0.50};
  double goal_bias{0.20};
  double goal_tolerance{0.40};
  int max_iterations{6000};
  double robot_radius{0.35};
  bool treat_unknown_as_occ{true};
  double start_snap_radius{1.0};
  double goal_snap_radius{1.5};
  // 采样边界：未设置时从 OcTree BBX 取
  bool use_custom_bounds{false};
  Vec3 bmin{};
  Vec3 bmax{};
};

struct RRT3DResult
{
  bool ok{false};
  std::string message;
  std::vector<Vec3> path;
};

class RRT3DPlanner
{
public:
  explicit RRT3DPlanner(RRT3DConfig cfg = {})
  : cfg_(std::move(cfg)), gen_(rd_())
  {
  }

  void set_config(const RRT3DConfig & cfg) { cfg_ = cfg; }

  RRT3DResult plan(
    const octomap::OcTree & tree,
    const Vec3 & start_in,
    const Vec3 & goal_in)
  {
    RRT3DResult res;
    Vec3 start = start_in;
    Vec3 goal = goal_in;

    if (!SnapToFree(tree, start, cfg_.start_snap_radius)) {
      res.message = "start blocked and snap failed";
      return res;
    }
    if (!SnapToFree(tree, goal, cfg_.goal_snap_radius)) {
      res.message = "goal blocked and snap failed";
      return res;
    }

    if (SegmentFree(tree, start, goal)) {
      res.ok = true;
      res.message = "direct";
      res.path = {start, goal};
      return res;
    }

    Vec3 bmin;
    Vec3 bmax;
    if (!GetBounds(tree, bmin, bmax)) {
      res.message = "empty octomap bounds";
      return res;
    }

    struct Node
    {
      Vec3 p;
      int parent{-1};
    };

    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(cfg_.max_iterations) + 2);
    nodes.push_back(Node{start, -1});

    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> dx(bmin.x, bmax.x);
    std::uniform_real_distribution<double> dy(bmin.y, bmax.y);
    std::uniform_real_distribution<double> dz(bmin.z, bmax.z);

    int goal_idx = -1;
    for (int it = 0; it < cfg_.max_iterations; ++it) {
      Vec3 sample;
      if (unit(gen_) < cfg_.goal_bias) {
        sample = goal;
      } else {
        sample = Vec3{dx(gen_), dy(gen_), dz(gen_)};
      }

      int nearest = 0;
      double best = Dist3(nodes[0].p, sample);
      for (size_t i = 1; i < nodes.size(); ++i) {
        const double d = Dist3(nodes[i].p, sample);
        if (d < best) {
          best = d;
          nearest = static_cast<int>(i);
        }
      }

      const Vec3 & np = nodes[nearest].p;
      const double dist = Dist3(np, sample);
      if (dist < 1e-6) {
        continue;
      }
      const double scale = std::min(cfg_.step_size, dist) / dist;
      Vec3 steped{
        np.x + (sample.x - np.x) * scale,
        np.y + (sample.y - np.y) * scale,
        np.z + (sample.z - np.z) * scale};

      if (!SegmentFree(tree, np, steped)) {
        continue;
      }

      nodes.push_back(Node{steped, nearest});
      const int new_idx = static_cast<int>(nodes.size()) - 1;
      if (Dist3(steped, goal) <= cfg_.goal_tolerance) {
        if (SegmentFree(tree, steped, goal)) {
          nodes.push_back(Node{goal, new_idx});
          goal_idx = static_cast<int>(nodes.size()) - 1;
        } else {
          goal_idx = new_idx;
        }
        break;
      }
    }

    if (goal_idx < 0) {
      res.message = "rrt3d failed to reach goal";
      return res;
    }

    std::vector<Vec3> raw;
    for (int i = goal_idx; i >= 0; i = nodes[static_cast<size_t>(i)].parent) {
      raw.push_back(nodes[static_cast<size_t>(i)].p);
    }
    std::reverse(raw.begin(), raw.end());
    res.path = Shortcut(tree, raw);
    res.ok = res.path.size() >= 2;
    res.message = res.ok ? "ok" : "shortcut collapsed";
    return res;
  }

private:
  bool GetBounds(const octomap::OcTree & tree, Vec3 & bmin, Vec3 & bmax) const
  {
    if (cfg_.use_custom_bounds) {
      bmin = cfg_.bmin;
      bmax = cfg_.bmax;
      return true;
    }
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    // OcTree::getMetricMin/Max 在本机 octomap 返回 void
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    bmin = Vec3{min_x, min_y, min_z};
    bmax = Vec3{max_x, max_y, max_z};
    return bmax.x > bmin.x && bmax.y > bmin.y && bmax.z > bmin.z;
  }

  bool PointFree(const octomap::OcTree & tree, const Vec3 & p) const
  {
    // 机身球体采样碰撞：半径内有占据/未知(可选)则判堵
    const double r = std::max(0.05, cfg_.robot_radius);
    const double res = std::max(0.05, tree.getResolution());
    const int n = std::max(1, static_cast<int>(std::ceil(r / res)));
    for (int ix = -n; ix <= n; ++ix) {
      for (int iy = -n; iy <= n; ++iy) {
        for (int iz = -n; iz <= n; ++iz) {
          const double ox = p.x + ix * res;
          const double oy = p.y + iy * res;
          const double oz = p.z + iz * res;
          if (Dist3(p, Vec3{ox, oy, oz}) > r + 1e-6) {
            continue;
          }
          const octomap::OcTreeNode * node =
            tree.search(octomap::point3d(ox, oy, oz));
          if (node == nullptr) {
            if (cfg_.treat_unknown_as_occ) {
              return false;
            }
            continue;
          }
          if (tree.isNodeOccupied(node)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool SegmentFree(const octomap::OcTree & tree, const Vec3 & a, const Vec3 & b) const
  {
    const double dist = Dist3(a, b);
    const double step = std::max(0.05, tree.getResolution() * 0.5);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      Vec3 p{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t};
      if (!PointFree(tree, p)) {
        return false;
      }
    }
    return true;
  }

  bool SnapToFree(const octomap::OcTree & tree, Vec3 & p, double radius) const
  {
    if (PointFree(tree, p)) {
      return true;
    }
    const double res = std::max(0.05, tree.getResolution());
    const int n = std::max(1, static_cast<int>(std::ceil(radius / res)));
    double best = radius + 1.0;
    Vec3 best_p = p;
    bool found = false;
    for (int ix = -n; ix <= n; ++ix) {
      for (int iy = -n; iy <= n; ++iy) {
        for (int iz = -n; iz <= n; ++iz) {
          Vec3 c{p.x + ix * res, p.y + iy * res, p.z + iz * res};
          const double d = Dist3(p, c);
          if (d > radius || d >= best) {
            continue;
          }
          if (PointFree(tree, c)) {
            best = d;
            best_p = c;
            found = true;
          }
        }
      }
    }
    if (found) {
      p = best_p;
    }
    return found;
  }

  std::vector<Vec3> Shortcut(const octomap::OcTree & tree, const std::vector<Vec3> & raw) const
  {
    if (raw.size() <= 2) {
      return raw;
    }
    std::vector<Vec3> out;
    out.reserve(raw.size());
    size_t i = 0;
    out.push_back(raw.front());
    while (i + 1 < raw.size()) {
      size_t j = raw.size() - 1;
      while (j > i + 1) {
        if (SegmentFree(tree, raw[i], raw[j])) {
          break;
        }
        --j;
      }
      out.push_back(raw[j]);
      i = j;
    }
    return out;
  }

  RRT3DConfig cfg_;
  std::random_device rd_;
  std::mt19937 gen_;
};

}  // namespace nav3
