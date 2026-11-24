#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace nbvplanner_ros2 {

/// 本地 OctoMap 后端，提供 NBVPlanner 算法需要的几何查询。
/// 官方依赖 ROS1 的 volumetric_mapping 包，这里用 octomap::OcTree 直接实现。
///
/// 语义约定（与作者一致）：
///   - 未知格当作**可通行**（探索就是要往未知走）；
///   - 只有占据格会阻挡；
///   - 碰撞检查用机体包围盒扫过整条线段。
class LocalOctomapManager {
public:
  enum class CellStatus { kFree, kOccupied, kUnknown };

  explicit LocalOctomapManager(octomap::OcTree *tree = nullptr) : tree_(tree) {}

  void setTree(octomap::OcTree *tree) { tree_ = tree; }

  double getResolution() const { return tree_ ? tree_->getResolution() : 0.1; }

  bool hasTree() const { return tree_ != nullptr; }

  CellStatus getCellProbabilityPoint(const Eigen::Vector3d &point,
                                     double *probability = nullptr) const {
    if (!tree_) return CellStatus::kUnknown;
    const auto *node = tree_->search(point.x(), point.y(), point.z());
    if (!node) return CellStatus::kUnknown;
    if (probability) *probability = node->getOccupancy();
    return tree_->isNodeOccupied(node) ? CellStatus::kOccupied : CellStatus::kFree;
  }

  /// origin 到 target 的射线是否被占据格挡住。
  /// 未知格不算遮挡：作者想评估"从这里能不能看见那片未知"。
  CellStatus getVisibility(const Eigen::Vector3d &origin,
                           const Eigen::Vector3d &target) const {
    if (!tree_) return CellStatus::kUnknown;
    octomap::KeyRay ray;
    if (!tree_->computeRayKeys(
            octomap::point3d(origin.x(), origin.y(), origin.z()),
            octomap::point3d(target.x(), target.y(), target.z()), ray)) {
      return CellStatus::kFree;
    }
    for (const auto &key : ray) {
      const auto *node = tree_->search(key);
      if (node && tree_->isNodeOccupied(node)) return CellStatus::kOccupied;
    }
    return CellStatus::kFree;
  }

  /// 机体中心放在 center 时，安全盒体积内有没有**已知占据**格。
  /// 直接遍历包围盒内的叶子节点：既不会像"只查角点"那样漏掉薄墙，
  /// 也不会像"按分辨率遍历整盒"那样慢。
  bool boxOccupied(const Eigen::Vector3d &center,
                   const Eigen::Vector3d &box) const {
    if (!tree_) return false;
    const Eigen::Vector3d half = 0.5 * box;
    const double res = tree_->getResolution();
    // bbx 迭代器按 key 取整，会把包围盒外最多一格的叶子也吐出来。
    // 直接拿它判占据等于凭空放大安全盒 ~0.1 m：贴墙走在 0.5 m 处会被判
    // "机身盒压障碍"，然后反复脱困、原地打转（仿真里 400 s 的根因）。
    // 这里再逐叶子做一次精确 AABB 相交。
    const octomap::point3d mn(center.x() - half.x() - 0.5 * res,
                              center.y() - half.y() - 0.5 * res,
                              center.z() - half.z() - 0.5 * res);
    const octomap::point3d mx(center.x() + half.x() + 0.5 * res,
                              center.y() + half.y() + 0.5 * res,
                              center.z() + half.z() + 0.5 * res);
    if (mn.x() > mx.x() || mn.y() > mx.y() || mn.z() > mx.z()) return false;
    for (auto it = tree_->begin_leafs_bbx(mn, mx), end = tree_->end_leafs_bbx();
         it != end; ++it) {
      if (!tree_->isNodeOccupied(&*it)) continue;
      const double s = 0.5 * it.getSize();
      if (std::abs(it.getX() - center.x()) <= s + half.x() &&
          std::abs(it.getY() - center.y()) <= s + half.y() &&
          std::abs(it.getZ() - center.z()) <= s + half.z()) {
        return true;
      }
    }
    return false;
  }

  /// 机体安全盒从 start 扫到 end（逐截面检查）是否全程无占据。
  /// 与作者的 getLineStatusBoundingBox 等价。
  CellStatus getLineStatusBoundingBox(const Eigen::Vector3d &start,
                                      const Eigen::Vector3d &end,
                                      const Eigen::Vector3d &boundingBox) const {
    if (!tree_) return CellStatus::kUnknown;
    const Eigen::Vector3d seg = end - start;
    const double len = seg.norm();
    const double res = std::max(1e-3, tree_->getResolution());
    // 步长取 0.75*res：不会跨过一个小格子，同时比逐格采样省 25%。
    const int samples = std::max(
        1, std::min(400, static_cast<int>(std::ceil(len / (0.75 * res)))));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      if (boxOccupied(start + t * seg, boundingBox)) return CellStatus::kOccupied;
    }
    return CellStatus::kFree;
  }

  /// 从"已经贴住障碍"的状态往外挪：不检查起点自身那一小段。
  /// 机体一旦进入自己的安全盒，普通版本在起点截面就必然判占据，
  /// 任何方向都过不去 —— 这正是"卡住不动"的机制。
  CellStatus getLineStatusFromBody(const Eigen::Vector3d &start,
                                   const Eigen::Vector3d &end,
                                   const Eigen::Vector3d &boundingBox) const {
    if (!tree_) return CellStatus::kUnknown;
    if (boxOccupied(end, boundingBox)) return CellStatus::kOccupied;
    const Eigen::Vector3d d = end - start;
    const double len = d.norm();
    const double self = 0.5 * std::max(boundingBox.x(), boundingBox.y());
    if (len <= self) return CellStatus::kFree;
    return getLineStatusBoundingBox(start + (self / len) * d, end, boundingBox);
  }

  /// 安全版边检查：机体盒沿整条线段扫过时，任何截面只要碰到 unknown 或
  /// occupied 就拒绝。作者的 getLineStatusBoundingBox 只挡 occupied，这里
  /// 额外把 unknown 也当成不可穿越；RRT 的边检查用它，增益计算仍用原版。
  CellStatus getLineStatusBoundingBoxKnownFree(
      const Eigen::Vector3d &start, const Eigen::Vector3d &end,
      const Eigen::Vector3d &boundingBox) const {
    if (!tree_) return CellStatus::kUnknown;
    const Eigen::Vector3d seg = end - start;
    const double len = seg.norm();
    if (len < 1e-9) return CellStatus::kFree;
    const double res = std::max(1e-3, tree_->getResolution());
    // 跳过机体自身占据的起始段，避免飞机已经在 unknown 里时所有边都被判死。
    const double self = 0.5 * std::max(boundingBox.x(), boundingBox.y());
    if (len <= self) return CellStatus::kFree;
    const Eigen::Vector3d begin = start + (self / len) * seg;
    const Eigen::Vector3d rest = end - begin;
    const double rest_len = rest.norm();
    const int samples = std::max(
        1, std::min(800, static_cast<int>(std::ceil(rest_len / (0.5 * res)))));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      if (!isKnownFreeBox(begin + t * rest, boundingBox)) {
        return CellStatus::kUnknown;
      }
    }
    return CellStatus::kFree;
  }

  /// 安全盒体积内是否**每个采样点都是已知自由**（未知不算）。
  /// 只用于给"已知区域优先"打分，不参与碰撞判定。
  bool isKnownFreeBox(const Eigen::Vector3d &center,
                      const Eigen::Vector3d &box) const {
    if (!tree_) return false;
    const double res = std::max(1e-3, tree_->getResolution());
    const Eigen::Vector3d half = 0.5 * box;
    auto axis = [res](double h) {
      std::vector<double> v;
      const int n = std::max(1, static_cast<int>(std::ceil(2.0 * h / res)));
      for (int i = 0; i <= n; ++i) {
        v.push_back(-h + 2.0 * h * static_cast<double>(i) / n);
      }
      return v;
    };
    for (double dx : axis(half.x()))
      for (double dy : axis(half.y()))
        for (double dz : axis(half.z())) {
          if (getCellProbabilityPoint(
                  center + Eigen::Vector3d(dx, dy, dz)) != CellStatus::kFree) {
            return false;
          }
        }
    return true;
  }

private:
  octomap::OcTree *tree_;
};

}  // namespace nbvplanner_ros2
