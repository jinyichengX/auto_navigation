#pragma once

#include <stack>
#include <string>
#include <vector>

#include "tree.h"

// 官方 kdtree C 库：include 根是 src/explorer/official
extern "C" {
#include "kdtree/kdtree.h"
}

// 与作者 nbvplanner/include/nbvplanner/rrt.h 保持一致
#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

/// 原作者 Andreas Bircher 的 NBVP 局部 RRT，变量命名与作者源码一致。
///   - exact_root_：上一轮最优分枝第一段的终点；
///   - bestBranchMemory_：上一轮尚未飞完的最优分枝；
///   - history_：退回上一位置时用；
///   - getBestEdge()：只返回最优分枝第一段。
/// 作者没有的 ROS2/360° 雷达适配，会在实现里单独标注。
class RrtTree final : public TreeBase<Eigen::Vector4d> {
public:
  using StateVec = Eigen::Vector4d;

  explicit RrtTree(nbvplanner_ros2::LocalOctomapManager *manager = nullptr);
  ~RrtTree() override;

  void setRoot(const Eigen::Vector3d &pos, double yaw);

  void initialize() override;
  void iterate(int iterations) override;
  void clear() override;

  std::vector<StateVec> getBestEdge();
  void memorizeBestBranch();
  std::vector<StateVec> getPathBackToPrevious();

  /// 可视化用：history_ 中的历史出发点，按时间顺序（最早 -> 最近）。
  std::vector<StateVec> getHistoryPath() const;

  /// 可视化用：rootNode_ 到 bestNode_ 的完整最优枝。
  std::vector<StateVec> getBestBranch() const;

  /// 可视化用：当前最优节点。
  bool getBestNodeState(StateVec &state) const;

  double gain(StateVec state);

private:
  std::vector<StateVec> samplePath(StateVec start, StateVec end);

  kdtree *kdTree_{nullptr};
  StateVec exact_root_{StateVec::Zero()};
  std::vector<StateVec> bestBranchMemory_;
  std::stack<StateVec> history_;
  int iterationCount_{0};
};

}  // namespace nbvInspection
