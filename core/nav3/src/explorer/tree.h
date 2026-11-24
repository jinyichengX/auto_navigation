#pragma once

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "octomap_manager.hpp"

namespace nbvInspection {

/// 与作者 nbvplanner/nbv.h 的 Params 一一对应（去掉 ROS/mesh 相关项）。
struct Params {
  // ===== 视场 =====
  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d>> camBoundNormals_;

  // 水平 360° 的雷达：视场只由仰角带决定，与节点 yaw 无关。
  // 作者按 yaw 旋转视场法线是给单目相机用的；套到 360° 雷达上会把增益场
  // 变成随机楔形，bestNode 的选择失去意义。
  bool fovYawInvariant_{true};
  double fovLow_{-0.12217304763960307};   // -7°
  double fovHigh_{0.9075712110370514};    // +52°

  // ===== 增益 =====
  double igProbabilistic_{0.0};
  double igFree_{0.0};
  double igOccupied_{0.0};
  double igUnmapped_{1.0};
  double igArea_{0.0};
  double gainRange_{1.0};
  double degressiveCoeff_{0.25};
  double zero_gain_{0.0006};

  // ===== 系统 =====
  double v_max_{0.4};
  double dyaw_max_{1.0};
  double dOvershoot_{0.5};
  double extensionRange_{1.5};
  bool exact_root_{false};
  int initIterations_{30};
  int cuttoffIterations_{200};
  double dt_{0.1};

  // ===== 局部工作空间（每次规划前围绕飞机重设，来自 map 的局部范围）=====
  double minX_{-8.0}, minY_{-8.0}, minZ_{-4.0};
  double maxX_{8.0}, maxY_{8.0}, maxZ_{4.0};
  // 增益评估的竖直范围（通常=当前走廊全高），与"飞行工作空间"分开：
  // 飞机只在巡航高度附近飞，但打分必须看得见整条巷道截面，
  // 否则局部 NBV 会因为"只看得见薄薄一层"而长期找不到增益。
  double gainMinZ_{-1.0e9};
  double gainMaxZ_{1.0e9};
  bool softBounds_{true};
  // 机体碰撞盒，覆盖 x500 机身 + 桨叶 + Mid-360（雷达顶部约 +0.34 m）。
  Eigen::Vector3d boundingBox_{0.7, 0.7, 0.7};

  std::string navigationFrame_;
};

template <typename stateVec>
struct Node {
  Node() : parent_(nullptr), gain_(0.0), distance_(0.0) {}
  ~Node() {
    for (auto *child : children_) delete child;
  }
  stateVec state_;
  Node *parent_;
  std::vector<Node *> children_;
  double gain_;
  double distance_;
};

template <typename stateVec>
class TreeBase {
public:
  TreeBase() = default;
  explicit TreeBase(nbvplanner_ros2::LocalOctomapManager *manager)
      : manager_(manager) {}
  // 树的释放由具体实现负责（RrtTree::~RrtTree），这里不能再删一次。
  virtual ~TreeBase() = default;

  void setManager(nbvplanner_ros2::LocalOctomapManager *manager) { manager_ = manager; }
  void setParams(const Params &p) { params_ = p; }
  const Params &params() const { return params_; }
  int getCounter() const { return counter_; }
  bool gainFound() const { return bestGain_ > (params_.zero_gain_ + 0.0002); }
  double bestGain() const { return bestGain_; }

  virtual void initialize() = 0;
  virtual void iterate(int iterations) = 0;
  virtual void clear() = 0;

protected:
  Params params_;
  int counter_{0};
  double bestGain_{0.0};
  Node<stateVec> *bestNode_{nullptr};
  Node<stateVec> *rootNode_{nullptr};
  nbvplanner_ros2::LocalOctomapManager *manager_{nullptr};
  stateVec root_{stateVec::Zero()};
};

}  // namespace nbvInspection
