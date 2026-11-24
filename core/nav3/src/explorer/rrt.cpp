#include "rrt.h"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <limits>

namespace nbvInspection {

RrtTree::RrtTree(nbvplanner_ros2::LocalOctomapManager *manager)
    : TreeBase<StateVec>(manager) {}

RrtTree::~RrtTree() {
  delete rootNode_;
  rootNode_ = nullptr;
  if (kdTree_) {
    kd_free(kdTree_);
    kdTree_ = nullptr;
  }
}

void RrtTree::setRoot(const Eigen::Vector3d &pos, double yaw) {
  root_[0] = pos.x();
  root_[1] = pos.y();
  root_[2] = pos.z();
  root_[3] = yaw;
}

void RrtTree::clear() {
  delete rootNode_;
  rootNode_ = nullptr;
  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = nullptr;
  if (kdTree_) {
    kd_free(kdTree_);
    kdTree_ = nullptr;
  }
}

void RrtTree::initialize() {
  kdTree_ = kd_create(3);

  // 与作者一致：树根直接放在 rootNode_，优先使用上一轮第一段终点。
  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = nullptr;
  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }
  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(),
             rootNode_->state_.z(), rootNode_);
  iterationCount_++;

  // 把上一轮没飞完的最优分枝重新接回树。
  for (typename std::vector<StateVec>::reverse_iterator iter =
           bestBranchMemory_.rbegin();
       iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres * nearest =
        kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    Node<StateVec> * newParent =
        (Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);

    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1],
                           newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0],
                              newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    if (nbvplanner_ros2::LocalOctomapManager::CellStatus::kFree ==
        manager_->getLineStatusBoundingBoxKnownFree(
            origin,
            direction + origin + direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)) {
      Node<StateVec> * newNode = new Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ =
          newParent->gain_ +
          gain(newNode->state_) *
              exp(-params_.degressiveCoeff_ * newNode->distance_);

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }
}

void RrtTree::iterate(int iterations) {
  (void)iterations;
  StateVec newState;

//在一个以探索空间最大对角线为半径的球内采样
//为什要在球面上采样：如果采样点是在一个矩形/立方体区域里均匀撒的，
//而树本身倾向于往某些方向长，树就容易往区域的中心扎堆，探索不均匀
  double radius = sqrt(SQ(params_.minX_ - params_.maxX_) +
                       SQ(params_.minY_ - params_.maxY_) +
                       SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound = false;
  // 工程保护：硬边界 + 异常根节点时，作者的 while 可能永远采不到合法点。
  const int kMaxSamplingAttempts = 2000;
  int attempts = 0;
  while (!solutionFound) {
    if (++attempts > kMaxSamplingAttempts) {
      return;
    }
    for (int i = 0; i < 3; i++) {
      newState[i] =
          2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0)) {
      continue;//若点在球体外，丢弃并重新采样
    }
    newState += rootNode_->state_;
    if (!params_.softBounds_) {//这段必执行，因为此项目softBounds_=0
      if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() <
                 params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() <
                 params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() >
                 params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() >
                 params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() >
                 params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    solutionFound = true;
  }

  kdres * nearest =
      kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  Node<StateVec> * newParent = (Node<StateVec> *) kd_res_item_data(nearest);
  kd_res_free(nearest);

  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1],
                         newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0],
                            newState[1] - origin[1],
                            newState[2] - origin[2]);//newParent到newState的向量
  if (direction.norm() > params_.extensionRange_) {//方向不变，限制到RRT采样步进长度
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];

  if (nbvplanner_ros2::LocalOctomapManager::CellStatus::kFree ==
      manager_->getLineStatusBoundingBoxKnownFree(
          origin,
          direction + origin + direction.normalized() * params_.dOvershoot_,//给控制过冲/刹车留余量
          params_.boundingBox_)) {
    newState[3] =
        2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);

    Node<StateVec> * newNode = new Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    newNode->gain_ =
        newParent->gain_ +
        gain(newNode->state_) *
            exp(-params_.degressiveCoeff_ * newNode->distance_);

    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);
    // 作者的 publishNode(newNode) 与多机避让未移植。

    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }
}

double RrtTree::gain(StateVec state) {
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
       vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_);
       vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
         vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_);
         vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
           vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_);
           vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        if (params_.fovYawInvariant_) {
          // 雷达适配：360° 水平视场，作者原代码没有这一段。
          const double margin =
              std::asin(std::min(1.0, SQRT2 * disc / dir.norm()));
          const double elevation =
              std::asin(std::clamp(dir.z() / dir.norm(), -1.0, 1.0));
          insideAFieldOfView = (elevation > params_.fovLow_ + margin &&
                                elevation < params_.fovHigh_ - margin);
        } else {
          for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator
                   itCBN = params_.camBoundNormals_.begin();
               itCBN != params_.camBoundNormals_.end(); itCBN++) {
            bool inThisFieldOfView = true;
            for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                     itCBN->begin();
                 itSingleCBN != itCBN->end(); itSingleCBN++) {
              Eigen::Vector3d normal =
                  Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                  (*itSingleCBN);
              double val = dir.dot(normal.normalized());
              if (val < SQRT2 * disc) {
                inThisFieldOfView = false;
                break;
              }
            }
            if (inThisFieldOfView) {
              insideAFieldOfView = true;
              break;
            }
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }

        double probability;
        nbvplanner_ros2::LocalOctomapManager::CellStatus node =
            manager_->getCellProbabilityPoint(vec, &probability);
        if (node ==
            nbvplanner_ros2::LocalOctomapManager::CellStatus::kUnknown) {
          if (nbvplanner_ros2::LocalOctomapManager::CellStatus::kOccupied !=
              manager_->getVisibility(origin, vec)) {
            gain += params_.igUnmapped_;
          }
        } else if (node ==
                   nbvplanner_ros2::LocalOctomapManager::CellStatus::kOccupied) {
          if (nbvplanner_ros2::LocalOctomapManager::CellStatus::kOccupied !=
              manager_->getVisibility(origin, vec)) {
            gain += params_.igOccupied_;
          }
        } else {
          if (nbvplanner_ros2::LocalOctomapManager::CellStatus::kOccupied !=
              manager_->getVisibility(origin, vec)) {
            gain += params_.igFree_;
          }
        }
      }
    }
  }
  gain *= pow(disc, 3.0);
  return gain;
}

std::vector<RrtTree::StateVec> RrtTree::getBestEdge() {
  std::vector<StateVec> ret;
  Node<StateVec> * current = bestNode_;
  if (current->parent_ != nullptr) {
    while (current->parent_ != rootNode_ && current->parent_ != nullptr) {
      current = current->parent_;//current最后等于rootNode_的子节点
    }
    ret = samplePath(current->parent_->state_, current->state_);//从rootNode_到current的路径
    history_.push(current->parent_->state_);//这一段飞出去之后，万一要退回，以及下一轮树根该钉在哪
    exact_root_ = current->state_;//记下 A 的位姿 （这一段终点），下一轮`initialize()` 会用它当新树根
  }
  return ret;
}

std::vector<RrtTree::StateVec> RrtTree::getBestBranch() const {
  std::vector<StateVec> path;
  if (!bestNode_ || !rootNode_) return path;
  const Node<StateVec> *current = bestNode_;
  while (current && current != rootNode_) {
    path.push_back(current->state_);
    current = current->parent_;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool RrtTree::getBestNodeState(StateVec &state) const {
  if (!bestNode_) return false;
  state = bestNode_->state_;
  return true;
}

void RrtTree::memorizeBestBranch() {
  bestBranchMemory_.clear();
  // Node<StateVec> * current = bestNode_;
  // while (current->parent_ && current->parent_->parent_) {
  //   bestBranchMemory_.push_back(current->state_);
  //   current = current->parent_;
  // }
}

std::vector<RrtTree::StateVec> RrtTree::getPathBackToPrevious() {
  std::vector<StateVec> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top());
  history_.pop();
  return ret;
}

std::vector<RrtTree::StateVec> RrtTree::getHistoryPath() const {
  std::vector<StateVec> ret;
  std::stack<StateVec> tmp = history_;
  ret.reserve(tmp.size());
  while (!tmp.empty()) {
    ret.push_back(tmp.top());
    tmp.pop();
  }
  // 栈顶是最近的出发点，翻转成按时间顺序，便于 visualize。
  std::reverse(ret.begin(), ret.end());
  return ret;
}

std::vector<RrtTree::StateVec> RrtTree::samplePath(StateVec start,
                                                   StateVec end) {
  std::vector<StateVec> ret;
  // 作者用 TF 转换到 targetFrame；这里地图与位姿同帧，省略 TF。
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1],
                           end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                         params_.dt_ * params_.dyaw_max_ /
                             std::abs(yaw_direction));
  // 作者这里是 assert(disc > 0.0)；距离为零时会产生 NaN，这里兜底一次。
  if (!(disc > 0.0) || !std::isfinite(disc)) {
    disc = 1.0;
  }
  for (double it = 0.0; it <= 1.0; it += disc) {
    Eigen::Vector3d origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    if (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }
    StateVec state;
    state[0] = origin[0];
    state[1] = origin[1];
    state[2] = origin[2];
    state[3] = yaw;
    ret.push_back(state);
  }
  return ret;
}

}  // namespace nbvInspection
