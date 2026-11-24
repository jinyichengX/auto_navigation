#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "octomap_manager.hpp"
#include "rrt.h"

namespace nbvplanner_ros2 {

using nbvInspection::Params;
using nbvInspection::RrtTree;

/// ROS2 外壳：负责地图/里程计输入、参数装载和“规划 → 飞第一段 → 再规划”的外层循环。
///
/// 内层决策完全交给 RrtTree（原作者的 NBVP 语义）：
///   - 局部 RRT + 未知体素增益；
///   - getBestEdge() 只返回最优分枝第一段；
///   - 无增益时 getPathBackToPrevious() 回退；
///   - 无历史可回退时保持当前位置并结束。
///
/// 外层循环等价于原仓库 interface_nbvp_rotors/src/exploration.cpp，
/// 只是把“ROS1 服务调用 + 逐点发轨迹”改成了节点内部状态机。
class NbvplannerExplorer : public rclcpp::Node {
public:
  NbvplannerExplorer() : Node("nbvplanner_explorer") {
    // ===== 作者参数（默认值与 exploration.yaml 对齐）=====
    params_.navigationFrame_ = declare_parameter<std::string>("tf_frame", "camera_init");
    params_.v_max_ = declare_parameter<double>("system/v_max", 1);
    params_.dyaw_max_ = declare_parameter<double>("system/dyaw_max", 0.75);
    // 机体碰撞盒。作者的 0.5x0.5x0.3 包不住当前 x500 + Mid-360：
    // 桨尖约 ±0.31 m，雷达顶部约 +0.34 m。这里用 0.7 立方体覆盖全机。
    params_.boundingBox_.x() = declare_parameter<double>("system/bbx/x", 0.7);
    params_.boundingBox_.y() = declare_parameter<double>("system/bbx/y", 0.7);
    params_.boundingBox_.z() = declare_parameter<double>("system/bbx/z", 0.7);
    params_.dOvershoot_ = declare_parameter<double>("system/bbx/overshoot", 0.4);
    params_.dt_ = declare_parameter<double>("nbvp/dt", 0.1);

    params_.igFree_ = declare_parameter<double>("nbvp/gain/free", 0.0);
    params_.igOccupied_ = declare_parameter<double>("nbvp/gain/occupied", 0.0);
    params_.igUnmapped_ = declare_parameter<double>("nbvp/gain/unmapped", 1.0);
    params_.igArea_ = declare_parameter<double>("nbvp/gain/area", 0.0);
    params_.igProbabilistic_ =
        declare_parameter<double>("nbvp/gain/probabilistic", 0.0);
    params_.gainRange_ = declare_parameter<double>("nbvp/gain/range", 1.5);
    params_.degressiveCoeff_ =
        declare_parameter<double>("nbvp/gain/degressive_coeff", 0.5);
    params_.zero_gain_ = declare_parameter<double>("nbvp/gain/zero", 0.0006);

    params_.extensionRange_ =
        declare_parameter<double>("nbvp/tree/extension_range", 6);
    params_.exact_root_ = declare_parameter<bool>("nbvp/tree/exact_root", false);
    params_.initIterations_ =
        declare_parameter<int>("nbvp/tree/initial_iterations", 15);
    params_.cuttoffIterations_ =
        declare_parameter<int>("nbvp/tree/cuttoff_iterations", 200);

    // 原作者的 bbx 是固定场景包围盒。这里改成围绕当前位姿的移动窗口，
    // 否则任意不规则地图无法使用；其余采样/边界判据保持作者语义。
    workspace_radius_ = declare_parameter<double>("workspace/radius", 40.0);
    workspace_z_radius_ = declare_parameter<double>("workspace/z_radius", 20.0);
    params_.softBounds_ = declare_parameter<bool>("bbx/softBounds", false);

    // ===== 视场：Mid-360 的竖直带，水平 360° 与 yaw 无关 =====
    const double camera_horizontal =
        declare_parameter<double>("system/camera/horizontal", 360.0);
    const double camera_vertical =
        declare_parameter<double>("system/camera/vertical", 59.0);
    const double low_deg =
        declare_parameter<double>("nbvp/gain/fov_min_deg", -7.0);
    const double high_deg =
        declare_parameter<double>("nbvp/gain/fov_max_deg", 52.0);
    params_.camHorizontal_ = {camera_horizontal};
    params_.camVertical_ = {camera_vertical};
    params_.fovLow_ = M_PI * low_deg / 180.0;
    params_.fovHigh_ = M_PI * high_deg / 180.0;
    params_.fovYawInvariant_ = camera_horizontal >= 359.0;
    {
      const Eigen::Vector3d bottom(std::cos(params_.fovLow_ + M_PI / 2.0), 0.0,
                                   std::sin(params_.fovLow_ + M_PI / 2.0));
      const Eigen::Vector3d top(std::cos(params_.fovHigh_ - M_PI / 2.0), 0.0,
                                std::sin(params_.fovHigh_ - M_PI / 2.0));
      std::vector<Eigen::Vector3d> normals{bottom, top};
      if (!params_.fovYawInvariant_) {
        const double side = M_PI * camera_horizontal / 360.0 - M_PI / 2.0;
        normals.emplace_back(std::cos(side), std::sin(side), 0.0);
        normals.emplace_back(std::cos(side), -std::sin(side), 0.0);
      }
      params_.camBoundNormals_.push_back(normals);
    }

    // ===== 外层执行参数（对应 exploration.cpp）=====
    const double init_x = declare_parameter<double>("init/wp_x", 0.0);
    const double init_y = declare_parameter<double>("init/wp_y", 0.0);
    const double init_z = declare_parameter<double>("init/wp_z", 1.0);
    init_offset_x_ = declare_parameter<double>("init/offset_x", -0.5);
    init_offset_y_ = declare_parameter<double>("init/offset_y", -0.5);
    init_tolerance_ = declare_parameter<double>("init/tolerance", 0.30);
    const double init_stable_sec =
        declare_parameter<double>("init/stable_sec", 1.0);
    exec_z_offset_ = declare_parameter<double>("execution/z_offset", 0.0);
    plan_tick_budget_ms_ =
        declare_parameter<int>("planner/tick_budget_ms", 60);
    map_parse_period_ = declare_parameter<double>("planner/map_parse_period", 1.0);
    odom_timeout_ = declare_parameter<double>("planner/odom_timeout", 2.0);

    init_climb_target_ = Eigen::Vector3d(init_x, init_y, init_z);
    init_offset_target_ = Eigen::Vector3d(
        init_x + init_offset_x_, init_y + init_offset_y_, init_z);
    init_stable_ticks_ = std::max(
        1, static_cast<int>(std::ceil(init_stable_sec / std::max(1e-3, params_.dt_))));

    tree_ = std::make_unique<RrtTree>(&manager_);
    tree_->setParams(params_);

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::QoS(10));
    workspace_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/nbvp/workspace", rclcpp::QoS(10));
    best_branch_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/nbvp/best_branch", rclcpp::QoS(10));
    history_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/nbvp/history", rclcpp::QoS(10));
    candidate_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/nbvp/candidate", rclcpp::QoS(10));
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", rclcpp::QoS(10),
        std::bind(&NbvplannerExplorer::mapCallback, this,
                  std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", rclcpp::QoS(10),
        std::bind(&NbvplannerExplorer::odomCallback, this,
                  std::placeholders::_1));

    const auto period = std::chrono::milliseconds(
        std::max(20, static_cast<int>(std::lround(1000.0 * params_.dt_))));
    timer_ = create_wall_timer(period,
                               std::bind(&NbvplannerExplorer::tick, this));

    RCLCPP_INFO(
        get_logger(),
        "NBVP explorer ready. frame=%s, bbx=%.2fx%.2fx%.2f m, "
        "workspace=%.1f/%.1f m, extRange=%.2f, gainRange=%.2f, "
        "v_max=%.2f, dt=%.2f, FOV=[%.0f°,%.0f°]",
        params_.navigationFrame_.c_str(), params_.boundingBox_.x(), params_.boundingBox_.y(),
        params_.boundingBox_.z(), workspace_radius_, workspace_z_radius_,
        params_.extensionRange_, params_.gainRange_, params_.v_max_, params_.dt_, low_deg,
        high_deg);
  }

private:
  enum class Phase { kInitClimb, kInitOffset, kIdle, kBuild, kExec, kDone };

  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    const double t = now().seconds();
    if (have_map_ && t - last_map_parse_ < map_parse_period_) return;
    last_map_parse_ = t;

    octomap::AbstractOcTree *raw = octomap_msgs::msgToMap(*msg);
    if (!raw) return;
    auto *tree_ptr = dynamic_cast<octomap::OcTree *>(raw);
    if (!tree_ptr) {
      delete raw;
      return;
    }
    std::shared_ptr<octomap::OcTree> next(tree_ptr);
    // 先告诉 manager 新树，再释放旧树。
    manager_.setTree(next.get());
    octree_ = std::move(next);
    have_map_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    pos_ = Eigen::Vector3d(p.x, p.y, p.z);
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_odom_received_ = now().seconds();
    ready_ = true;
  }

  void tick() {
    if (!ready_ || !pos_.allFinite() ||
        now().seconds() - last_odom_received_ > odom_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s",
                           "等待 /Odometry");
      return;
    }
    // 作者每次规划都用机器人当前位姿更新 root_；回退路径也从这里出发。
    tree_->setRoot(pos_, yaw_);
    publishHistory();

    switch (phase_) {
      case Phase::kInitClimb:
        publishTarget(init_climb_target_);
        if ((pos_ - init_climb_target_).norm() < init_tolerance_) {
          if (++stable_ticks_ >= init_stable_ticks_) {
            phase_ = Phase::kInitOffset;
            stable_ticks_ = 0;
            RCLCPP_INFO(get_logger(), "初始化爬升完成，执行初始位移");
          }
        } else {
          stable_ticks_ = 0;
        }
        break;

      case Phase::kInitOffset:
        publishTarget(init_offset_target_);
        if ((pos_ - init_offset_target_).norm() < init_tolerance_) {
          RCLCPP_INFO(get_logger(), "初始化运动完成，开始 NBVP 规划");
          phase_ = Phase::kIdle;
        }
        break;

      case Phase::kIdle:
        if (!have_map_) {
          publishTarget(pos_);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s",
                               "等待 /octomap_binary");
          return;
        }
        beginPlan();
        break;

      case Phase::kBuild:
        // 规划期间保持当前位置，等价于作者在服务调用期间让控制器保持上一点。
        publishTarget(pos_);
        continuePlan();
        break;

      case Phase::kExec:
        executeEdge();
        break;

      case Phase::kDone:
        publishTarget(pos_);
        break;
    }
  }

  void beginPlan() {
    // 作者用固定 bbx；这里用围绕当前位姿的移动窗口，算法其余部分不变。
    params_.minX_ = pos_.x() - workspace_radius_;
    params_.maxX_ = pos_.x() + workspace_radius_;
    params_.minY_ = pos_.y() - workspace_radius_;
    params_.maxY_ = pos_.y() + workspace_radius_;
    params_.minZ_ = pos_.z() - workspace_z_radius_;
    params_.maxZ_ = pos_.z() + workspace_z_radius_;

    tree_->setParams(params_);
    tree_->setRoot(pos_, yaw_);
    // 与作者 plannerCallback 一致：先 clear，再 initialize。
    tree_->clear();
    tree_->initialize();
    publishWorkspaceMarker();
    clearPlanVisualization();
    plan_start_wall_ = std::chrono::steady_clock::now();
    loop_count_ = 0;
    backtrack_ = false;
    phase_ = Phase::kBuild;
  }

  void continuePlan() {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(plan_tick_budget_ms_);
    while (true) {
      // 工程保护：单轮规划不能无限拖住，超时就带着当前最优结果结束。
      const double elapsed =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - plan_start_wall_)
              .count();
      if (elapsed > kPlanWallTimeoutSec) {
        RCLCPP_WARN(get_logger(), "本轮规划超过 %.1f s，强制结束",
                    kPlanWallTimeoutSec);
        break;
      }
      // 作者的终止条件：最少迭代数 + 已找到增益，或超过 cutoff。
      if (tree_->getCounter() >= params_.initIterations_ && tree_->gainFound()) {
        break;
      }
      if (tree_->getCounter() > params_.cuttoffIterations_) break;
      if (loop_count_ > 1500 * (tree_->getCounter() + 1)) {
        backtrack_ = true;
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "规划中: counter=%d bestGain=%.4f",
                             tree_->getCounter(), tree_->bestGain());
        return;  // 留在 kBuild，下个 tick 继续
      }
      tree_->iterate(1);
      ++loop_count_;
    }
    finishPlan();
  }

  void finishPlan() {
    // 采样严重失败：作者会退回上一位置；若已无历史且确实有增益，继续正常出边。
    if (backtrack_) {
      auto back = tree_->getPathBackToPrevious();
      if (!back.empty()) {
        RCLCPP_INFO(get_logger(), "采样失败过多，退回上一位置（%zu 点）",
                    back.size());
        startExec(std::move(back));
        return;
      }
      backtrack_ = false;
    }

    if (!tree_->gainFound()) {
      auto back = tree_->getPathBackToPrevious();
      if (!back.empty()) {
        RCLCPP_INFO(get_logger(), "无可用增益，退回上一位置（%zu 点）",
                    back.size());
        startExec(std::move(back));
        return;
      }
      done("无增益且没有可回退的历史");
      return;
    }

    auto edge = tree_->getBestEdge();
    tree_->memorizeBestBranch();
    if (edge.empty()) {//没有目的地
      auto back = tree_->getPathBackToPrevious();
      if (!back.empty()) {
        RCLCPP_INFO(get_logger(), "最优边为空，退回上一位置（%zu 点）",
                    back.size());
        startExec(std::move(back));
        return;
      }
      done("最优边为空且没有历史");
      return;
    }

    RCLCPP_INFO(get_logger(),
                "第 %d 次规划: counter=%d bestGain=%.4f 第一段 %.2f m (%zu 点)",
                ++plan_count_, tree_->getCounter(), tree_->bestGain(),
                (edge.back().head<3>() - pos_).norm(), edge.size());
    publishBestBranch();
    startExec(std::move(edge));
  }

  void startExec(std::vector<RrtTree::StateVec> path) {
    edge_ = std::move(path);
    edge_index_ = 0;
    phase_ = Phase::kExec;
  }

  void executeEdge() {
    if (edge_index_ >= edge_.size()) {
      edge_.clear();
      edge_index_ = 0;
      phase_ = Phase::kIdle;
      return;
    }
    Eigen::Vector3d goal = edge_[edge_index_].head<3>();
    goal.z() += exec_z_offset_;
    publishTarget(goal);
    ++edge_index_;
  }

  void done(const char *why) {
    phase_ = Phase::kDone;
    RCLCPP_WARN(get_logger(), "探索结束：%s（当前位置 %.2f,%.2f,%.2f）", why,
                pos_.x(), pos_.y(), pos_.z());
  }

  void publishWorkspaceMarker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.navigationFrame_;
    marker.header.stamp = now();
    marker.ns = "nbvp_workspace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
    marker.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
    marker.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.maxX_ - params_.minX_;
    marker.scale.y = params_.maxY_ - params_.minY_;
    marker.scale.z = params_.maxZ_ - params_.minZ_;
    marker.color.r = 0.43f;
    marker.color.g = 0.91f;
    marker.color.b = 1.0f;
    marker.color.a = 0.08f;
    workspace_marker_pub_->publish(marker);
  }

  void clearPlanVisualization() {
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.navigationFrame_;
    path.header.stamp = now();
    best_branch_pub_->publish(path);

    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "nbvp_candidate";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    candidate_pub_->publish(marker);
  }

  /// 可视化：history_（每次规划出发时的位置，按时间顺序）+ 当前位置。
  void publishHistory() {
    const auto hist = tree_->getHistoryPath();
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.navigationFrame_;
    path.header.stamp = now();
    path.poses.reserve(hist.size() + 1);
    for (const auto &state : hist) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = state[0];
      pose.pose.position.y = state[1];
      pose.pose.position.z = state[2];
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    geometry_msgs::msg::PoseStamped cur;
    cur.header = path.header;
    cur.pose.position.x = pos_.x();
    cur.pose.position.y = pos_.y();
    cur.pose.position.z = pos_.z();
    cur.pose.orientation.w = 1.0;
    path.poses.push_back(cur);
    history_pub_->publish(path);
  }

  void publishBestBranch() {
    const auto branch = tree_->getBestBranch();
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.navigationFrame_;
    path.header.stamp = now();
    path.poses.reserve(branch.size());
    for (const auto &state : branch) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = state[0];
      pose.pose.position.y = state[1];
      pose.pose.position.z = state[2];
      pose.pose.orientation.z = std::sin(0.5 * state[3]);
      pose.pose.orientation.w = std::cos(0.5 * state[3]);
      path.poses.push_back(pose);
    }
    best_branch_pub_->publish(path);

    RrtTree::StateVec best;
    if (!tree_->getBestNodeState(best)) return;
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "nbvp_candidate";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = best[0];
    marker.pose.position.y = best[1];
    marker.pose.position.z = best[2];
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 0.94f;
    marker.color.g = 0.71f;
    marker.color.b = 0.35f;
    marker.color.a = 0.9f;
    candidate_pub_->publish(marker);
  }

  void publishTarget(const Eigen::Vector3d &p) {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = params_.navigationFrame_;
    target.header.stamp = now();
    target.pose.position.x = p.x();
    target.pose.position.y = p.y();
    target.pose.position.z = p.z();
    // Mid-360 水平 360°，航向不影响观测；保持当前航向，避免 LIO/PX4
    // 航向偏置被当成转向指令。
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = std::sin(0.5 * yaw_);
    target.pose.orientation.w = std::cos(0.5 * yaw_);
    target_pub_->publish(target);
  }

  // ===== 参数 =====
  Params params_;
  double workspace_radius_{10.0};
  double workspace_z_radius_{5.0};
  double init_offset_x_{-0.5};
  double init_offset_y_{-0.5};
  double init_tolerance_{0.30};
  double exec_z_offset_{0.0};
  int plan_tick_budget_ms_{60};
  double map_parse_period_{1.0};
  double odom_timeout_{2.0};

  // ===== 地图 / 位姿 =====
  LocalOctomapManager manager_{nullptr};
  std::unique_ptr<RrtTree> tree_;
  std::shared_ptr<octomap::OcTree> octree_;
  bool have_map_{false};
  double last_map_parse_{-1.0e9};

  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  double yaw_{0.0};
  bool ready_{false};
  double last_odom_received_{-1.0e9};

  // ===== 外层状态机 =====
  Phase phase_{Phase::kInitClimb};
  Eigen::Vector3d init_climb_target_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d init_offset_target_{Eigen::Vector3d::Zero()};
  int init_stable_ticks_{1};
  int stable_ticks_{0};
  std::vector<RrtTree::StateVec> edge_;
  size_t edge_index_{0};
  int loop_count_{0};
  int plan_count_{0};
  bool backtrack_{false};
  std::chrono::steady_clock::time_point plan_start_wall_;
  static constexpr double kPlanWallTimeoutSec = 10.0;

  // ===== ROS 接口 =====
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr workspace_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr best_branch_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr candidate_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nbvplanner_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nbvplanner_ros2::NbvplannerExplorer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
