// esdf_server.cpp
//
// 定稿方案 A：只订阅 /octomap_binary，增量维护 ESDF，发布 /esdf_pointcloud。
// intensity = 到最近障碍的距离（米）。
//
// 增量策略（FIESTA-inspired）：
// 1) 首帧全量 BFS
// 2) 后续对 occupied 做帧间 diff
// 3) insert: 新障碍置 0 并向外传播更短距离
// 4) delete: 找出依赖被删障碍的受影响区域，raise 后从仍存活障碍 reseed

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace esdf_node
{

struct Voxel
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const Voxel & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  size_t operator()(const Voxel & v) const
  {
    const size_t h1 = std::hash<int32_t>()(v.x);
    const size_t h2 = std::hash<int32_t>()(v.y);
    const size_t h3 = std::hash<int32_t>()(v.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

static const int DX[6] = {1, -1, 0, 0, 0, 0};
static const int DY[6] = {0, 0, 1, -1, 0, 0};
static const int DZ[6] = {0, 0, 0, 0, 1, -1};

class ESDFServer : public rclcpp::Node
{
public:
  ESDFServer()
  : Node("esdf_server")
  {
    declare_parameter<std::string>("input_topic", "/octomap_binary");
    declare_parameter<std::string>("output_topic", "/esdf_pointcloud");
    declare_parameter<std::string>("frame_id", "camera_init");
    declare_parameter<double>("max_distance", 2.0);
    declare_parameter<bool>("publish_obstacle_only", false);
    declare_parameter<double>("origin_x", 0.0);
    declare_parameter<double>("origin_y", 0.0);
    declare_parameter<double>("origin_z", 0.0);

    const std::string in_topic = get_parameter("input_topic").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    max_distance_ = get_parameter("max_distance").as_double();
    publish_obstacle_only_ = get_parameter("publish_obstacle_only").as_bool();
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();
    origin_z_ = get_parameter("origin_z").as_double();

    // octomap_server 默认 VOLATILE；这里必须匹配，否则收不到任何地图
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      in_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(&ESDFServer::OnOctomap, this, std::placeholders::_1));

    esdf_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);

    RCLCPP_INFO(
      get_logger(),
      "ESDF server 定稿: sub=%s pub=%s frame=%s max_dist=%.2f",
      in_topic.c_str(), out_topic.c_str(), frame_id_.c_str(), max_distance_);
  }

private:
  void OnOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "收到空 octomap");
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    std::unique_ptr<octomap::AbstractOcTree> abs(octomap_msgs::msgToMap(*msg));
    if (!abs) {
      RCLCPP_ERROR(get_logger(), "octomap 反序列化失败");
      return;
    }

    auto * tree = dynamic_cast<octomap::OcTree *>(abs.get());
    if (tree == nullptr) {
      RCLCPP_ERROR(get_logger(), "octomap 不是 OcTree");
      return;
    }

    // 分辨率强制与 OctoMap 对齐，避免体素索引漂移
    const double tree_res = tree->getResolution();
    if (!has_resolution_ || std::abs(tree_res - resolution_) > 1e-9) {
      if (has_resolution_) {
        RCLCPP_WARN(
          get_logger(),
          "OctoMap 分辨率变化 %.3f -> %.3f，重置 ESDF",
          resolution_, tree_res);
        ResetMaps();
      }
      resolution_ = tree_res;
      max_dist_voxels_ = std::max(1, static_cast<int>(std::ceil(max_distance_ / resolution_)));
      has_resolution_ = true;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "octomap frame=%s，ESDF 固定输出 frame=%s",
        msg->header.frame_id.c_str(), frame_id_.c_str());
    }

    std::unordered_set<Voxel, VoxelHash> current_obs;
    current_obs.reserve(prev_occupied_.size() + 1024);
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        current_obs.insert(WorldToVoxel(it.getX(), it.getY(), it.getZ()));
      }
    }

    if (first_frame_) {
      InitFullESDF(current_obs);
      first_frame_ = false;

      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      RCLCPP_INFO(
        get_logger(),
        "ESDF 首帧全量: obs=%zu voxels=%zu res=%.3f max_v=%d 用时=%.1fms",
        current_obs.size(), esdf_map_.size(), resolution_, max_dist_voxels_, ms);
    } else {
      std::vector<Voxel> insert_list;
      std::vector<Voxel> delete_list;
      DiffOccupied(current_obs, insert_list, delete_list);

      if (!insert_list.empty() || !delete_list.empty()) {
        IncrementalUpdate(current_obs, insert_list, delete_list);
      }

      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ESDF 增量: +%zu -%zu total=%zu 用时=%.1fms",
        insert_list.size(), delete_list.size(), esdf_map_.size(), ms);
    }

    prev_occupied_ = std::move(current_obs);
    PublishESDF(msg->header.stamp);
  }

  void ResetMaps()
  {
    esdf_map_.clear();
    prev_occupied_.clear();
    first_frame_ = true;
  }

  void InitFullESDF(const std::unordered_set<Voxel, VoxelHash> & obstacles)
  {
    esdf_map_.clear();
    std::queue<Voxel> q;
    for (const auto & v : obstacles) {
      esdf_map_[v] = 0.0f;
      q.push(v);
    }
    BfsPropagate(q);
  }

  void DiffOccupied(
    const std::unordered_set<Voxel, VoxelHash> & current,
    std::vector<Voxel> & insert_list,
    std::vector<Voxel> & delete_list) const
  {
    insert_list.clear();
    delete_list.clear();
    insert_list.reserve(64);
    delete_list.reserve(64);

    for (const auto & v : current) {
      if (prev_occupied_.find(v) == prev_occupied_.end()) {
        insert_list.push_back(v);
      }
    }
    for (const auto & v : prev_occupied_) {
      if (current.find(v) == current.end()) {
        delete_list.push_back(v);
      }
    }
  }

  void IncrementalUpdate(
    const std::unordered_set<Voxel, VoxelHash> & current_obs,
    const std::vector<Voxel> & insert_list,
    const std::vector<Voxel> & delete_list)
  {
    // Phase 1: 新障碍插入
    std::queue<Voxel> insert_q;
    for (const auto & v : insert_list) {
      esdf_map_[v] = 0.0f;
      insert_q.push(v);
    }
    BfsPropagate(insert_q);

    if (delete_list.empty()) {
      return;
    }

    // Phase 2: 删除障碍
    // 受影响区域：以被删障碍为中心，半径 max_dist 内的体素
    std::unordered_set<Voxel, VoxelHash> affected;
    affected.reserve(delete_list.size() * 64);

    for (const auto & dv : delete_list) {
      for (int dx = -max_dist_voxels_; dx <= max_dist_voxels_; ++dx) {
        for (int dy = -max_dist_voxels_; dy <= max_dist_voxels_; ++dy) {
          for (int dz = -max_dist_voxels_; dz <= max_dist_voxels_; ++dz) {
            if (std::abs(dx) + std::abs(dy) + std::abs(dz) > max_dist_voxels_) {
              continue;
            }
            affected.insert(Voxel{dv.x + dx, dv.y + dy, dv.z + dz});
          }
        }
      }
    }

    // 先把受影响区 raise：当前仍是障碍的保持 0，其余清空待重算
    for (const auto & v : affected) {
      if (current_obs.find(v) != current_obs.end()) {
        esdf_map_[v] = 0.0f;
      } else {
        esdf_map_.erase(v);
      }
    }

    // 从“受影响区边界上的存活障碍/已有距离场”reseed
    // 关键半径扩大 max_dist，确保能找回仍有效的障碍源
    std::queue<Voxel> reseed_q;
    std::unordered_set<Voxel, VoxelHash> seeded;
    seeded.reserve(affected.size());

    for (const auto & av : affected) {
      for (int dx = -max_dist_voxels_; dx <= max_dist_voxels_; ++dx) {
        for (int dy = -max_dist_voxels_; dy <= max_dist_voxels_; ++dy) {
          for (int dz = -max_dist_voxels_; dz <= max_dist_voxels_; ++dz) {
            if (std::abs(dx) + std::abs(dy) + std::abs(dz) > max_dist_voxels_) {
              continue;
            }
            const Voxel cand{av.x + dx, av.y + dy, av.z + dz};
            if (seeded.find(cand) != seeded.end()) {
              continue;
            }

            // 存活障碍是最可靠种子
            if (current_obs.find(cand) != current_obs.end()) {
              esdf_map_[cand] = 0.0f;
              reseed_q.push(cand);
              seeded.insert(cand);
              continue;
            }

            // 受影响区外、仍保留旧距离的体素也可作为边界种子
            if (affected.find(cand) == affected.end()) {
              const float d = GetDistance(cand);
              if (d >= 0.0f) {
                reseed_q.push(cand);
                seeded.insert(cand);
              }
            }
          }
        }
      }
    }

    BfsPropagate(reseed_q);
  }

  void BfsPropagate(std::queue<Voxel> & q)
  {
    while (!q.empty()) {
      const Voxel cur = q.front();
      q.pop();

      const float cur_dist = GetDistance(cur);
      if (cur_dist < 0.0f) {
        continue;
      }

      for (int i = 0; i < 6; ++i) {
        const Voxel nb{cur.x + DX[i], cur.y + DY[i], cur.z + DZ[i]};
        const float new_dist = cur_dist + 1.0f;
        if (new_dist > static_cast<float>(max_dist_voxels_)) {
          continue;
        }

        const float nb_dist = GetDistance(nb);
        if (nb_dist < 0.0f || new_dist < nb_dist) {
          esdf_map_[nb] = new_dist;
          q.push(nb);
        }
      }
    }
  }

  float GetDistance(const Voxel & v) const
  {
    const auto it = esdf_map_.find(v);
    if (it == esdf_map_.end()) {
      return -1.0f;
    }
    return it->second;
  }

  Voxel WorldToVoxel(double x, double y, double z) const
  {
    return Voxel{
      static_cast<int32_t>(std::lround((x - origin_x_) / resolution_)),
      static_cast<int32_t>(std::lround((y - origin_y_) / resolution_)),
      static_cast<int32_t>(std::lround((z - origin_z_) / resolution_))};
  }

  void VoxelToWorld(const Voxel & v, double & x, double & y, double & z) const
  {
    x = origin_x_ + static_cast<double>(v.x) * resolution_;
    y = origin_y_ + static_cast<double>(v.y) * resolution_;
    z = origin_z_ + static_cast<double>(v.z) * resolution_;
  }

  void PublishESDF(const builtin_interfaces::msg::Time & stamp)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(esdf_map_.size());

    for (const auto & [voxel, dist_voxels] : esdf_map_) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      VoxelToWorld(voxel, x, y, z);

      pcl::PointXYZI p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z);
      p.intensity = dist_voxels * static_cast<float>(resolution_);

      if (publish_obstacle_only_ && p.intensity < 1e-6f) {
        continue;
      }
      cloud.push_back(p);
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = stamp;
    esdf_pub_->publish(cloud_msg);
  }

  std::unordered_map<Voxel, float, VoxelHash> esdf_map_;
  std::unordered_set<Voxel, VoxelHash> prev_occupied_;
  bool first_frame_{true};
  bool has_resolution_{false};

  double resolution_{0.1};
  double max_distance_{2.0};
  int max_dist_voxels_{20};
  std::string frame_id_{"camera_init"};
  bool publish_obstacle_only_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
};

}  // namespace esdf_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<esdf_node::ESDFServer>());
  rclcpp::shutdown();
  return 0;
}
