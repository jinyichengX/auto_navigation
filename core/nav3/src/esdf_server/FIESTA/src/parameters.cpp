//
// Created by tommy on 5/2/19.
//

#include "parameters.h"

#include <cmath>

namespace {

template <typename T>
T GetParam(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value) {
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace

void fiesta::Parameters::SetParameters(const rclcpp::Node::SharedPtr &node) {
  resolution_ = GetParam<double>(node, "resolution", 0.1);
  visualize_every_n_updates_ = GetParam<int>(node, "visualize_every_n_updates", 1);
  min_ray_length_ = GetParam<double>(node, "min_ray_length", 0.5);
  max_ray_length_ = GetParam<double>(node, "max_ray_length", 10.0);

  double slice_vis_level_tmp = GetParam<double>(node, "slice_vis_level", 1.0);
  double vis_lower_bound_tmp = GetParam<double>(node, "vis_lower_bound", -1.0);
  double vis_upper_bound_tmp = GetParam<double>(node, "vis_upper_bound", 4.0);
  slice_vis_max_dist_ = GetParam<double>(node, "slice_vis_max_dist", 3.0);

  center_x_ = GetParam<double>(node, "center_x", 322.477357419);
  center_y_ = GetParam<double>(node, "center_y", 237.076346481);
  focal_x_ = GetParam<double>(node, "focal_x", 384.458089392);
  focal_y_ = GetParam<double>(node, "focal_y", 383.982755697);
  ray_cast_num_thread_ = GetParam<int>(node, "ray_cast_num_thread", 0);

  double radius_x = GetParam<double>(node, "radius_x", 5.0);
  double radius_y = GetParam<double>(node, "radius_y", 5.0);
  double radius_z = GetParam<double>(node, "radius_z", 2.5);
  radius_ = Eigen::Vector3d(radius_x, radius_y, radius_z);

  global_map_ = GetParam<bool>(node, "global_map", true);
  global_update_ = GetParam<bool>(node, "global_update", true);
  global_vis_ = GetParam<bool>(node, "global_vis", true);
  if (!global_map_) {
    global_vis_ = global_update_ = false;
  }

  use_depth_filter_ = GetParam<bool>(node, "use_depth_filter", false);
  depth_filter_tolerance_ = GetParam<double>(node, "depth_filter_tolerance", 0.1);
  depth_filter_max_dist_ = GetParam<double>(node, "depth_filter_max_dist", 10.0);
  depth_filter_min_dist_ = GetParam<double>(node, "depth_filter_min_dist", 0.1);
  depth_filter_margin_ = GetParam<int>(node, "depth_filter_margin", 0);

  frame_id_ = GetParam<std::string>(node, "frame_id", "camera_init");

#ifdef HASH_TABLE
  l_cornor_ << -100.f, -100.f, -100.f;
  r_cornor_ << 100.f, 100.f, 100.f;
  reserved_size_ = GetParam<int>(node, "reserved_size", 1000000);
#else
  double lx = GetParam<double>(node, "lx", -20.0);
  double ly = GetParam<double>(node, "ly", -20.0);
  double lz = GetParam<double>(node, "lz", -1.0);
  double rx = GetParam<double>(node, "rx", 20.0);
  double ry = GetParam<double>(node, "ry", 20.0);
  double rz = GetParam<double>(node, "rz", 4.0);

  l_cornor_ << lx, ly, lz;
  r_cornor_ << rx, ry, rz;
  map_size_ = r_cornor_ - l_cornor_;
#endif

  // Convert world-frame visualization heights into voxel indices.
  slice_vis_level_ = static_cast<int>(std::floor((slice_vis_level_tmp - l_cornor_.z()) / resolution_));
  vis_lower_bound_ = static_cast<int>(std::floor((vis_lower_bound_tmp - l_cornor_.z()) / resolution_));
  vis_upper_bound_ = static_cast<int>(std::floor((vis_upper_bound_tmp - l_cornor_.z()) / resolution_));

  update_esdf_every_n_sec_ = GetParam<double>(node, "update_esdf_every_n_sec", 0.1);

  // FAST-LIO body-frame LiDAR point cloud: identity extrinsics
  T_B_C_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
  T_D_B_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

#ifdef PROBABILISTIC
  p_hit_ = GetParam<double>(node, "p_hit", 0.70);
  p_miss_ = GetParam<double>(node, "p_miss", 0.35);
  p_min_ = GetParam<double>(node, "p_min", 0.12);
  p_max_ = GetParam<double>(node, "p_max", 0.97);
  p_occ_ = GetParam<double>(node, "p_occ", 0.80);
#endif
}
