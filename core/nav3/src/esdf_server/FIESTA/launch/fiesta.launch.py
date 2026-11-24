from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nav3",
                executable="fiesta_esdf",
                name="fiesta",
                output="screen",
                parameters=[
                    {
                        # 0.1m keeps update fast enough for global incremental ESDF
                        "resolution": 0.1,
                        "update_esdf_every_n_sec": 0.1,
                        "reserved_size": 1000000,
                        # Global fixed map box in camera_init
                        "lx": -20.0,
                        "ly": -20.0,
                        "lz": -1.0,
                        "rx": 20.0,
                        "ry": 20.0,
                        "rz": 4.0,
                        "min_ray_length": 0.3,
                        # Longer rays => faster coverage of global free/occ space
                        "max_ray_length": 10.0,
                        # Must stay 0: occupancy_queue_ is not thread-safe
                        "ray_cast_num_thread": 0,
                        "center_x": 3.2247735741936430e02,
                        "center_y": 2.3707634648111778e02,
                        "focal_x": 3.8445808939187248e02,
                        "focal_y": 3.8398275569654390e02,
                        "p_hit": 0.70,
                        "p_miss": 0.35,
                        "p_min": 0.12,
                        "p_max": 0.97,
                        "p_occ": 0.80,
                        "global_map": True,
                        "global_update": True,
                        "global_vis": True,
                        "radius_x": 5.0,
                        "radius_y": 5.0,
                        "radius_z": 2.5,
                        "use_depth_filter": False,
                        "depth_filter_tolerance": 0.1,
                        "depth_filter_max_dist": 10.0,
                        "depth_filter_min_dist": 0.1,
                        "depth_filter_margin": 0,
                        "visualize_every_n_updates": 1,
                        "slice_vis_max_dist": 3.0,
                        # World-frame heights in camera_init
                        "slice_vis_level": 1.0,
                        "vis_lower_bound": -0.5,
                        "vis_upper_bound": 4.0,
                        "frame_id": "camera_init",
                    }
                ],
                remappings=[
                    ("depth", "/cloud_registered_body"),
                    ("transform", "/Odometry"),
                ],
            )
        ]
    )
