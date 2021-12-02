# DOGM plugin

## Overview

DOGM plugin to build dynamic occupancy map.

## Configuration

Parameters for DOGM are described [here](https://docs.google.com/document/d/13jvR1O_fP9Rz0Z_3aojrFFGzsZLsNm-Wte2rHdSkeFQ/edit?usp=sharing) in section "Настройка параметров DOGM".

There are also some parameters that are not listed in the doc above. Here they are.

| Parameter                      | Type  | Definition                                              |
|--------------------------------|-------|---------------------------------------------------------|
| opencv_visualization           | bool  | Whether to use opencv visualization for dynamic map     |
| motion_compensation            | bool  | Whether to use motion compensation                      |
| normalized_occupancy_threshold | float | Threshold for occupied state (occupancy is from 0 to 1) |

Example fully-described XML with default parameter values:

```
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      footprint: '[[0.14, 0.16], [0.14, -0.16], [-0.14, -0.16], [-0.14, 0.16]]'
      plugins: ["voxel_layer", "inflation_layer", "dogm_layer"]

      dogm_layer:
        plugin: "dogm_plugin/DogmLayer"
        enabled: True
        opencv_visualization: True
        motion_compensation: False
        size: 5.0
        resolution: 0.2
        particle_count: 3000000
        new_born_particle_count: 300000
        persistence_prob: 0.99
        stddev_process_noise_position: 0.1
        stddev_process_noise_velocity: 1.0
        birth_prob: 0.02
        stddev_velocity: 30.0
        init_max_velocity: 30.0
        normalized_occupancy_threshold: 0.5

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.75

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: point_cloud scan
        point_cloud:
          topic: /camera/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 2.0
          raytrace_min_range: 0.0
          obstacle_max_range: 1.5
          obstacle_min_range: 0.0
          min_obstacle_height: 0.1
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True
```

## Topics

| Topic      | Type                             | Description                  |
|------------|----------------------------------|------------------------------|
| `dogm_map` | `dogm_msgs/DynamicOccupancyGrid` | Output dynamic occupancy map |
