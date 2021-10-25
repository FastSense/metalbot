#include "dogm_plugin/dogm_layer.h"
#include "dogm_msgs/msg/dynamic_occupancy_grid.hpp"
#include <iostream>

namespace dogm_plugin
{

DogmLayer::DogmLayer() {}

void DogmLayer::onInitialize() {
    params_.size = 50.0f;
    params_.resolution = 0.2f;
    params_.particle_count = 3 * static_cast<int>(1e6);
    params_.new_born_particle_count = 3 * static_cast<int>(1e5);
    params_.persistence_prob = 0.99f;
    params_.stddev_process_noise_position = 0.1f;
    params_.stddev_process_noise_velocity = 1.0f;
    params_.birth_prob = 0.02f;
    params_.stddev_velocity = 30.0f;
    params_.init_max_velocity = 30.0f;

    grid_map_ = std::make_unique<dogm::DOGM>(params_);
}

void DogmLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y,
                             double* max_x, double* max_y) {
    *min_x = 0;
    *min_y = 0;
    *max_x = 0;
    *max_y = 0;
    return;
}

void DogmLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                            int min_i, int min_j, int max_i, int max_j) {
    std::cout << "                123\n";
}

void DogmLayer::reset() {
    return;
}

}  // namespace dogm_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dogm_plugin::DogmLayer, nav2_costmap_2d::Layer)
