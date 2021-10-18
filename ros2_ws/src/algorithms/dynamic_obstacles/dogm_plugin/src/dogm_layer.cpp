#include "dogm_plugin/dogm_layer.h"
#include <iostream>

namespace dogm_plugin
{

DogmLayer::DogmLayer() {}

void DogmLayer::onInitialize() {
    return;
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

void DogmLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                            int min_i, int min_j, int max_i, int max_j) {
    std::cout << "                123\n";
}

void DogmLayer::reset() {
    return;
}

}  // namespace dogm_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dogm_plugin::DogmLayer, nav2_costmap_2d::Layer)
