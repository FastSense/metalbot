#ifndef DOGM_LAYER_HPP_
#define DOGM_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <memory>
#include <dogm/dogm.h>
#include <dogm/dogm_types.h>

namespace dogm_plugin
{

class DogmLayer : public nav2_costmap_2d::Layer
{
public:
    DogmLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j);
    virtual void reset();

private:
    dogm::DOGM::Params params_;
    std::unique_ptr<dogm::DOGM> grid_map_;
};

}  // namespace dogm_plugin

#endif  // DOGM_LAYER_HPP_