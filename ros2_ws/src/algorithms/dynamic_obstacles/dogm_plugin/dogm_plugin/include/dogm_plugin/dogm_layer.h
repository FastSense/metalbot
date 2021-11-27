#ifndef DOGM_LAYER_HPP_
#define DOGM_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <memory>
#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include "dogm_msgs/msg/dynamic_occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"

namespace dogm_plugin {

class DogmLayer : public nav2_costmap_2d::Layer {
public:
    DogmLayer();
    ~DogmLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j);
    void costMapToMeasurementGrid(nav2_costmap_2d::Costmap2D& master_grid,
                                  int min_i, int min_j, int max_i, int max_j);
    void publishDynamicGrid();
    virtual void reset();

private:
    bool opencv_visualization_;
    bool motion_compensation_;
    dogm::DOGM::Params params_;
    float normalized_threshold_;
    std::unique_ptr<dogm::DOGM> dogm_map_;
    dogm::MeasurementCell* measurement_grid_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<dogm_msgs::msg::DynamicOccupancyGrid>> publisher_;

    double robot_x_;
    double robot_y_;
    bool is_first_measurement_;
    rclcpp::Time last_time_stamp_;
};

}  // namespace dogm_plugin

#endif  // DOGM_LAYER_HPP_