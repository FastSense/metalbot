#ifndef DOGM_LAYER_HPP_
#define DOGM_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
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

    /**
     * @brief Converts costmap to measurement grid to feed it to DOGM.
     */
    void costMapToMeasurementGrid(nav2_costmap_2d::Costmap2D& master_grid,
                                  int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Converts dynamic map to custom ROS message and sends it.
     */
    void publishDynamicGrid();
    virtual void reset();

private:
    /**
     * @brief Computes transform from measurement grid (dynamic grid) coordinate system to costmap coordinate system.
     */
    cv::Mat getTransformFromMeasurementGridToCostMap(nav2_costmap_2d::Costmap2D& master_grid,
                                                     int min_i, int min_j, int max_i, int max_j);
    /**
     * @brief Gets data from master grid and transfers it to GPU.
     */
    cv::cuda::GpuMat getMasterArrayDevice(nav2_costmap_2d::Costmap2D& master_grid,
                                          int min_i, int min_j, int max_i, int max_j);
    /**
     * @brief Sets unknown cells in master grid on GPU as free.
     */
    void setUnknownAsFree(cv::cuda::GpuMat master_array_device);
    /**
     * @brief Transforms master grid to measurement grid (dynamic grid) system.
     */
    cv::cuda::GpuMat transformCostMapToMeasurementGrid(cv::cuda::GpuMat master_array_device, cv::Mat measurement_grid_to_costmap);
    /**
     * @brief Fills measurement grid to feed it to DOGM.
     */
    void fillMeasurementGrid(cv::cuda::GpuMat measurement_grid_device);

private:
    bool opencv_visualization_;
    bool motion_compensation_;
    dogm::DOGM::Params params_;
    float normalized_occupancy_threshold_;
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