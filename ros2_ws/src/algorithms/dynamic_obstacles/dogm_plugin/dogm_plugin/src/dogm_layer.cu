#include "dogm_plugin/dogm_layer.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>

#include <iostream>

namespace dogm_plugin {

__global__ void setUnknownAsFree(cv::cuda::PtrStepSzi occupancy_grid);
__global__ void fillMeasurementGrid(dogm::MeasurementCell* __restrict__ measurement_grid, const cv::cuda::PtrStepSzi source,
                                    float occupancy_threshold);

DogmLayer::DogmLayer() {}

void DogmLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node_->get_parameter(name_ + "." + "enabled", enabled_);
    declareParameter("motion_compensation", rclcpp::ParameterValue(true));
    node_->get_parameter(name_ + "." + "motion_compensation", motion_compensation_);
    declareParameter("size", rclcpp::ParameterValue(50.0f));
    node_->get_parameter(name_ + "." + "size", params_.size);
    declareParameter("resolution", rclcpp::ParameterValue(0.2f));
    node_->get_parameter(name_ + "." + "resolution", params_.resolution);
    declareParameter("particle_count", rclcpp::ParameterValue(3 * static_cast<int>(1e6)));
    node_->get_parameter(name_ + "." + "particle_count", params_.particle_count);
    declareParameter("new_born_particle_count", rclcpp::ParameterValue(3 * static_cast<int>(1e5)));
    node_->get_parameter(name_ + "." + "new_born_particle_count", params_.new_born_particle_count);
    declareParameter("persistence_prob", rclcpp::ParameterValue(0.99f));
    node_->get_parameter(name_ + "." + "persistence_prob", params_.persistence_prob);
    declareParameter("stddev_process_noise_position", rclcpp::ParameterValue(0.1f));
    node_->get_parameter(name_ + "." + "stddev_process_noise_position", params_.stddev_process_noise_position);
    declareParameter("stddev_process_noise_velocity", rclcpp::ParameterValue(1.0f));
    node_->get_parameter(name_ + "." + "stddev_process_noise_velocity", params_.stddev_process_noise_velocity);
    declareParameter("birth_prob", rclcpp::ParameterValue(0.02f));
    node_->get_parameter(name_ + "." + "birth_prob", params_.birth_prob);
    declareParameter("stddev_velocity", rclcpp::ParameterValue(30.0f));
    node_->get_parameter(name_ + "." + "stddev_velocity", params_.stddev_velocity);
    declareParameter("init_max_velocity", rclcpp::ParameterValue(30.0f));
    node_->get_parameter(name_ + "." + "init_max_velocity", params_.init_max_velocity);

    dogm_map_ = std::make_unique<dogm::DOGM>(params_);
    CHECK_ERROR(cudaMalloc(&measurement_grid_, dogm_map_->grid_cell_count * sizeof(dogm::MeasurementCell)));

    robot_x_ = 0;
    robot_y_ = 0;
    is_first_measurement_ = true;

    // publisher_ = node_->create_publisher<dogm_msgs::msg::DynamicOccupancyGrid>("dogm_map", 10);
}

DogmLayer::~DogmLayer() {
    CHECK_ERROR(cudaFree(measurement_grid_));
}

void DogmLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y,
                             double* max_x, double* max_y) {
    *min_x = robot_x - dogm_map_->params.size / 2;
    *min_y = robot_y - dogm_map_->params.size / 2;
    *max_x = robot_x + dogm_map_->params.size / 2;
    *max_y = robot_y + dogm_map_->params.size / 2;
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    return;
}

void DogmLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                            int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }

    auto time_stamp = node_->now();
    costMapToMeasurementGrid(master_grid, min_i, min_j, max_i, max_j, 0.5);
    float robot_x = 0.f;
    float robot_y = 0.f;
    if (motion_compensation_) {
        robot_x = robot_x_;
        robot_y = robot_y_;
    }
    if (!is_first_measurement_) {
        float dt = (time_stamp - last_time_stamp_).seconds();
        dogm_map_->updateGrid(measurement_grid_, robot_x, robot_y, 0, dt);
    } else {
        dogm_map_->updateGrid(measurement_grid_, robot_x, robot_y, 0, 0);
        is_first_measurement_ = false;
    }
    last_time_stamp_ = time_stamp;

    cv::Mat occupancy_image = dogm_map_->getOccupancyImage();
    int vis_image_size_ = 600;
    float vis_occupancy_threshold_ = 0.6;
    float vis_mahalanobis_distance_ = 2.0;
    dogm_map_->drawVelocities(occupancy_image, vis_image_size_, 1., vis_occupancy_threshold_, vis_mahalanobis_distance_);
    cv::namedWindow("occupancy_image", cv::WINDOW_NORMAL);
    cv::imshow("occupancy_image", occupancy_image);
    cv::waitKey(1);
}

void DogmLayer::costMapToMeasurementGrid(nav2_costmap_2d::Costmap2D& master_grid,
                                         int min_i, int min_j, int max_i, int max_j,
                                         float occupancy_threshold) {
    unsigned int size_x = master_grid.getSizeInCellsX();
    unsigned int size_y = master_grid.getSizeInCellsY();
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    float measurement_grid_resolution = dogm_map_->params.resolution;
    float costmap_resolution = master_grid.getResolution();
    cv::Mat scale_measurement_grid(cv::Mat::eye(cv::Size(3, 3), CV_32F));
    float scale = costmap_resolution / measurement_grid_resolution;
    scale_measurement_grid.at<float>(0, 0) *= scale;
    scale_measurement_grid.at<float>(1, 1) *= scale;
    
    float measurement_grid_origin_x = robot_x_ - dogm_map_->params.size / 2;
    float measurement_grid_origin_y = robot_y_ - dogm_map_->params.size / 2;
    float costmap_origin_x = master_grid.getOriginX() + min_i * costmap_resolution;
    float costmap_origin_y = master_grid.getOriginY() + min_j * costmap_resolution;
    cv::Mat scaled_measurement_grid_to_costmap(cv::Mat::eye(cv::Size(3, 3), CV_32F));
    scaled_measurement_grid_to_costmap.at<float>(0, 2) = (costmap_origin_x - measurement_grid_origin_x) / costmap_resolution;
    scaled_measurement_grid_to_costmap.at<float>(1, 2) = (costmap_origin_y - measurement_grid_origin_y) / costmap_resolution;

    cv::Mat measurement_grid_to_costmap = scale_measurement_grid * scaled_measurement_grid_to_costmap;

    dim3 blocks(1, 1);
    dim3 threads(16, 16);
    unsigned char* master_array = master_grid.getCharMap();
    cv::Mat costmap(cv::Size(max_i - min_i, max_j - min_j), CV_8U, master_array + master_grid.getIndex(min_i, min_j), size_x * sizeof(unsigned char));
    costmap.convertTo(costmap, CV_32S);
    cv::cuda::GpuMat costmap_device;
    costmap_device.upload(costmap);
    setUnknownAsFree<<<blocks, threads>>>(costmap_device);

    cv::Mat measurement_grid;
    cv::cuda::GpuMat measurement_grid_device;
    cv::cuda::warpAffine(costmap_device, measurement_grid_device, measurement_grid_to_costmap(cv::Range(0, 2), cv::Range(0, 3)),
        cv::Size(dogm_map_->grid_size, dogm_map_->grid_size), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(nav2_costmap_2d::FREE_SPACE));
    fillMeasurementGrid<<<blocks, threads>>>(measurement_grid_, measurement_grid_device, occupancy_threshold);

    CHECK_ERROR(cudaGetLastError());
    CHECK_ERROR(cudaDeviceSynchronize());
}

void DogmLayer::publishDynamicGrid() {
    auto message = dogm_msgs::msg::DynamicOccupancyGrid();
    message.header.stamp = node_->now();
    message.header.frame_id = "abc";
    message.info.resolution = dogm_map_->getResolution();
    message.info.length = dogm_map_->getGridSize() * dogm_map_->getResolution();
    message.info.size = dogm_map_->getGridSize();
    message.info.pose.position.x = dogm_map_->getPositionX();
    message.info.pose.position.y = dogm_map_->getPositionY();
    message.info.pose.position.z = 0.0;
    message.info.pose.orientation.x = 0.0;
    message.info.pose.orientation.y = 0.0;
    message.info.pose.orientation.z = 0.0;
    message.info.pose.orientation.w = 1.0;

    message.data.clear();
    message.data.resize(dogm_map_->getGridSize() * dogm_map_->getGridSize());

    std::vector<dogm::GridCell> grid_cell_array = dogm_map_->getGridCells();
    #pragma omp parallel for
    for (int i = 0; i < message.data.size(); i++) {
        const dogm::GridCell& cell = grid_cell_array[i];

        message.data[i].free_mass = cell.free_mass;
        message.data[i].occ_mass = cell.occ_mass;

        message.data[i].mean_x_vel = cell.mean_x_vel;
        message.data[i].mean_y_vel = cell.mean_y_vel;
        message.data[i].var_x_vel = cell.var_x_vel;
        message.data[i].var_y_vel = cell.var_y_vel;
        message.data[i].covar_xy_vel = cell.covar_xy_vel;
    }

    publisher_->publish(message);
}

void DogmLayer::reset() {
    return;
}

__global__ void setUnknownAsFree(cv::cuda::PtrStepSzi occupancy_grid)
{
    int start_row = blockIdx.y * blockDim.y + threadIdx.y;
    int start_col = blockIdx.x * blockDim.x + threadIdx.x;
    int step_row = blockDim.y * gridDim.y;
    int step_col = blockDim.x * gridDim.x;
    for (int row = start_row; row < occupancy_grid.rows; row += step_row)
    {
        for (int col = start_col; col < occupancy_grid.cols; col += step_col)
        {
            if (occupancy_grid(row, col) == nav2_costmap_2d::NO_INFORMATION)
            {
                occupancy_grid(row, col) = nav2_costmap_2d::FREE_SPACE;
            }
        }
    }
}

__device__ float clip(float x, float min, float max)
{
    assert(min <= max);
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

__global__ void fillMeasurementGrid(dogm::MeasurementCell* __restrict__ measurement_grid, const cv::cuda::PtrStepSzi source,
                                    float occupancy_threshold)
{
    int start_row = blockIdx.y * blockDim.y + threadIdx.y;
    int start_col = blockIdx.x * blockDim.x + threadIdx.x;
    int step_row = blockDim.y * gridDim.y;
    int step_col = blockDim.x * gridDim.x;
    const float eps = 0.0001f;
    for (int row = start_row; row < source.rows; row += step_row)
    {
        for (int col = start_col; col < source.cols; col += step_col)
        {
            int index = col + row * source.cols;
            float occ = 1.0f * source(row, col) / nav2_costmap_2d::LETHAL_OBSTACLE;
            if (occ < occupancy_threshold)
            {
                measurement_grid[index].free_mass = clip(1 - occ, eps, 1 - eps);
                measurement_grid[index].occ_mass = eps;
            }
            else
            {
                measurement_grid[index].free_mass = eps;
                measurement_grid[index].occ_mass = clip(occ, eps, 1 - eps);
            }
            measurement_grid[index].likelihood = 1.0f;
            measurement_grid[index].p_A = 1.0f;
        }
    }
}

}  // namespace dogm_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dogm_plugin::DogmLayer, nav2_costmap_2d::Layer)
