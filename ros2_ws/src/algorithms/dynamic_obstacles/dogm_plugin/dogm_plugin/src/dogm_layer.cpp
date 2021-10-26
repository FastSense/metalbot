#include "dogm_plugin/dogm_layer.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>

#include <iostream>

namespace dogm_plugin {

DogmLayer::DogmLayer() {}

void DogmLayer::onInitialize() {
    params_.size = 20.0f;
    params_.resolution = 0.2f;
    params_.particle_count = 3 * static_cast<int>(1e6);
    params_.new_born_particle_count = 3 * static_cast<int>(1e5);
    params_.persistence_prob = 0.99f;
    params_.stddev_process_noise_position = 0.1f;
    params_.stddev_process_noise_velocity = 1.0f;
    params_.birth_prob = 0.02f;
    params_.stddev_velocity = 30.0f;
    params_.init_max_velocity = 30.0f;

    dogm_map_ = std::make_unique<dogm::DOGM>(params_);
    measurement_grid_.resize(dogm_map_->grid_cell_count);

    is_first_measurement_ = true;

    // publisher_ = node_->create_publisher<dogm_msgs::msg::DynamicOccupancyGrid>("dogm_map", 10);
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
    auto time_stamp = node_->now();
    costMapToMeasurementGrid(master_grid, min_i, min_j, max_i, max_j);
    if (!is_first_measurement_) {
        float dt = (time_stamp - last_time_stamp_).seconds();
        dogm_map_->updateGrid(measurement_grid_.data(), 0, 0, 0, dt, false);
    } else {
        dogm_map_->updateGrid(measurement_grid_.data(), 0, 0, 0, 0, false);
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
                                         int min_i, int min_j, int max_i, int max_j) {
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
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
    float costmap_origin_x = master_grid.getOriginX();
    float costmap_origin_y = master_grid.getOriginY();
    std::cout << "robot_position\n";
    std::cout << robot_x_ << ' ' << robot_y_ << std::endl;
    std::cout << "costmap_origin\n";
    std::cout << costmap_origin_x << ' ' << costmap_origin_y << std::endl;
    std::cout << "costmap_size_in_meters\n";
    std::cout << master_grid.getSizeInMetersX() << ' ' << master_grid.getSizeInMetersY() << std::endl;
    std::cout << std::endl;
    cv::Mat scaled_measurement_grid_to_costmap(cv::Mat::eye(cv::Size(3, 3), CV_32F));
    scaled_measurement_grid_to_costmap.at<float>(0, 2) = (costmap_origin_x - measurement_grid_origin_x) / costmap_resolution;
    scaled_measurement_grid_to_costmap.at<float>(1, 2) = (costmap_origin_y - measurement_grid_origin_y) / costmap_resolution;

    cv::Mat measurement_grid_to_costmap = scale_measurement_grid * scaled_measurement_grid_to_costmap;

    cv::Mat costmap(cv::Size(size_x, size_y), CV_8U, master_array);
    costmap.convertTo(costmap, CV_32S);
	cv::cuda::GpuMat costmap_device;
    costmap_device.upload(costmap);

    cv::Mat measurement_grid;
    cv::cuda::GpuMat measurement_grid_device;
    cv::cuda::warpAffine(costmap_device, measurement_grid_device, measurement_grid_to_costmap(cv::Range(0, 2), cv::Range(0, 3)),
		cv::Size(dogm_map_->grid_size, dogm_map_->grid_size), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1));
    measurement_grid_device.download(measurement_grid);

    const float eps = 0.0001;
    float occupancy_threshold = 0.5;
	for (int x = 0; x < dogm_map_->grid_size; x++)
	{
		for (int y = 0; y < dogm_map_->grid_size; y++)
		{
			int index = x + y * dogm_map_->grid_size;
			float occ = measurement_grid.at<int>(y, x) / 100.;
			if (occ < 0)
			{
				measurement_grid_[index].free_mass = eps;
				measurement_grid_[index].occ_mass = eps;
			}
			else if (occ < occupancy_threshold)
			{
				measurement_grid_[index].free_mass = std::max(eps, std::min(1 - eps, 1 - occ));
				measurement_grid_[index].occ_mass = eps;
			}
			else
			{
				measurement_grid_[index].free_mass = eps;
				measurement_grid_[index].occ_mass = std::max(eps, std::min(1 - eps, occ));
			}
			measurement_grid_[index].likelihood = 1.0f;
			measurement_grid_[index].p_A = 1.0f;
		}
	}
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

}  // namespace dogm_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dogm_plugin::DogmLayer, nav2_costmap_2d::Layer)
