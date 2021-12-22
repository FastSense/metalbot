/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *  Note. Large parts are adopted from grid_map_demos/FiltersDemo.cpp.
 */

//#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  readParameters();

  publisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(outputTopic_, 1);

  // Setup filter chain.
  std::string plh;
  if (!nodeHandle->get_parameter(filterChainParametersName_, plh) || 
      !filterChain_.configure(filterChainParametersName_, nodeHandle->get_node_logging_interface(), nodeHandle->get_node_parameters_interface())) {
    RCLCPP_WARN(nodeHandle->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  nodeHandle_->declare_parameter("output_topic", std::string("elevation_map_raw"));
  nodeHandle_->get_parameter("output_topic", outputTopic_);
  nodeHandle_->declare_parameter("postprocessor_pipeline_name", std::string("postprocessor_pipeline"));
  nodeHandle_->get_parameter("postprocessor_pipeline_name", filterChainParametersName_);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (not filterChainConfigured_) {
    RCLCPP_WARN_ONCE(nodeHandle_->get_logger(), "No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }

  grid_map::GridMap outputMap;
  if (not filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  // Publish filtered output grid map.
  grid_map_msgs::msg::GridMap::SharedPtr outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(*outputMessage);
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

}  // namespace elevation_mapping
