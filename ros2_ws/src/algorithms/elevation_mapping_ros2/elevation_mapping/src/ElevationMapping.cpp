/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <string>

/*#include <grid_map_msgs/GridMap.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"*/
#include "elevation_mapping/ElevationMapping.hpp"
#include "rclcpp/logger.hpp"
#include <assert.h>

using namespace std::chrono_literals;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle)
      /*inputSources_(nodeHandle_),
      robotPoseCacheSize_(200),
      robotMotionMapUpdater_(nodeHandle),
      ignoreRobotMotionUpdates_(false),
      updatesEnabled_(true),
      isContinuouslyFusing_(false),
      receivedFirstMatchingPointcloudAndPose_(false),
      initializeElevationMap_(false),
      initializationMethod_(0),
      lengthInXInitSubmap_(1.2),
      lengthInYInitSubmap_(1.8),
      marginInitSubmap_(0.3),
      initSubmapHeightOffset_(0.0)*/ {
#ifndef NDEBUG
  // Print a warning if built in debug.
  RCLCPP_WARN(nodeHandle_->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node started.");

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  initialize();

  RCLCPP_INFO(nodeHandle_->get_logger(), "Successfully launched node.");
}

bool ElevationMapping::readParameters() {
  // ElevationMapping parameters.
  nodeHandle_->declare_parameter("point_cloud_topic", std::string("/points"));
  nodeHandle_->get_parameter("point_cloud_topic", pointCloudTopic_);
  nodeHandle_->declare_parameter("robot_pose_with_covariance_topic", std::string("/pose"));
  nodeHandle_->get_parameter("robot_pose_with_covariance_topic", robotPoseTopic_);
  nodeHandle_->declare_parameter("track_point_frame_id", std::string("/roobt"));
  nodeHandle_->get_parameter("track_point_frame_id", trackPointFrameId_);
  nodeHandle_->declare_parameter("track_point_x", 0.0);
  nodeHandle_->get_parameter("track_point_x", trackPoint_.x());
  nodeHandle_->declare_parameter("track_point_y", 0.0);
  nodeHandle_->get_parameter("track_point_y", trackPoint_.y());
  nodeHandle_->declare_parameter("track_point_z", 0.0);
  nodeHandle_->get_parameter("track_point_z", trackPoint_.z());

  nodeHandle_->declare_parameter("robot_pose_cache_size", 200);
  nodeHandle_->get_parameter("robot_pose_cache_size", robotPoseCacheSize_);
  assert(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_->declare_parameter("min_update_rate", 2.0);
  nodeHandle_->get_parameter("min_update_rate", minUpdateRate);
  if (minUpdateRate == 0.0) {
    //maxNoUpdateDuration_.from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for publishing the map is zero.");
  } else {
    //maxNoUpdateDuration_.from_seconds(1.0 / minUpdateRate);
  }
  assert(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_->declare_parameter("time_tolerance", 0.0);
  nodeHandle_->get_parameter("time_tolerance", timeTolerance);
  //timeTolerance_.from_seconds(timeTolerance);

  double fusedMapPublishingRate;
  nodeHandle_->declare_parameter("fused_map_publishing_rate", 1.0);
  nodeHandle_->get_parameter("fused_map_publishing_rate", fusedMapPublishingRate);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_ = std::chrono::duration<double, std::ratio<1, 1> >(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    isContinuouslyFusing_ = true;
    fusedMapPublishTimerDuration_ = std::chrono::duration<double, std::ratio<1, 1> >(0.0);
  } else {
    fusedMapPublishTimerDuration_ = std::chrono::duration<double, std::ratio<1, 1> >(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate;
  nodeHandle_->declare_parameter("visibility_cleanup_rate", 1.0);
  nodeHandle_->get_parameter("visibility_cleanup_rate", visibilityCleanupRate);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_ = std::chrono::duration<double, std::ratio<1, 1> >(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    visibilityCleanupTimerDuration_ = std::chrono::duration<double, std::ratio<1, 1> >(1.0 / visibilityCleanupRate);
    //map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  nodeHandle_->declare_parameter("map_frame_id", std::string("/map"));
  nodeHandle_->get_parameter("map_frame_id", mapFrameId_);
  //map_.setFrameId(mapFrameId_);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_->declare_parameter("length_in_x", 1.5);
  nodeHandle_->get_parameter("length_in_x", length(0));
  nodeHandle_->declare_parameter("length_in_y", 1.5);
  nodeHandle_->get_parameter("length_in_y", length(1));
  nodeHandle_->declare_parameter("position_x", 0.0);
  nodeHandle_->get_parameter("position_x", position.x());
  nodeHandle_->declare_parameter("position_y", 0.0);
  nodeHandle_->get_parameter("position_y", position.y());
  nodeHandle_->declare_parameter("resolution", 0.01);
  nodeHandle_->get_parameter("resolution", resolution);
  //map_.setGeometry(length, resolution, position);

  nodeHandle_->declare_parameter("min_variance", pow(0.003, 2));
  //nodeHandle_->get_parameter("min_variance", map_.minVariance_);
  nodeHandle_->declare_parameter("max_variance", pow(0.03, 2));
  //nodeHandle_->get_parameter("max_variance", map_.maxVariance_);
  nodeHandle_->declare_parameter("mahalanobis_distance_threshold", 2.5);
  //nodeHandle_->get_parameter("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_);
  nodeHandle_->declare_parameter("multi_height_noise", pow(0.003, 2));
  //nodeHandle_->get_parameter("multi_height_noise", map_.multiHeightNoise_);
  nodeHandle_->declare_parameter("min_horizontal_variance", pow(resolution / 2.0, 2));
  //nodeHandle_->get_parameter("min_horizontal_variance", map_.minHorizontalVariance_);
  nodeHandle_->declare_parameter("max_horizontal_variance", 0.5);
  //nodeHandle_->get_parameter("max_horizontal_variance", map_.maxHorizontalVariance_);
  nodeHandle_->declare_parameter("underlying_map_topic", std::string());
  //nodeHandle_->get_parameter("underlying_map_topic", map_.underlyingMapTopic_);
  nodeHandle_->declare_parameter("enable_visibility_cleanup", true);
  //nodeHandle_->get_parameter("enable_visibility_cleanup", map_.enableVisibilityCleanup_);
  nodeHandle_->declare_parameter("enable_continuous_cleanup", false);
  //nodeHandle_->get_parameter("enable_continuous_cleanup", map_.enableContinuousCleanup_);
  nodeHandle_->declare_parameter("scanning_duration", 1.0);
  //nodeHandle_->get_parameter("scanning_duration", map_.scanningDuration_);
  nodeHandle_->declare_parameter("masked_replace_service_mask_layer_name", std::string("mask"));
  nodeHandle_->get_parameter("masked_replace_service_mask_layer_name", maskedReplaceServiceMaskLayerName_);

  // Settings for initializing elevation map
  nodeHandle_->declare_parameter("initialize_elevation_map", false);
  nodeHandle_->get_parameter("initialize_elevation_map", initializeElevationMap_);
  nodeHandle_->declare_parameter("initialization_method", 0);
  nodeHandle_->get_parameter("initialization_method", initializationMethod_);
  nodeHandle_->declare_parameter("length_in_x_init_submap", 1.2);
  nodeHandle_->get_parameter("length_in_x_init_submap", lengthInXInitSubmap_);
  nodeHandle_->declare_parameter("length_in_y_init_submap", 1.8);
  nodeHandle_->get_parameter("length_in_y_init_submap", lengthInYInitSubmap_);
  nodeHandle_->declare_parameter("margin_init_submap", 0.3);
  nodeHandle_->get_parameter("margin_init_submap", marginInitSubmap_);
  nodeHandle_->declare_parameter("init_submap_height_offset", 0.0);
  nodeHandle_->get_parameter("init_submap_height_offset", initSubmapHeightOffset_);
  nodeHandle_->declare_parameter("target_frame_init_submap", std::string("/footprint"));
  nodeHandle_->get_parameter("target_frame_init_submap", targetFrameInitSubmap_);

  // SensorProcessor parameters. Deprecated, use the sensorProcessor from within input sources instead!
  std::string sensorType;
  nodeHandle_->declare_parameter("sensor_processor/type", std::string("structured_light"));
  nodeHandle_->get_parameter("sensor_processor/type", sensorType);

  /*SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_.param("robot_base_frame_id", std::string("/robot")),
                                                                      mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) {
    return false;
  }
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }*/

  return true;
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pointCloudMsg) {/*, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor_) {*/
  /*ROS_DEBUG("Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(ros::Time::now());
      map_.publishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().toSec();
    const double currentPointCloudTime = pointCloudMsg->header.stamp.toSec();

    if (currentPointCloudTime < oldestPoseTime) {
      ROS_WARN_THROTTLE(5, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      ROS_INFO("First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
        ROS_ERROR("The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                  lastPointCloudUpdateTime_.toSec());
      } else {
        ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor_->isTfAvailableInBuffer()) {
      ROS_INFO_THROTTLE(10, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    ROS_ERROR("Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Clear the map if continuous clean-up was enabled.
  if (map_.enableContinuousCleanup_) {
    ROS_DEBUG("Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    ROS_ERROR("Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_.publishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();*/
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  //const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = nodeHandle_->has_parameter("point_cloud_topic");
  if (hasDeprecatedPointcloudTopic) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  if (/*!configuredInputSources && */hasDeprecatedPointcloudTopic) {
    nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointCloudTopic_, 1,
        //std::bind(&ElevationMapping::pointCloudCallback, this, std::placeholders::_1/*, true, std::ref(sensorProcessor_)*/));
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { ElevationMapping::pointCloudCallback(msg); }
    );
  }
  /*if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  }*/

  if (!robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::mapUpdateTimerCallback() {
  /*if (!updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(ros::Time::now());
    map_.publishRawElevationMap();
    return;
  }

  ros::Time time = ros::Time::now();
  if ((lastPointCloudUpdateTime_ - time) <= maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
    return;
  }
  ROS_WARN_THROTTLE(5, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.publishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();*/
}

void ElevationMapping::publishFusedMapCallback() {
  /*if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  ROS_DEBUG("Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();*/
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  //map_.visibilityCleanup(ros::Time(lastPointCloudUpdateTime_));
}

void ElevationMapping::setupTimers() {
  mapUpdateTimer_ = nodeHandle_->create_wall_timer(maxNoUpdateDuration_, std::bind(&ElevationMapping::mapUpdateTimerCallback, this));

  /*if (!fusedMapPublishTimerDuration_.isZero()) {
    ros::TimerOptions timerOptions =
        ros::TimerOptions(fusedMapPublishTimerDuration_, boost::bind(&ElevationMapping::publishFusedMapCallback, this, _1),
                          &fusionServiceQueue_, false, false);
    fusedMapPublishTimer_ = nodeHandle_.createTimer(timerOptions);
  }*/
  fusedMapPublishTimer_ = nodeHandle_->create_wall_timer(fusedMapPublishTimerDuration_, std::bind(&ElevationMapping::publishFusedMapCallback, this));

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  /*if (map_.enableVisibilityCleanup_ && !visibilityCleanupTimerDuration_.isZero() && !map_.enableContinuousCleanup_) {
    ros::TimerOptions timerOptions =
        ros::TimerOptions(visibilityCleanupTimerDuration_, boost::bind(&ElevationMapping::visibilityCleanupCallback, this, _1),
                          &visibilityCleanupQueue_, false, false);
    visibilityCleanupTimer_ = nodeHandle_.createTimer(timerOptions);
  }*/
  visibilityCleanupTimer_ = nodeHandle_->create_wall_timer(visibilityCleanupTimerDuration_, std::bind(&ElevationMapping::visibilityCleanupCallback, this));
}

void ElevationMapping::fuseEntireMapServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/, 
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  //boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  //map_.fuseAll();
  //map_.publishFusedElevationMap();
}

void ElevationMapping::getFusedSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  /*grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;*/
}

void ElevationMapping::getRawSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  /*grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess;
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;*/
}

void ElevationMapping::clearMapServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/, 
                                               std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  /*ROS_INFO("Clearing map...");
  bool success = map_.clear();
  success &= initializeElevationMap();
  ROS_INFO("Map cleared.");

  return success;*/
}

void ElevationMapping::maskedReplaceServiceCallback(std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> response) {
  /*ROS_INFO("Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request.map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      ROS_ERROR("Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;*/
}

void ElevationMapping::saveMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  /*ROS_INFO("Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = nodeHandle_.getNamespace() + "/elevation_map";
  if (!request.topic_name.empty()) {
    topic = nodeHandle_.getNamespace() + "/" + request.topic_name;
  }
  response.success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request.file_path, topic));
  response.success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request.file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response.success));
  return static_cast<bool>(response.success);*/
}

void ElevationMapping::loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  /*ROS_WARN("Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_.getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_.getRawDataMutex());

  std::string topic = nodeHandle_.getNamespace();
  if (!request.topic_name.empty()) {
    topic += "/" + request.topic_name;
  } else {
    topic += "/elevation_map";
  }

  response.success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request.file_path, topic, map_.getFusedGridMap()));
  response.success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request.file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
      static_cast<bool>(response.success));

  // Update timestamp for visualization in ROS
  map_.setTimestamp(ros::Time::now());
  map_.publishRawElevationMap();
  return static_cast<bool>(response.success);*/
}

void ElevationMapping::disableUpdatesServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> , 
                                                     std::shared_ptr<std_srvs::srv::Empty::Response> ) {
  /*ROS_INFO("Disabling updates.");
  updatesEnabled_ = false;
  return true;*/
}

void ElevationMapping::enableUpdatesServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/, 
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  /*ROS_INFO("Enabling updates.");
  updatesEnabled_ = true;
  return true;*/
}

void ElevationMapping::setupServices() {
  // Multi-threading for fusion.
  /*ros::AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "trigger_fusion", boost::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForTriggerFusion);*/
  fusionTriggerService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("trigger_fusion", 
                                       std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, 
                                                  std::placeholders::_1, std::placeholders::_2));

  /*ros::AdvertiseServiceOptions advertiseServiceOptionsForGetFusedSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_submap", boost::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, _1, _2), ros::VoidConstPtr(), &fusionServiceQueue_);
  fusedSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetFusedSubmap);*/
  fusedSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>("get_submap", 
                                      std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, 
                                                  std::placeholders::_1, std::placeholders::_2));

  /*ros::AdvertiseServiceOptions advertiseServiceOptionsForGetRawSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_raw_submap", boost::bind(&ElevationMapping::getRawSubmapServiceCallback, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  rawSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetRawSubmap);*/
  rawSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>("get_raw_submap",
                                    std::bind(&ElevationMapping::getRawSubmapServiceCallback, this, 
                                                  std::placeholders::_1, std::placeholders::_2));

  clearMapService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("clear_map", 
                                  std::bind(&ElevationMapping::clearMapServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  enableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("enable_updates", 
                                  std::bind(&ElevationMapping::enableUpdatesServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  disableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("disable_updates", 
                                  std::bind(&ElevationMapping::disableUpdatesServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  maskedReplaceService_ = nodeHandle_->create_service<grid_map_msgs::srv::SetGridMap>("masked_replace", 
                                  std::bind(&ElevationMapping::maskedReplaceServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  saveMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>("save_map", 
                                  std::bind(&ElevationMapping::saveMapServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  loadMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>("load_map", 
                                  std::bind(&ElevationMapping::loadMapServiceCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(boost::bind(&ElevationMapping::runFusionServiceThread, this));
  std::this_thread::sleep_for(std::chrono::seconds(1));  // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  visibilityCleanupThread_ = boost::thread(boost::bind(&ElevationMapping::visibilityCleanupThread, this));
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  /*ros::Rate loopRate(20);

  while (nodeHandle_.ok()) {
    fusionServiceQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }*/
}

void ElevationMapping::visibilityCleanupThread() {
  /*ros::Rate loopRate(20);

  while (nodeHandle_.ok()) {
    visibilityCleanupQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }*/
}

bool ElevationMapping::initializeElevationMap() {
  /*if (initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      tf::StampedTransform transform;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {
        transformListener_.waitForTransform(mapFrameId_, targetFrameInitSubmap_, ros::Time(0), ros::Duration(5.0));
        transformListener_.lookupTransform(mapFrameId_, targetFrameInitSubmap_, ros::Time(0), transform);
        ROS_DEBUG_STREAM("Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z() + initSubmapHeightOffset_, lengthInXInitSubmap_,
                                lengthInYInitSubmap_, marginInitSubmap_);
        return true;
      } catch (tf::TransformException& ex) {
        ROS_DEBUG("%s", ex.what());
        ROS_WARN("Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }*/
  return true;
}

void ElevationMapping::resetMapUpdateTimer() {
  /*mapUpdateTimer_.stop();
  ros::Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) {
    periodSinceLastUpdate.fromSec(0.0);
  }
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();*/
}

bool ElevationMapping::isFusingEnabled() {
  //return isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  /*if (ignoreRobotMotionUpdates_) {
    return true;
  }

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(),
              map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
      ROS_ERROR("The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                lastPointCloudUpdateTime_.toSec());
    } else {
      ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);*/

  return true;
}

bool ElevationMapping::updateMapLocation() {
  /*ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
  kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);*/
  return true;
}

void ElevationMapping::stopMapUpdateTimer() {
  //mapUpdateTimer_.stop();
}

ElevationMapping::~ElevationMapping() {
  // Shutdown all services.

  {  // Fusion Service Queue
    //rawSubmapService_->shutdown();
    //fusionTriggerService_->shutdown();
    //fusedSubmapService_->shutdown();
    //fusedMapPublishTimer_.stop();

    //fusionServiceQueue_.disable();
    //fusionServiceQueue_.clear();
  }

  {  // Visibility cleanup queue
    /*visibilityCleanupTimer_.stop();

    visibilityCleanupQueue_.disable();
    visibilityCleanupQueue_.clear();*/
  }

  /*nodeHandle_.shutdown();

  // Join threads.
  if (fusionServiceThread_.joinable()) {
    fusionServiceThread_.join();
  }
  if (visibilityCleanupThread_.joinable()) {
    visibilityCleanupThread_.join();
  }*/
}

}  // namespace elevation_mapping
