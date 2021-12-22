/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include "grid_map_msgs/srv/get_grid_map.hpp"
#include "grid_map_msgs/srv/process_file.hpp"
#include "grid_map_msgs/srv/set_grid_map.hpp"
#include "grid_map_core/GridMap.hpp"

// ROS
/*#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>*/
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/cache.h"
#include "message_filters/subscriber.h"

// Eigen
//#include <Eigen/Core>
//#include <Eigen/Geometry>

// Boost
#include <boost/thread.hpp>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
/*#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"*/

#include <kindr/Core>
#include <chrono>

namespace elevation_mapping {

enum class InitializationMethods { PlanarFloorInitializer };

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ElevationMapping {
 public:
  /*!
   * Constructor.
   *
   * @param nodeHandle the ROS node handle.
   */
  explicit ElevationMapping(rclcpp::Node::SharedPtr nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
   * Callback function for new data to be added to the elevation map.
   *
   * @param pointCloudMsg    The point cloud to be fused with the existing data.
   * @param publishPointCloud If true, publishes the pointcloud after updating the map.
   * @param sensorProcessor_ The sensorProcessor to use in this callback.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pointCloudMsg);/*, bool publishPointCloud,
                          const SensorProcessorBase::Ptr& sensorProcessor_);*/

  /*!
   * Callback function for the update timer. Forces an update of the map from
   * the robot's motion if no new measurements are received for a certain time
   * period.
   *
   * @param timerEvent    The timer event.
   */
  void mapUpdateTimerCallback();

  /*!
  * Callback function for the fused map publish timer. Publishes the fused map
  * based on configurable duration.
  *
  * @param timerEvent    The timer event.
  */
  void publishFusedMapCallback();

  /*!
  * Callback function for cleaning map based on visibility ray tracing.
  *
  * @param timerEvent  The timer event.
  */
  void visibilityCleanupCallback();

    /*!
   * ROS service callback function to enable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  void enableUpdatesServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * ROS service callback function to disable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  void disableUpdatesServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                                     std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * ROS service callback function to clear all data of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  void clearMapServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * ROS service callback function to allow for setting the individual layers of the elevation map through a service call.
   * The layer mask can be used to only set certain cells and not the entire map. Cells
   * containing NAN in the mask are not set, all the others are set. If the layer mask is
   * not supplied, the entire map will be set in the intersection of both maps. The
   * provided map can be of different size and position than the map that will be altered.
   *
   * @param request    The ROS service request.
   * @param response   The ROS service response.
   * @return true if successful.
   */
  void maskedReplaceServiceCallback(std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request, 
                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> response);

  /*!
   * ROS service callback function to save the grid map with all layers to a ROS bag file.
   *
   * @param request   The ROS service request.
   * @param response  The ROS service response.
   * @return true if successful.
   */
  void saveMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, 
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  /*!
   * ROS service callback function to load the grid map with all layers from a ROS bag file.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  void loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, 
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

     /*!
   * ROS service callback function to trigger the fusion of the entire
   * elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   */
  void fuseEntireMapServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
  * ROS service callback function to return a submap of the fused elevation map.
  *
  * @param request     The ROS service request defining the location and size of the fused submap.
  * @param response    The ROS service response containing the requested fused submap.
  * @return true if successful.
  */
  void getFusedSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  /*!
  * ROS service callback function to return a submap of the raw elevation map.
  *
  * @param request     The ROS service request defining the location and size of the raw submap.
  * @param response    The ROS service response containing the requested raw submap.
  * @return true if successful.
  */
  void getRawSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);


 private:
  /**
  * Sets up the services.
  */
  void setupServices();  

  /*!
  * Reads and verifies the ROS parameters.
  *
  * @return true if successful.
  */
  bool readParameters();

  /**
  * Sets up the subscribers for both robot poses and input data.
  */
  void setupSubscribers();

   /**
   * Sets up the timers.
   */
  void setupTimers();

  /*!
  * Performs the initialization procedure.
  *
  * @return true if successful.
  */
  bool initialize();

  /*!
   * Separate thread for all fusion service calls.
   */
  void runFusionServiceThread();

  /*!
   * Separate thread for visibility cleanup.
   */
  void visibilityCleanupThread();

  /*!
  * Reset and start the map update timer.
  */
  void resetMapUpdateTimer();

    /*!
   * Update the elevation map from the robot motion up to a certain time.
   *
   * @param time    Time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const rclcpp::Time& time);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   *
   * @return true if successful.
   */
  bool updateMapLocation();

  /*!
   * Stop the map update timer.
   */
  void stopMapUpdateTimer();

  /*!
   * Returns true if fusing the map is enabled.
   */
  bool isFusingEnabled();

 
 protected:
  rclcpp::Node::SharedPtr nodeHandle_;

  //! ROS topics for subscriptions.
  std::string pointCloudTopic_;  //!< Deprecated, use input_source instead.
  std::string robotPoseTopic_;

  //! Point which the elevation map follows.
  kindr::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! Size of the cache for the robot pose messages.
  int robotPoseCacheSize_;

  //! If map is fused after every change for debugging/analysis purposes.
  bool isContinuouslyFusing_;

  //! Frame ID of the elevation map
  std::string mapFrameId_;

  //! Becomes true when corresponding poses and point clouds can be found
  bool receivedFirstMatchingPointcloudAndPose_;

  //! Name of the mask layer used in the masked replace service
  std::string maskedReplaceServiceMaskLayerName_;

  //! Enables initialization of the elevation map
  bool initializeElevationMap_;

  //! Enum to choose the initialization method
  int initializationMethod_;

  //! Width of submap of the elevation map with a constant height
  double lengthInXInitSubmap_;

  //! Height of submap of the elevation map with a constant height
  double lengthInYInitSubmap_;

  //! Margin of submap of the elevation map with a constant height
  double marginInitSubmap_;

  //! Target frame to get the init height of the elevation map
  std::string targetFrameInitSubmap_;

  //! Additional offset of the height value
  double initSubmapHeightOffset_;

  //! If true, robot motion updates are ignored.
  bool ignoreRobotMotionUpdates_;

  //! If false, elevation mapping stops updating
  bool updatesEnabled_;

  //! Maximum time that the map will not be updated.
  //rclcpp::Duration maxNoUpdateDuration_;
  std::chrono::duration<double, std::ratio<1, 1> > maxNoUpdateDuration_;

  //! Time tolerance for updating the map with data before the last update.
  //! This is useful when having multiple sensors adding data to the map.
  //rclcpp::Duration timeTolerance_;
  std::chrono::duration<double, std::ratio<1, 1> > timeTolerance_;

  //! Duration for the publishing the fusing map.
  //ros::Duration fusedMapPublishTimerDuration_;
  std::chrono::duration<double, std::ratio<1, 1> > fusedMapPublishTimerDuration_;

  //! Duration for the raytracing cleanup timer.
  //ros::Duration visibilityCleanupTimerDuration_;
  std::chrono::duration<double, std::ratio<1, 1> > visibilityCleanupTimerDuration_;

  //! Timer for the robot motion update.
  rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

  //! Timer for publishing the fused map.
  rclcpp::TimerBase::SharedPtr fusedMapPublishTimer_;

  //! Timer for the raytracing cleanup.
  rclcpp::TimerBase::SharedPtr visibilityCleanupTimer_;

  //! Input sources.
  //InputSourceManager inputSources_;

  //! ROS subscribers.
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped> robotPoseCache_;

  //! ROS service servers.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr fusionTriggerService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr fusedSubmapService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr rawSubmapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableUpdatesService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableUpdatesService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapService_;
  rclcpp::Service<grid_map_msgs::srv::SetGridMap>::SharedPtr maskedReplaceService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr saveMapService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr loadMapService_;

  /*!
  * Initializes a submap around the robot of the elevation map with a constant height.
  */
  bool initializeElevationMap();

    //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback thread for raytracing cleanup.
  boost::thread visibilityCleanupThread_;

  //! Callback queue for fusion service thread.
  //ros::CallbackQueue fusionServiceQueue_;

  //! TF listener and broadcaster.
  //tf::TransformListener transformListener_;

  //! Elevation map.
  ElevationMap map_;

  //! Sensor processors. Deprecated use the one from input sources instead.
  //SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  //RobotMotionMapUpdater robotMotionMapUpdater_;

  //! Time of the last point cloud update.
  rclcpp::Time lastPointCloudUpdateTime_;

  //! Callback queue for raytracing cleanup thread.
  //ros::CallbackQueue visibilityCleanupQueue_;
};

}  // namespace elevation_mapping
