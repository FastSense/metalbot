#include <chrono>
#include <memory>
#include <iostream>
// OpenVDB
#include <openvdb/openvdb.h>
#include "openvdb/tools/GridTransformer.h"
#include "openvdb/tools/RayIntersector.h"
#include <openvdb/tools/LevelSetSphere.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{

public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0){
    
    // initialize the OpenVDB Grid volume
    openvdb::initialize();
    // make it default to background value
    _grid = openvdb::FloatGrid::create(_background_value);

    // setup scale and tranform
    openvdb::Mat4d m = openvdb::Mat4d::identity();
    m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
    m.preTranslate(openvdb::Vec3d(0, 0, 0));
    m.preRotate(openvdb::math::Z_AXIS, 0);

    // setup transform and other metadata
    _grid->setTransform(openvdb::math::Transform::createLinearTransform(m));
    _grid->setName("SpatioTemporalVoxelLayer");
    _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
    _grid->setGridClass(openvdb::GRID_LEVEL_SET);


    openvdb::FloatGrid::Ptr _grid =
        openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
          /*radius=*/1,
          /*center=*/openvdb::Vec3f(0, 0, 0),
          /*voxel size=*/0.5);

    _grid->print();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

    for (auto iter = _grid->cbeginValueOn(); iter.test(); ++iter) {

      // Compute the world-space position of the point.
      //std::cout<<"TEST KEK"<<std::endl;
      // std::cout<<iter.getCoord()<<std::endl;
      //std::cout<<iter.getValue()<<std::endl;
      // openvdb::Vec3d pose_world = _grid->indexToWorld(pt);
      openvdb::Vec3f worldPosition = _grid->transform().indexToWorld(iter.getCoord());
      // std::cout<<worldPosition<<std::endl;
      geometry_msgs::msg::Point32 point;
      point.x = worldPosition[0];
      point.y = worldPosition[1];
      point.z = worldPosition[2];
      _grid_points.push_back(point);
      // std::cout<<" X: "<<point.x <<" Y: "<<point.y <<" Z: "<<point.z <<std::endl;
    }
  // std::cout<< "!!!!!!!!!!!!!!!!!!" <<std::endl;


  }


private:

inline void GetOccupancyPointCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points.size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  // std::cout<< "POINTS" <<std::endl;
  const double & center_offset = _voxel_size / 2.0;
  for (auto it =
    _grid_points.begin();
    it != _grid_points.end(); ++it)
  {
    const geometry_msgs::msg::Point32 & pt = *it;
    *iter_x = pt.x + center_offset;
    *iter_y = pt.y + center_offset;
    *iter_z = pt.z;
    
    std::cout<< "00000000000" <<std::endl;

    std::cout<<" X: "<<pt.x <<" Y: "<<pt.y <<" Z: "<<pt.z <<std::endl;
    ++iter_x; ++iter_y; ++iter_z;
  }
}

  void timer_callback()
  {
   // RCLCPP_INFO(this->get_logger(), "Publishing: points");

    std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2 =
     std::make_unique<sensor_msgs::msg::PointCloud2>();
    
    RCLCPP_INFO(this->get_logger(), "GET points");
    pc2->header.frame_id = "base_link";
    
    pc2->header.stamp = this->now();
    // std::cout<<this->now()<<std::endl;
    GetOccupancyPointCloud(pc2);
//    RCLCPP_INFO(this->get_logger(), "Publishing: points");

    publisher_->publish(*pc2);
    RCLCPP_INFO(this->get_logger(), "Publishing: points");

  }





  // OpenVDB
  float _voxel_size = 0.5;
  float _background_value = 0.;
  mutable openvdb::FloatGrid::Ptr _grid;
  std::vector<geometry_msgs::msg::Point32> _grid_points;
  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t count_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
