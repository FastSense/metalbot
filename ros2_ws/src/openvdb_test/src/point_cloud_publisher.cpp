#include <chrono>
#include <memory>
#include <iostream>
#include <stdlib.h> 
// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#include "openvdb/tools/GridTransformer.h"
#include "openvdb/tools/RayIntersector.h"
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/Platform.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point.hpp"

// Custom
#include "grid_utils.h"

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
    _grid->setName("GridTest");
    _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
    _grid->setGridClass(openvdb::GRID_LEVEL_SET);

    openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
    openvdb::Coord xyz(5, 0, 0);
    accessor.setValue(xyz, 1.0);

    // create small sphere, not filled with points
    makeSphere(*_grid, 25, openvdb::Vec3f(50, 0, 0));
    makeLineAlongX(*_grid, 30, openvdb::Coord(-50, 0, 0));
    makeSquare(*_grid, 30, openvdb::Coord(-80, 0, 0));
    makeCube(*_grid, 30, openvdb::Coord(0, 30, 0));
    fillGridPoints();
    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));



  }


private:

  void fillGridPoints(){
    for (auto iter = _grid->cbeginValueOn(); iter.test(); ++iter) {
        openvdb::Vec3f worldPosition = _grid->transform().indexToWorld(iter.getCoord());
        geometry_msgs::msg::Point point;
        point.x = worldPosition[0];
        point.y = worldPosition[1];
        point.z = worldPosition[2];
        _grid_points.push_back(point);
    }
  }

  inline void GetOccupancyPointCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2){
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

    //const double & center_offset = _voxel_size / 2.0;
    const double & center_offset = 0.;
    for (auto it =
      _grid_points.begin();
      it != _grid_points.end(); ++it)
    {
      const geometry_msgs::msg::Point & pt = *it;
      *iter_x = pt.x + center_offset;
      *iter_y = pt.y + center_offset;
      *iter_z = pt.z;
      ++iter_x; ++iter_y; ++iter_z;
    }
  }

  void timer_callback()
  {
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2 = 
      std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc2->header.stamp = this->now();
    pc2->header.frame_id = _global_frame;
    GetOccupancyPointCloud(pc2);
    publisher_->publish(*pc2);
  }

  // OpenVDB
  float _voxel_size = 0.1;
  float _background_value = 0.;
  std::string _global_frame = "odom";
  mutable openvdb::FloatGrid::Ptr _grid;
  std::vector<geometry_msgs::msg::Point> _grid_points;
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
