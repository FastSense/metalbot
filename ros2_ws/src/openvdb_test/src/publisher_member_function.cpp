#include <chrono>
#include <memory>
#include <iostream>
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

    // create sphere grid
    // openvdb::FloatGrid::Ptr _grid =
    //     openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
    //       /*radius=*/1.5,
    //       /*center=*/openvdb::Vec3f(5, 0, 0),
    //       /*voxel size=*/0.1);


    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

    for (auto iter = _grid->cbeginValueOn(); iter.test(); ++iter) {

      openvdb::Vec3f worldPosition = _grid->transform().indexToWorld(iter.getCoord());
      geometry_msgs::msg::Point point;
      point.x = worldPosition[0];
      point.y = worldPosition[1];
      point.z = worldPosition[2];
      _grid_points.push_back(point);
    }
    // print info about grid
    _grid->print();
  }

// from https://academysoftwarefoundation.github.io/openvdb/codeExamples.html
void makeSphere(openvdb::FloatGrid& grid, float radius, const openvdb::Vec3f& c)
{
    using ValueT = typename openvdb::FloatGrid::ValueType;
    // Distance value for the constant region exterior to the narrow band
    const ValueT outside = grid.background();
    // Distance value for the constant region interior to the narrow band
    // (by convention, the signed distance is negative in the interior of
    // a level set)
    const ValueT inside = -outside;
    // Use the background value as the width in voxels of the narrow band.
    // (The narrow band is centered on the surface of the sphere, which
    // has distance 0.)
    int padding = int(openvdb::math::RoundUp(openvdb::math::Abs(outside)));
    // The bounding box of the narrow band is 2*dim voxels on a side.
    int dim = int(radius + padding);
    // Get a voxel accessor.
    typename openvdb::FloatGrid::Accessor accessor = grid.getAccessor();
    // Compute the signed distance from the surface of the sphere of each
    // voxel within the bounding box and insert the value into the grid
    // if it is smaller in magnitude than the background value.
    openvdb::Coord ijk;
    int &i = ijk[0], &j = ijk[1], &k = ijk[2];
    for (i = c[0] - dim; i < c[0] + dim; ++i) {
        const float x2 = openvdb::math::Pow2(i - c[0]);
        for (j = c[1] - dim; j < c[1] + dim; ++j) {
            const float x2y2 = openvdb::math::Pow2(j - c[1]) + x2;
            for (k = c[2] - dim; k < c[2] + dim; ++k) {
                // The distance from the sphere surface in voxels
                const float dist = openvdb::math::Sqrt(x2y2
                    + openvdb::math::Pow2(k - c[2])) - radius;
                // Convert the floating-point distance to the grid's value type.
                ValueT val = ValueT(dist);
                // Only insert distances that are smaller in magnitude than
                // the background value.
                if (val < inside || outside < val) continue;
                // Set the distance for voxel (i,j,k).
                accessor.setValue(ijk, val);
            }
        }
    }
    // Propagate the outside/inside sign information from the narrow band
    // throughout the grid.
    openvdb::tools::signedFloodFill(grid.tree());
}


private:

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
      std::cout<< pt.x << " " << pt.y << " " << pt.z << " " <<std::endl;
      ++iter_x; ++iter_y; ++iter_z;
    }
  }

  void timer_callback()
  {
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc2->header.frame_id = "odom";
    
    pc2->header.stamp = this->now();
    GetOccupancyPointCloud(pc2);
    publisher_->publish(*pc2);
  }

  // OpenVDB
  float _voxel_size = 0.1;
  float _background_value = 0.;
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
