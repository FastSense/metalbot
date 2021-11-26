#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#define BOOST_BIND_NO_PLACEHOLDERS

using namespace std::chrono_literals;
using std::placeholders::_1;

class PcdSubscriber : public rclcpp::Node
{
  public:
    PcdSubscriber()
    : Node("pointcloud_subscriber")
    {
      this->declare_parameter<std::string>("input_topic", "points");
      this->declare_parameter<std::string>("output_topic", "points_filtered");
      this->declare_parameter<float>("resolution", 0.01f);
      this->declare_parameter<bool>("verbose", true);
      this->get_parameter("input_topic", input_topic_);
      this->get_parameter("output_topic", output_topic_);
      this->get_parameter("resolution", resolution_);
      this->get_parameter("verbose", verbose_);
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10, 
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { pcd_callback(msg); }
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
      RCLCPP_INFO(this->get_logger(), "Input pcd topic: %s", input_topic_);
      RCLCPP_INFO(this->get_logger(), "Output pcd topic: %s", output_topic_);
      RCLCPP_INFO(this->get_logger(), "Filter resolution: %.3f", resolution_);
      RCLCPP_INFO(this->get_logger(), "Verbose: %d", verbose_);
    }

  private:
    void pcd_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg) const
    {
      std::vector<pcl::PointXYZRGB> points;
      pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
      pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
      pcl_conversions::toPCL(*msg, *cloud);
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (resolution_, resolution_, resolution_);
      sor.filter (*cloud_filtered);
      if (verbose_)
      {
        RCLCPP_INFO(this->get_logger(), "Input pcd width: %d", cloud->width);
        RCLCPP_INFO(this->get_logger(), "Filtered pcd width: %d", cloud->width);
      }

      std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_filtered_msg(new sensor_msgs::msg::PointCloud2 ());
      pcl_conversions::fromPCL(*cloud_filtered, *cloud_filtered_msg);
      publisher_->publish(std::move(cloud_filtered_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string input_topic_, output_topic_;
    float resolution_;
    bool verbose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdSubscriber>());
  rclcpp::shutdown();
}