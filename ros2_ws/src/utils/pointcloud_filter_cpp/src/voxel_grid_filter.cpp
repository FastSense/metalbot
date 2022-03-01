#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <chrono>

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
      this->declare_parameter<bool>("filter_outliers", true);
      this->declare_parameter<float>("search_radius", 0.15f);
      this->declare_parameter<int>("min_neighbors_in_radius", 2);
      this->declare_parameter<bool>("verbose", true);
      this->get_parameter("input_topic", input_topic_);
      this->get_parameter("output_topic", output_topic_);
      this->get_parameter("resolution", resolution_);
      this->get_parameter("filter_outliers", filter_outliers_);
      this->get_parameter("search_radius", search_radius_);
      this->get_parameter("min_neighbors_in_radius", min_neighbors_in_radius_);
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
      auto t1 = std::chrono::high_resolution_clock::now();
      pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
      pcl::PCLPointCloud2::Ptr cloud_reduced (new pcl::PCLPointCloud2 ());
      pcl_conversions::toPCL(*msg, *cloud);
      pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
      vg.setInputCloud (cloud);
      vg.setLeafSize (resolution_, resolution_, resolution_);
      vg.filter (*cloud_reduced);
      auto t2 = std::chrono::high_resolution_clock::now();
      if (verbose_)
      {
        std::chrono::duration<double> dt = t2 - t1;
        RCLCPP_INFO(this->get_logger(), "Time to launch voxel grid: %f", dt.count());
      }

      std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_filtered_msg(new sensor_msgs::msg::PointCloud2 ());
      if (filter_outliers_)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_reduced_pcd (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_for_filter (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_resulting (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::fromPCLPointCloud2(*cloud_reduced, *cloud_reduced_pcd);
        cloud_reduced_pcd->header = cloud_reduced->header;
        for (auto pt : cloud_reduced_pcd->points)
        {
          if ((pt.z < 2) && (pt.y < 0.2))
          {
            cloud_for_filter->points.push_back(pt);
          }
          else
          {
            cloud_resulting->points.push_back(pt);
          }
        }
        cloud_for_filter->width = cloud_for_filter->points.size();
        cloud_for_filter->height = 1;
        cloud_for_filter->header = cloud_reduced->header;
        cloud_resulting->width = cloud_resulting->points.size();
        cloud_resulting->height = 1;
        cloud_resulting->header = cloud_reduced->header;
        auto t3 = std::chrono::high_resolution_clock::now();
        if (verbose_)
        {
          RCLCPP_INFO(this->get_logger(), "Full cloud width: %d; reduced cloud width: %d, width of cloud to filter: %d", 
                    cloud->width, cloud_reduced->width, cloud_for_filter->points.size());
          std::chrono::duration<double> dt = t3 - t2;
          RCLCPP_INFO(this->get_logger(), "Time to copy: %f", dt.count());
        }

        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr cloud_for_filter_pcl2 (new pcl::PCLPointCloud2 ());
        pcl::toPCLPointCloud2(*cloud_for_filter, *cloud_for_filter_pcl2);
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud_for_filter_pcl2);
        sor.setRadiusSearch(search_radius_);
        sor.setMinNeighborsInRadius(min_neighbors_in_radius_);
        sor.filter(*cloud_filtered);
        auto t4 = std::chrono::high_resolution_clock::now();
        if (verbose_)
        {
          std::chrono::duration<double> dt = t4 - t3;
          RCLCPP_INFO(this->get_logger(), "Time to launch radius filter: %f", dt.count());
        }
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pcd (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filtered_pcd);
        for (auto pt : cloud_filtered_pcd->points)
        {
          cloud_resulting->points.push_back(pt);
        }
        cloud_resulting->width = cloud_resulting->points.size();
        pcl::PCLPointCloud2::Ptr cloud_resulting_pcl2 (new pcl::PCLPointCloud2 ());
        pcl::toPCLPointCloud2(*cloud_resulting, *cloud_resulting_pcl2);
        pcl_conversions::fromPCL(*cloud_resulting_pcl2, *cloud_filtered_msg);
      }
      else
      {
        pcl_conversions::fromPCL(*cloud_reduced, *cloud_filtered_msg);
      }
      publisher_->publish(std::move(cloud_filtered_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string input_topic_, output_topic_;
    float resolution_, search_radius_;
    int min_neighbors_in_radius_;
    bool verbose_, filter_outliers_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdSubscriber>());
  rclcpp::shutdown();
}