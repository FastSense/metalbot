cmake_minimum_required(VERSION 3.12)
project(openvdb_test CXX)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")

# find dependencies
find_package(OpenVDB REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(point_cloud_publisher src/point_cloud_publisher.cpp)

target_link_libraries(point_cloud_publisher OpenVDB::openvdb tbb)

include_directories(
  point_cloud_publisher
  include
  ${OpenVDB_INCLUDE_DIR}
)

ament_target_dependencies(point_cloud_publisher
 rclcpp
 std_msgs 
 sensor_msgs 
 geometry_msgs
 pcl_conversions
)

install(TARGETS
  point_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
