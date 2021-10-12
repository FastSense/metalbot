cmake_minimum_required(VERSION 3.12)
project(openvdb_test CXX)

message(STATUS "SOME TEXT")
message(STATUS ${CMAKE_MODULE_PATH})


# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")

# find dependencies
find_package(OpenVDB REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)

target_link_libraries(talker OpenVDB::openvdb tbb)

include_directories(
  talker
  ${OpenVDB_INCLUDE_DIR}
)

ament_target_dependencies(talker
 rclcpp
 std_msgs 
 sensor_msgs 
 geometry_msgs
 pcl_conversions
)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
