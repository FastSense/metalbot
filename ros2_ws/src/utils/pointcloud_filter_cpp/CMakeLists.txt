cmake_minimum_required(VERSION 3.5)
project(pointcloud_filter_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
message("Pcl found in ${PCL_LIBRARY_DIRS}")
message("Pcl includes are ${PCL_INCLUDE_DIRS}")
message("Pcl libs are ${PCL_LIBRARIES}")
message("Pcl components are ${PCL_COMPONENTS}")



add_executable(voxel_grid_filter src/voxel_grid_filter.cpp)
target_include_directories(voxel_grid_filter PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})
ament_target_dependencies(voxel_grid_filter rclcpp std_msgs sensor_msgs)

install(TARGETS 
  voxel_grid_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
