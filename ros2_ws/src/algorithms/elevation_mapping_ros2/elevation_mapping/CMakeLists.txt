cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping)

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

set(
  PACKAGE_DEPENDENCIES 
    rclcpp
    sensor_msgs
    std_srvs
    eigen_conversions
    Eigen3
    kindr
    grid_map_core
    grid_map_ros
    grid_map_msgs
    message_filters
    PCL
)

find_package(ament_cmake REQUIRED)
foreach (PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
)

##############
# Executable #
##############

add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
)

add_executable(elevation_mapping_node
  src/elevation_mapping_node.cpp
)

target_include_directories(${PROJECT_NAME}_library PRIVATE include)
foreach (pkg ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_library ${${pkg}_LIBRARIES})
endforeach()
target_link_libraries(${PROJECT_NAME}_library Boost::thread)
target_link_libraries(elevation_mapping_node ${PROJECT_NAME}_library)

ament_target_dependencies(elevation_mapping_node ${PACKAGE_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME}_library ${PACKAGE_DEPENDENCIES})
ament_export_libraries(${PROJECT_NAME}_library)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

#############
## Install ##
#############

install(
  DIRECTORY 
    include
  DESTINATION include
)

install(
  TARGETS 
    elevation_mapping_node
    ${PROJECT_NAME}_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()