cmake_minimum_required(VERSION 3.5)
project(dogm_plugin LANGUAGES CXX CUDA)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV 4.5.2 EXACT REQUIRED)
find_package(dogm REQUIRED)
find_package(dogm_msgs REQUIRED)

# build
add_library(${PROJECT_NAME} SHARED src/dogm_layer.cu)
include_directories(include ${OpenCV_INCLUDE_DIRS} ${DOGM_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${DOGM_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp nav2_costmap_2d pluginlib dogm_msgs)

#install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d dogm_plugin.xml)
ament_package()
