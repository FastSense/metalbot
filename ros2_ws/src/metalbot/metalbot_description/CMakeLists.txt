cmake_minimum_required(VERSION 3.5)
project(metalbot_description)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(DIRECTORY urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
