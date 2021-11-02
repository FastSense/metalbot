cmake_minimum_required(VERSION 3.5)
project(metalbot_gazebo)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/spawn_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
