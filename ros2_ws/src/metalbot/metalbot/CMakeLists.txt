cmake_minimum_required(VERSION 3.5)
project(metalbot)

# find dependencies
find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/publish_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
