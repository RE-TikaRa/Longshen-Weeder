cmake_minimum_required(VERSION 3.0.2)
project(weed_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
