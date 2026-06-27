cmake_minimum_required(VERSION 3.0.2)
project(plant_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  PlantDetection.msg
  PlantDetectionArray.msg
  PlantTrack.msg
  PlantPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
