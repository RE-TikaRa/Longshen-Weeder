cmake_minimum_required(VERSION 3.0.2)
project(plant_depth)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  plant_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    plant_msgs
    roscpp
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(plant_depth_node src/plant_depth_node.cpp)
add_dependencies(plant_depth_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(plant_depth_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(plant_depth_test
    test/depth.test
    test/depth_test.cpp)
  add_dependencies(plant_depth_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(plant_depth_test ${catkin_LIBRARIES})
endif()
