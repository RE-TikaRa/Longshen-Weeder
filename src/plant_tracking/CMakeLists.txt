cmake_minimum_required(VERSION 3.0.2)
project(plant_tracking)

find_package(catkin REQUIRED COMPONENTS
  plant_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    plant_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(plant_tracking_node src/plant_tracking_node.cpp)
add_dependencies(plant_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(plant_tracking_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(plant_tracking_test
    test/tracking.test
    test/tracking_test.cpp)
  add_dependencies(plant_tracking_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(plant_tracking_test ${catkin_LIBRARIES})
endif()
