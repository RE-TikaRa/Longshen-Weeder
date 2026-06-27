cmake_minimum_required(VERSION 3.0.2)
project(weed_decision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  plant_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    plant_msgs
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(weed_decision_node src/weed_decision_node.cpp)
add_dependencies(weed_decision_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(weed_decision_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(weed_decision_test
    test/decision.test
    test/decision_test.cpp)
  add_dependencies(weed_decision_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(weed_decision_test ${catkin_LIBRARIES})
endif()
