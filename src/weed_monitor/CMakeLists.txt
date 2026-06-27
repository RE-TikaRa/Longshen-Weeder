cmake_minimum_required(VERSION 3.0.2)
project(weed_monitor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  plant_msgs
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    plant_msgs
    roscpp
    std_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(weed_monitor_node src/weed_monitor_node.cpp)
add_dependencies(weed_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(weed_monitor_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(weed_monitor_test
    test/monitor.test
    test/monitor_test.cpp)
  add_dependencies(weed_monitor_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(weed_monitor_test ${catkin_LIBRARIES})
endif()
