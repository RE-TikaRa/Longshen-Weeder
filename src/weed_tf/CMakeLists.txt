cmake_minimum_required(VERSION 3.0.2)
project(weed_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(weed_tf_node src/weed_tf_node.cpp)
target_link_libraries(weed_tf_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(weed_tf_test
    test/tf.test
    test/tf_test.cpp)
  target_link_libraries(weed_tf_test ${catkin_LIBRARIES})
endif()
