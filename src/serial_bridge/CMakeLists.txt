cmake_minimum_required(VERSION 3.0.2)
project(serial_bridge)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  plant_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    plant_msgs
    roscpp
    std_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_bridge_node src/serial_bridge_node.cpp)
add_dependencies(serial_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_bridge_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
