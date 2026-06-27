cmake_minimum_required(VERSION 3.0.2)
project(chassis_bridge)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/muses.proto)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(chassis_bridge_node src/chassis_bridge_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(chassis_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(chassis_bridge_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Protobuf_LIBRARIES})
