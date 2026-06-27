cmake_minimum_required(VERSION 3.0.2)
project(plant_detection)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  plant_msgs
  roscpp
  roslib
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include /usr/include/aarch64-linux-gnu)
find_library(NVINFER_LIB nvinfer
  HINTS /usr/lib /usr/lib/aarch64-linux-gnu)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin
  HINTS /usr/lib /usr/lib/aarch64-linux-gnu)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    plant_msgs
    roscpp
    roslib
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

add_executable(plant_detection_node src/plant_detection_node.cpp)
add_dependencies(plant_detection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(plant_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
)
