cmake_minimum_required(VERSION 3.0.2)
project(object_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET)

if(NOT OpenCV_FOUND)
message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
COMPONENTS
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    cv_bridge
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

add_executable(color_thresholding_node src/utility/color_thresholding.cpp)
add_executable(${PROJECT_NAME}_node src/object_follower_node.cpp src/object_follower.cpp)

target_link_libraries(color_thresholding_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


