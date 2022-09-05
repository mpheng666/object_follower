cmake_minimum_required(VERSION 3.0.2)
project(pcl_preprocess)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
)

include_directories(
  include(${catkin_INCLUDE_DIRS})
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros

  DEPENDS
)

set(SOURCES src/pc_to_img_node.cpp)

add_executable(pc_to_img_node ${SOURCES})

target_link_libraries(pc_to_img_node
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
)
