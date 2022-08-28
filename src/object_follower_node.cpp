#include "object_follower/object_follower.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "object_follower_node");
  ros::NodeHandle nh;
  object_follower_ns::ObjectFollower object_follower(&nh);
  object_follower.start();
  
  // ros::spin();
  return 0; }