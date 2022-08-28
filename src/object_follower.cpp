#include "object_follower/object_follower.hpp"

namespace object_follower_ns {

ObjectFollower::ObjectFollower(ros::NodeHandle* nh)
    : image_sub_(nh->subscribe("input_image", 1000,
                               &ObjectFollower::imageCb, this)),
      image_pub_(nh->advertise<sensor_msgs::Image>("output_image", 10)) {}

ObjectFollower::~ObjectFollower() {}

void ObjectFollower::start() {
  ros::Rate rate(LOOP_RATE);

  
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}

void ObjectFollower::imageCb(const Image::ConstPtr& msg) {
  // std_msgs::Header msg_header = msg->header;
  // std::string frame_id = msg->header.frame_id;
  // ROS_INFO_STREAM("New Image from " << frame_id);

  // cv_bridge::CvImagePtr cv_ptr;
  // try {
  //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // } catch (cv_bridge::Exception& e) {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // Output modified video stream
  // image_pub_.publish(cv_ptr->toImageMsg());
}
}  // namespace object_follower_ns