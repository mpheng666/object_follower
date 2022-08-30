#include "object_follower/object_follower.hpp"

namespace object_follower_ns {

ObjectFollower::ObjectFollower(ros::NodeHandle* nh)
    : image_sub_(
          nh->subscribe("input_image", 1000, &ObjectFollower::imageCb, this)),
      image_pub_(nh->advertise<sensor_msgs::Image>("output_image", 10)),
      cmd_vel_pub(nh->advertise<geometry_msgs::Twist>("cmd_vel", 10)),
      image_depth_sub_(nh->subscribe("input_image", 1000,
                                     &ObjectFollower::imageDepthCb, this)) {}

ObjectFollower::~ObjectFollower() {}

void ObjectFollower::start() {
  ros::Rate rate(LOOP_RATE);
  this->loadRosParam();
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}

void ObjectFollower::loadRosParam() {
  nh_p_.param("threshold_value/hsv_lowerb", hsv_lowerb_, default_hsv_lowerb_);
  nh_p_.param("threshold_value/hsv_upperb", hsv_upperb_, default_hsv_upperb_);
  nh_p_.param("follower_controller/P", pid_follow_.P, 1.0);
  nh_p_.param("follower_controller/I", pid_follow_.I, 0.0);
  nh_p_.param("follower_controller/D", pid_follow_.D, 0.0);
  for (const auto& v : hsv_lowerb_) {
    ROS_INFO("hsv_lowerb: %i", v);
  }
  for (const auto& v : hsv_upperb_) {
    ROS_INFO("hsv_upperb: %i", v);
  }
}

cv::Point ObjectFollower::drawTarget(cv_bridge::CvImagePtr& image_ptr,
                                     const double percent_horizontal,
                                     const double percent_vertical,
                                     const double offset_horizontal,
                                     const double offset_vertical) {
  // Draw target marker for following
  cv::Point target(image_ptr->image.size().width * percent_horizontal +
                       image_ptr->image.size().width * offset_horizontal,
                   image_ptr->image.size().height * percent_vertical +
                       image_ptr->image.size().height * offset_vertical);
  cv::drawMarker(image_ptr->image, target, cv::Scalar(0, 128, 128),
                 cv::MARKER_CROSS);
  return target;
}

void ObjectFollower::followTarget(const cv::Point& target,
                                  const cv::Point& current) {
  auto error = target - current;
  auto cmd_vel_msg = geometry_msgs::Twist();
  cmd_vel_msg.linear.x = std::clamp(error.y * 0.005 * pid_follow_.P, -1.0, 1.0);
  cmd_vel_msg.angular.z =
      std::clamp(error.x * 0.005 * pid_follow_.P, -0.6, 0.6);
  cmd_vel_pub.publish(cmd_vel_msg);
}

void ObjectFollower::searchTarget(const double angular_z_vel) {
  auto cmd_vel_msg = geometry_msgs::Twist();
  cmd_vel_msg.angular.z = angular_z_vel;
  cmd_vel_pub.publish(cmd_vel_msg);
}

bool ObjectFollower::detectObject(cv_bridge::CvImagePtr& image_ptr,
                                  cv::Point& object_centroid) {
  // Convert BGR image to HSV
  cv::Mat HSVimage;
  cv::cvtColor(image_ptr->image, HSVimage, cv::COLOR_BGR2HSV);

  // Filter thresholded object with inRange function
  cv::Mat image_threshold;
  cv::inRange(HSVimage,
              cv::Scalar(hsv_lowerb_[0], hsv_lowerb_[1], hsv_lowerb_[2]),
              cv::Scalar(hsv_upperb_[0], hsv_upperb_[1], hsv_upperb_[2]),
              image_threshold);

  // Find thresholded object's contour
  cv::Mat threshold_image = cv::Mat::zeros(image_threshold.size(), CV_8UC3);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image_threshold, contours, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);

  if (contours.size() > 0) {
    cv::Moments m;
    for (size_t i = 0; i < contours.size(); i++) {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      drawContours(threshold_image, contours, (int)i, color, 2, cv::LINE_8);
      m = cv::moments(contours[i]);
    }

    // Find contour's centroid
    cv::Point detected_centroid(m.m10 / m.m00, m.m01 / m.m00);
    object_centroid = detected_centroid;
    cv::circle(image_ptr->image, object_centroid, 5, cv::Scalar(0, 0, 128), -1);
    return true;
  } else {
    return false;
  }
}

void ObjectFollower::imageCb(const Image::ConstPtr& msg) {
  // Convert ROS image to cv format using cv_bridge
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  auto target = this->drawTarget(cv_ptr, 0.5, 0.5, 0.0, 0.2);
  cv::Point object_centroid;

  if (this->detectObject(cv_ptr, object_centroid)) {
    this->followTarget(target, object_centroid);
  } else {
    const double angular_z_vel{0.3};
    this->searchTarget(angular_z_vel);
  }

  cv::imshow("centroid_window", cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

void ObjectFollower::imageDepthCb(const PC2::ConstPtr& msg) {
  
    }

}  // namespace object_follower_ns