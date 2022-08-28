#include "object_follower/object_follower.hpp"

namespace object_follower_ns {

ObjectFollower::ObjectFollower(ros::NodeHandle* nh)
    : image_sub_(
          nh->subscribe("input_image", 1000, &ObjectFollower::imageCb, this)),
      image_pub_(nh->advertise<sensor_msgs::Image>("output_image", 10)),
      cmd_vel_pub(nh->advertise<geometry_msgs::Twist>("cmd_vel", 10)) {}

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
  nh_p_.param("object_follower_node/hsv_lowerb", hsv_lowerb_,
              default_hsv_lowerb_);
  nh_p_.param("object_follower_node/hsv_upperb", hsv_upperb_,
              default_hsv_upperb_);
  for (const auto& v : hsv_lowerb_) {
    ROS_INFO("hsv_lowerb: %i", v);
  }
  for (const auto& v : hsv_upperb_) {
    ROS_INFO("hsv_upperb: %i", v);
  }
}

void ObjectFollower::followTarget(const cv::Point& target,
                                  const cv::Point& current) {
  auto error = target - current;
  auto cmd_vel_msg = geometry_msgs::Twist();
  cmd_vel_msg.linear.x = std::clamp(error.y * 0.005, -1.0, 1.0);
  cmd_vel_msg.angular.z = std::clamp(error.x * 0.005, -0.6, 0.6);
  cmd_vel_pub.publish(cmd_vel_msg);
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

  // Create cv Mat object for HSVimage
  cv::Mat HSVimage;
  // Convert BGR image to HSV
  cv::cvtColor(cv_ptr->image, HSVimage, cv::COLOR_BGR2HSV);

  // Create cv Mat object for image_threshold
  cv::Mat image_threshold;
  // Filter thresholded object with inRange function
  cv::inRange(HSVimage,
              cv::Scalar(hsv_lowerb_[0], hsv_lowerb_[1], hsv_lowerb_[2]),
              cv::Scalar(hsv_upperb_[0], hsv_upperb_[1], hsv_upperb_[2]),
              image_threshold);

  // Draw target marker for following
  cv::Point target(HSVimage.size().width / 2,
                   HSVimage.size().height / 2 + HSVimage.size().height * 0.2);
  cv::drawMarker(cv_ptr->image, target, cv::Scalar(0, 128, 128),
                 cv::MARKER_CROSS);

  // Find thresholded object's contour and centroid
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

    cv::Point object_centroid(m.m10 / m.m00, m.m01 / m.m00);
    cv::circle(cv_ptr->image, object_centroid, 5, cv::Scalar(0, 0, 128), -1);

    this->followTarget(target, object_centroid);
  }

  cv::imshow("threshold_window", threshold_image);
  cv::imshow("centroid_window", cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}
}  // namespace object_follower_ns