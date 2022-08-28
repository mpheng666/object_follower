#ifndef _OBJECT_FOLLOWER_HPP
#define _OBJECT_FOLLOWER_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

// Include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



namespace object_follower_ns {

struct PID {
  double P{0.0};
  double I{0.0};
  double D{0.0};
};

using Pose = geometry_msgs::Pose;
using Image = sensor_msgs::Image;
class ObjectFollower {
 public:
  ObjectFollower(ros::NodeHandle* nh);
  ~ObjectFollower();
  void start();

 private:
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  ros::Publisher object_pose_pub_;
  ros::Publisher cmd_vel_pub;
  const std::string OPENCV_WINDOW{"Image window"};
  static constexpr double LOOP_RATE{20.0};
  ros::NodeHandle nh_p_;
  PID pid_follow_ {10.0, 0.0, 0.01};

  std::vector<int> default_hsv_lowerb_{0, 0, 0};
  std::vector<int> default_hsv_upperb_{255, 255, 255};
  std::vector<int> hsv_lowerb_{0, 0, 0};
  std::vector<int> hsv_upperb_{0, 0, 0};

  void loadRosParam();
  void imageCb(const Image::ConstPtr& msg);
  void preProcessImage(const cv::Mat& input, cv::Mat& output);
  // void drawBoundaries(cv::Mat& image, const std::string& id,
  //                     const Rectangle& rec);
  Pose detectObjectPose(const Image& image);
  void followTarget(const cv::Point& target, const cv::Point& current);
};

}  // namespace object_follower_ns

#endif