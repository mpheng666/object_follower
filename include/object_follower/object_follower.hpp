#ifndef _OBJECT_FOLLOWER_HPP
#define _OBJECT_FOLLOWER_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



namespace object_follower_ns {
struct Point {
  double x{0.0};
  double y{0.0};
};

struct Rectangle {
  Point centre;
  double length{0.0};
  double width{0.0};
};

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
  cv::Mat image_;
  const std::string OPENCV_WINDOW{"Image window"};
  static constexpr double LOOP_RATE{20.0};

  void imageCb(const Image::ConstPtr& msg);
  void preProcessImage(const cv::Mat& input, cv::Mat& output);
  void drawBoundaries(cv::Mat& image, const std::string& id,
                      const Rectangle& rec);
  Pose detectObjectPose(const Image& image);
  bool followTarget(const Pose& pose, const PID& pid);
};

}  // namespace object_follower_ns

#endif