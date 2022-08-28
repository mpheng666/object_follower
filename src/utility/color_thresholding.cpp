#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int max_value_H = 360 / 2;
const int max_value = 255;
const std::string window_capture_name = "Video Capture";
const std::string window_detection_name = "Object Detection";
int low_H = 0;
int low_S = 0;
int low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
cv::Mat frame, frame_HSV, frame_threshold;

static void on_low_H_thresh_trackbar(int, void*) {
  low_H = std::min(high_H - 1, low_H);
  cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void*) {
  high_H = std::max(high_H, low_H + 1);
  cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void*) {
  low_S = std::min(high_S - 1, low_S);
  cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void*) {
  high_S = std::max(high_S, low_S + 1);
  cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void*) {
  low_V = std::min(high_V - 1, low_V);
  cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void*) {
  high_V = std::max(high_V, low_V + 1);
  cv::setTrackbarPos("High V", window_detection_name, high_V);
}

void imageCb(const sensor_msgs::Image::ConstPtr& msg) {
  std_msgs::Header msg_header = msg->header;
  std::string frame_id = msg->header.frame_id;
  ROS_INFO_STREAM("New Image from " << frame_id);

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert from BGR to HSV colorspace
  cv::cvtColor(cv_ptr->image, frame_HSV, cv::COLOR_BGR2HSV);
  // Detect the object based on HSV Range Values
  cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V),
              cv::Scalar(high_H, high_S, high_V), frame_threshold);
  // Show the frames
  cv::imshow(window_capture_name, cv_ptr->image);
  cv::imshow(window_detection_name, frame_threshold);
  cv::waitKey(3);
}

int main(int argv, char** argc) {
  ros::init(argv, argc, "color_threshold_node");
  ros::NodeHandle nh;
  ros::Subscriber image_sub = nh.subscribe("threshold_image_input", 1000, imageCb);

  cv::namedWindow(window_capture_name);
  cv::namedWindow(window_detection_name);
  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H,
                     on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H,
                     on_high_H_thresh_trackbar);
  cv::createTrackbar("Low S", window_detection_name, &low_S, max_value,
                     on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", window_detection_name, &high_S, max_value,
                     on_high_S_thresh_trackbar);
  cv::createTrackbar("Low V", window_detection_name, &low_V, max_value,
                     on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", window_detection_name, &high_V, max_value,
                     on_high_V_thresh_trackbar);
  ros::spin();
  return 0;
}