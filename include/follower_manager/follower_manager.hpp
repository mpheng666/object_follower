#ifndef _OBJECT_FOLLOWER_FOLLOWER_MANAGER_HPP_
#define _OBJECT_FOLLOWER_FOLLOWER_MANAGER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace follower_manager_ns {
enum class DetectionState { START_DETECTION, STOP_DETECTION};
enum class FollowerState { START_FOLLOWER, STOP_FOLLWER };

struct ManagerState {
  DetectionState detection;
  FollowerState follower;
};

class FollowerManager {
 public:
  FollowerManager();
  ~FollowerManager();
  bool initState();
  ManagerState getState() const;

 private:
  ManagerState manager_state_;
  bool setState(const ManagerState& state);

  ros::Subscriber joy_sub_;
};

}  // namespace follower_manager_ns
#endif