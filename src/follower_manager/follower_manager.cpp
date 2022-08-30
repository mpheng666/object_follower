#include "follower_manager/follower_manager.hpp"

namespace follower_manager_ns {
FollowerManager::FollowerManager() {}

FollowerManager::~FollowerManager() {}

bool FollowerManager::initState() {
  manager_state_.detection = DetectionState::START_DETECTION;
  manager_state_.follower = FollowerState::STOP_FOLLWER;
  return true;
}

ManagerState FollowerManager::getState() const { return manager_state_; }

bool FollowerManager::setState(const ManagerState& state) {
  manager_state_ = state;
  return true;
}
}  // namespace follower_manager_ns