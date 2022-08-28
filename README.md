# object_follower

## Purpose

This module outputs the cmd_vel based on the object position detected on the camera

## How to install and use

1. 'cd && mkdir -p robot_ws/src && cd src'
2. 'gitclone <https://github.com/mpheng666/object_follower.git>'
3. 'cd ~/robot_ws && catkin build'
4. 'source devel/setup.bash'
5. 'roslaunch object_follower start.launch'
6. roslaunch insert your fav robot_bringup in gazebo
7. Insert the green_ball object from models into your gazebo world
8. Drag the ball around and watch your robot chasing it

## Limitation

1. Current implementation is based on mono colour thresholding
2. Only single object can be detected
3. Searching is implemeted with simple spot turn

## Future work

1. Add CNN object classifier
2. Add multiple object detection
3. Improve searching strategy