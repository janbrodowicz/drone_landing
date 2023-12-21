#!/bin/bash

echo "Setting enviroment variables to run simulation..."

source ~/catkin_ws/devel/setup.bash
source ~/PX4/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4/PX4-Autopilot ~/PX4/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/ros_landing/plugins/moving_landing_pad/build:$GAZEBO_PLUGIN_PATH