# TurtleBot3 Controller for Obstacle Avoidance

## Overview
This project demonstrates a simple obstacle avoidance behavior for TurtleBot3 using ROS and Gazebo. The robot moves forward and turns when an obstacle is detected.

## Requirements
- ROS Melodic or Noetic
- TurtleBot3 Packages
- Gazebo

## Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg turtlebot3_controller rospy std_msgs geometry_msgs sensor_msgs nav_msgs move_base_msgs tf
cd ~/catkin_ws
catkin_make
