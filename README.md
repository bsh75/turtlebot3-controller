# TurtleBot3 Controller for Obstacle Avoidance

## Overview
This project demonstrates a simple obstacle avoidance behavior for TurtleBot3 using ROS and Gazebo. The robot moves forward and turns when an obstacle is detected.

## Requirements
- ROS Melodic or Noetic
- TurtleBot3 Packages:
    - List packages used?
- Gazebo
- catkin workspace setup if not:
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src

## Add to .bashrc file (or run each time a new terminal is opened)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

## Initialisation
Create catkin package with required dependancies:
catkin_create_pkg turtlebot3_controller rospy std_msgs geometry_msgs sensor_msgs nav_msgs move_base_msgs tf
catkin_make

## Running
Create environment variable for Turtlebot3 model eg waffle:
export TURTLEBOT3_MODEL=waffle

Launch (cmd will launch both gazebo and control node):
roslaunch turtlebot3_controller avoid_obstacles.launch

