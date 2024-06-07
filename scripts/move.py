#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
import math


class GoalSetter:
    def __init__(self):
        rospy.init_node('goal_setter_node')

        # Initialize action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        # Subscribe to laser scan data
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.laser_data = LaserScan

        # Create a timer to trigger the scan_callback every 3 seconds
        self.timer = rospy.Timer(rospy.Duration(3), self.calculate_goal)


    def scan_callback(self, data):
        # Process LaserScan data and set goal for move_base if no goal is active
        self.laser_data = data


    def calculate_goal(self, event):
        max_range = max(self.laser_data.ranges)
        max_index = self.laser_data.ranges.index(max_range)
        angle_increment = self.laser_data.angle_increment
        # Calculate goal position based on the maximum scanner reading
        goal_x = 1 * math.cos(angle_increment * max_index)
        goal_y = 1 * math.sin(angle_increment * max_index)
        rospy.loginfo(f"Sending x: {goal_x}, y: {goal_y}")
        self.send_goal(goal_x, goal_y)


    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
        rospy.loginfo(f"New goal: {goal} sent to move_base")


    def goal_done_callback(self, state, result):
        # Callback function to be executed when the goal is done (reached or aborted)
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached. Setting new goal...")
        else:
            rospy.logwarn("Goal aborted")

if __name__ == '__main__':
    try:
        goal_setter = GoalSetter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
