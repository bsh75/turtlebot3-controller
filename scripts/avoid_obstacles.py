#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time

class ObstacleAvoider:
    def __init__(self):
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('obstacle_avoider')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        rospy.spin()

    def scan_callback(self, data):
        # rospy.loginfo(f"\n\nDATA is: {data}\n\n")
        range_front = data.ranges[0]
        rospy.loginfo(f"Range infront: {range_front} lowest: {data.range_min}\n which is interesting\n\n")
        if (range_front) < 0.4:
            self.vel_msg.angular.z = 0.5  # Turn
            self.vel_msg.linear.x = 0.0
            self.publish_vel("turning")
        else:
            self.vel_msg.linear.x = 0.2  # Move forward
            self.vel_msg.angular.z = 0.0
            self.publish_vel("fowards")


    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo("Publishing %s message..." % msg_type)
        self.cmd_pub.publish(self.vel_msg)  # Directly publish without the loop


    def clean_shutdown(self):
        self.vel_msg = Twist()
        rospy.loginfo(f"Stopping {self.vel_msg}")
        self.publish_vel("stop")

    def move_to_goal(self, x, y, w):
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = w
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return self.client.get_result()

if __name__ == '__main__':
    try:
        ObstacleAvoider()

    except rospy.ROSInterruptException:
        pass
