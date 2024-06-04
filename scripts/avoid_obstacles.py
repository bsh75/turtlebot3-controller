#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()

    def scan_callback(self, data):
        twist = Twist()
        if min(data.ranges) < 0.5:
            twist.angular.z = 0.5  # Turn
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

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
        avoider = ObstacleAvoider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
