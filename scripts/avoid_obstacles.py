#!/usr/bin/env python3

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
        vel_cmd = Twist()
        # rospy.loginfo(f"\n\nDATA is: {data}\n\n")
        front_angle_low = 10 * (3.14159 / 180)  # +ve direction from origin
        front_angle_high = 3.14159 - 10 * (3.14159 / 180)  # negative direction from origin
        angle_increment = 0.017501922324299812 # Radians
        ranges_infront = []
        for i, range in enumerate(data.ranges):
            angle = i * angle_increment
            if angle > front_angle_high or angle < front_angle_low:
                ranges_infront.append(angle)
        rospy.loginfo(f"Ranges infront: {ranges_infront} has length {len(ranges_infront)} out of total {len(data.ranges)}")
        range_front = sum(ranges_infront) / len(ranges_infront) 

        rospy.loginfo(f"Low average : {range_front} lowest: {data.range_min}\n which is interesting\n\n")
        if (range_front) < 0.4:
            vel_cmd.angular.z = 0.5  # Turn
            vel_cmd.linear.x = 0.0
        else:
            vel_cmd.linear.x = 0.2  # Move forward
            vel_cmd.angular.z = 0.0
        self.cmd_pub.publish(vel_cmd)

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
