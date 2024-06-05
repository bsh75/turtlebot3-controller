#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

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
        data.ranges = [r if not math.isinf(r) else data.range_max for r in data.ranges]
        range_front = data.ranges[0]
        range_back = data.ranges[180]
        range_front_right = data.ranges[-45]
        range_front_left = data.ranges[45]
        range_right = data.ranges[-90]
        range_left = data.ranges[90]
        rospy.loginfo(f"\nInfront: {range_front}, Behind: {range_back}\nFrontleft: {range_front_left}, Frontright: {range_front_right}\nRight: {range_right}, Left: {range_left}")
        linear_scalar = 0.01
        angular_scalar = 0.1
        
        self.vel_msg.linear.x += linear_scalar*(range_front-range_back)
        if self.vel_msg.linear.x > 0.3: 
            self.vel_msg.linear.x = 0.3
        elif self.vel_msg.linear.x < -0.3:
            self.vel_msg.linear.x = -0.3
        # self.vel_msg.linear += 0.5*linear_scalar*(range_front_right-range_back)

        self.vel_msg.angular.z += angular_scalar*(range_left - range_right)
        if self.vel_msg.angular.z > 0.3: 
            self.vel_msg.angular.z = 0.3
        elif self.vel_msg.angular.z < -0.3:
            self.vel_msg.angular.z = -0.3

        self.publish_vel(f"linear: {self.vel_msg.linear}, angular: {self.vel_msg.angular}")


    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo("Publishing velocity: %s\n" % msg_type)
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
