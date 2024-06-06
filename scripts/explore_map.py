#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import subprocess

class ObstacleAvoider:
    def __init__(self):
        '''Initialises node. Also sets shutdown callback function'''
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('explore_map')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        # Start explore_lite node
        self.explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

        rospy.sleep(5)  # Give some time for the explore_lite node to initialize

        self.exploration_complete = False

        rospy.spin()

    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo(f"{msg_type} movement: linear: {self.vel_msg.linear.x}, angular: {self.vel_msg.angular.z}")
        self.cmd_pub.publish(self.vel_msg)  # Directly publish without the loop

    def clean_shutdown(self):
        '''Shutdown resets the velocity message to a blank Twist object then publishes
        should also save the generated map (LATER)'''
        self.vel_msg = Twist()
        rospy.loginfo(f"Stopping {self.vel_msg}")
        self.publish_vel("stop")
        # Save the map:

        # Shutdown the explore_lite node
        self.explore_process.terminate()


if __name__ == '__main__':
    try:
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass
