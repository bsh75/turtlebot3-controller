#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
import subprocess

class ObstacleAvoider:
    def __init__(self):
        '''Initializes node and sets shutdown callback function'''
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('explore_map')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        # Start explore_lite node
        self.explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

        rospy.sleep(5)  # Give some time for the explore_lite node to initialize

        self.exploration_complete = False

        # Subscribe to the move_base status to monitor exploration progress
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)

        rospy.spin()

    def move_base_status_callback(self, data):
        '''Monitor move_base status to determine if exploration is complete'''
        # Check if all goals are reached
        if not data.status_list:  # No goals in the list
            self.exploration_complete = True
        else:
            # Check if all goals have a status of SUCCEEDED
            self.exploration_complete = all(status.status == 3 for status in data.status_list)  # SUCCEEDED status is 3

        if self.exploration_complete:
            self.clean_shutdown()

    def publish_vel(self, msg_type="move"):
        '''Publish the Twist message'''
        rospy.loginfo(f"{msg_type} movement: linear: {self.vel_msg.linear.x}, angular: {self.vel_msg.angular.z}")
        self.cmd_pub.publish(self.vel_msg)  # Directly publish without the loop

    def clean_shutdown(self):
        '''Shutdown resets the velocity message to a blank Twist object then publishes and saves the generated map'''
        self.vel_msg = Twist()
        rospy.loginfo(f"Stopping {self.vel_msg}")
        self.publish_vel("stop")

        # Save the map
        rospy.loginfo("Saving the map...")
        map_saver_process = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "/home/bsh75/maps/my_map"])        
        map_saver_process.wait()
        rospy.loginfo("Map saved.")

        # Shutdown the explore_lite node
        rospy.loginfo("Terminating explore_lite process...")
        self.explore_process.terminate()
        self.explore_process.wait()

        rospy.signal_shutdown("Exploration complete and map saved")

if __name__ == '__main__':
    try:
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass
