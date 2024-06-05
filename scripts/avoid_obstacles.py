#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

class ObstacleAvoider:
    def __init__(self):
        '''Initialises node. Creats publisher, velocity message, subscriber, client and goal objects.
            Also sets shutdown callback function'''
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('obstacle_avoider')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        rospy.spin()

    def linear_ctl_rumba_move(self, ranges):
        '''Simple Rumba ctl method for controlling robots movements from a list of ranges. 
        Assumes: 0th index is directly infront with 360 measurements evenly spread around robot, +ve index direction is same as +ve rotation direction about z axis'''
        # Paramaters for velocity calculation (dependant on robots characteristics) -  should be defined in robot class
        linear_cap = 0.3
        angular_cap = 0.5
        range_lim = 0.5

        # Find average across 10 measurements spanning 10deg in desired directions
        ranges_front = ranges[:5] + ranges[-5:]
        range_front = sum(ranges_front)/len(ranges_front)
        ranges_front_right = ranges[-50:-40]
        range_front_right = sum(ranges_front_right)/len(ranges_front_right)
        ranges_front_left = ranges[40:50]
        range_front_left = sum(ranges_front_left)/len(ranges_front_left)

        # Move straight when no obstacles in front facing limits
        if (range_front > range_lim) and (range_front_right > range_lim) and (range_front_left > range_lim):
            self.vel_msg.linear.x = linear_cap
            self.vel_msg.angular.z = 0.0
            rospy.loginfo("Straight")
        # Turn left: Smaller range_front_right means tighter turn (reversing included)
        elif (range_front > range_lim) and (range_front_right < range_lim) and (range_front_left > range_lim):
            self.vel_msg.linear.x = 2*range_front_right*linear_cap/range_lim - linear_cap
            self.vel_msg.angular.z = angular_cap * (1-range_front_right/range_lim)
            rospy.loginfo("Ctl left")
        # Turn right: Smaller range_front_left means tighter turn (reversing included)
        elif (range_front > range_lim) and (range_front_right > range_lim) and (range_front_left < range_lim):
            self.vel_msg.linear.x = 2*range_front_left*linear_cap/range_lim - linear_cap
            self.vel_msg.angular.z = -(angular_cap * (1-range_front_left/range_lim))
            rospy.loginfo("Ctl right")
        # Hard turn left when obstacles infront (could change this for better coverage)
        else:
            self.vel_msg.linear.x = -0.1
            self.vel_msg.angular.z = angular_cap
            rospy.loginfo("Hard left")
        
        # Log ranges
        rospy.loginfo(f"\nFront: {range_front}\nLleft: {range_front_left}, Right: {range_front_right}")
        # Publish velocity and display commanded velocities
        self.publish_vel(f"linear: {self.vel_msg.linear.x}, angular: {self.vel_msg.angular.z}")


    def scan_callback(self, data):
        '''LaserScan callback function which then calls a movement'''

        # Swap infinity values for the range_max which better suits linear_ctl_rumba_move function
        linear_ctl_ranges = [r if not math.isinf(r) else data.range_max for r in data.ranges]
        self.linear_ctl_rumba_move(linear_ctl_ranges)        


    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo("Publishing velocity: %s\n" % msg_type)
        self.cmd_pub.publish(self.vel_msg)  # Directly publish without the loop


    def clean_shutdown(self):
        '''Shutdown resets the velocity message to a blank Twist object then publishes'''
        self.vel_msg = Twist()
        rospy.loginfo(f"Stopping {self.vel_msg}")
        self.publish_vel("stop")


    def move_to_goal(self, x, y, w):
        '''Moves robot to the desire frame set by x, y, w'''
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
