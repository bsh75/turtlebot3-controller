#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

class RobotParameters:
    '''Should really be determined from launch in relation to the robot type'''
    def __init__(self):
        self.linear_v_lim = 0.3
        self.angular_v_lim = 0.5
        self.range_lim = 0.5

class ObstacleAvoider:
    def __init__(self):
        '''Initialises node. Creats publisher, velocity message, subscriber, client and goal objects.
            Also sets shutdown callback function'''
        self.phys_params = RobotParameters()
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('obstacle_avoider')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        rospy.spin()

    def linear_ctl_rumba_move(self, data):
        '''Simple Rumba ctl method for controlling robots movements from a list of ranges. 
        Assumes: 0th index is directly infront with 360 measurements evenly spread around robot, +ve index direction is same as +ve rotation direction about z axis'''
        # Find average across 10 measurements spanning 10deg in desired directions
        # Swap infinity values for the range_max which better suits linear_ctl_rumba_move function
        ranges = [r if not math.isinf(r) else data.range_max for r in data.ranges]
        ranges_front = ranges[:5] + ranges[-5:]
        range_front = sum(ranges_front)/len(ranges_front)
        ranges_front_right = ranges[-50:-40]
        range_front_right = sum(ranges_front_right)/len(ranges_front_right)
        ranges_front_left = ranges[40:50]
        range_front_left = sum(ranges_front_left)/len(ranges_front_left)

        # Move straight when no obstacles in front facing limits
        if (range_front > self.phys_params.range_lim) and (range_front_right > self.phys_params.range_lim) and (range_front_left > self.phys_params.range_lim):
            self.vel_msg.linear.x = self.phys_params.linear_v_lim
            self.vel_msg.angular.z = 0.0
            msg_type = "Straight"
        # Turn left: Smaller range_front_right means tighter turn (reversing included)
        elif (range_front > self.phys_params.range_lim) and (range_front_right < self.phys_params.range_lim) and (range_front_left > self.phys_params.range_lim):
            self.vel_msg.linear.x = 2*range_front_right*self.phys_params.linear_v_lim/self.phys_params.range_lim - self.phys_params.linear_v_lim
            self.vel_msg.angular.z = self.phys_params.angular_v_lim * (1-range_front_right/self.phys_params.range_lim)
            msg_type = "Ctl left"
        # Turn right: Smaller range_front_left means tighter turn (reversing included)
        elif (range_front > self.phys_params.range_lim) and (range_front_right > self.phys_params.range_lim) and (range_front_left < self.phys_params.range_lim):
            self.vel_msg.linear.x = 2*range_front_left*self.phys_params.linear_v_lim/self.phys_params.range_lim - self.phys_params.linear_v_lim
            self.vel_msg.angular.z = -(self.phys_params.angular_v_lim * (1-range_front_left/self.phys_params.range_lim))
            msg_type = "Ctl right"
        # Hard turn left when obstacles infront (could change this for better coverage)
        else:
            self.vel_msg.linear.x = -0.1
            self.vel_msg.angular.z = self.phys_params.angular_v_lim
            msg_type = "Hard left"
        try:
            # Log ranges
            rospy.loginfo(f"\nFront: {range_front}\nLleft: {range_front_left}, Right: {range_front_right}")
            # Publish velocity and display commanded velocities
            self.publish_vel(msg_type)
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish velocity command: %s", e)


    def scan_callback(self, data):
        '''LaserScan callback function which then calls a movement'''
        # pass
        self.linear_ctl_rumba_move(data)        


    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo(f"{msg_type} movement: linear: {self.vel_msg.linear.x}, angular: {self.vel_msg.angular.z}")
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
