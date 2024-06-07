#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import actionlib
import numpy as np


class RobotParameters:
    '''Should really be determined from launch in relation to the robot type'''
    def __init__(self):
        self.linear_v_lim = 0.3
        self.angular_v_lim = 0.5
        self.range_lim = 0.5

class MapData(OccupancyGrid):  # Inherit from OccupancyGrid
    def __init__(self, *args, **kwargs):
        super(MapData, self).__init__(*args, **kwargs)  # Call the parent constructor
        self.covered_map = np.zeros_like(self.data, dtype=bool)  # Initialize covered_map

    def update_map(self, map_msg: OccupancyGrid):
        self.header = map_msg.header
        self.info = map_msg.info
        self.data = np.array(map_msg.data, dtype=np.int8).reshape((map_msg.info.height, map_msg.info.width))
        # map_msg.data
        
        # Resize covered_map if necessary
        if self.covered_map.shape != (self.info.height, self.info.width):
            self.covered_map = np.zeros_like(self.data, dtype=bool)

class CoverMap:
    def __init__(self):
        '''Initialises node. Creats publisher, velocity message, subscriber, client and goal objects.
            Also sets shutdown callback function'''
        self.phys_params = RobotParameters()
        rospy.on_shutdown(self.clean_shutdown)
        rospy.init_node('cover_map')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.map_data = MapData()  # Create a MapData object
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_data.update_map)

        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()

        if not self.client.wait_for_server(rospy.Duration(10)):
            rospy.logwarn("move_base action server is not available.")
        else:
            rospy.loginfo("Connected to move_base action server.")
        rospy.loginfo(f"\n\n\n--------------------------\n{self.goal}\n-------------------------\n\n\n")
        rospy.spin()


    def scan_callback(self, data: LaserScan):
        '''LaserScan callback function which then calls a movement'''
        pass
    

    def amcl_callback(self, data: PoseWithCovarianceStamped):
        """Callback for AMCL pose updates, triggers navigation."""
        rospy.logerr("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
        current_pose = data.pose.pose
        rospy.loginfo("\n\nCurrent Pose:\n%s", current_pose)
        rospy.logwarn("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
        self.goal.target_pose.pose.position.x = 1.0
        self.goal.target_pose.pose.position.y = 1.0
        self.goal.target_pose.pose.orientation.w = 1.0 
        # goal_pose = self.calculate_next_goal(current_pose)  # Calculate goal based on pose and map
        rospy.loginfo(f"GOAL POSE: {self.goal.target_pose.pose}\n\n")
        rospy.logerr("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
        
        self.move_to_goal()
        # if goal_pose:
        #     self.move_to_goal(goal_pose)


    def calculate_next_goal(self, current_pose):
        """Calculates the next goal pose using a boustrophedon pattern."""

        # Parameters (adjust as needed)
        cell_size = 0.3  # Size of each cell in meters
        coverage_threshold = 0.9  # Percentage of map considered covered

        # Convert current pose to grid coordinates
        current_x = int((current_pose.position.x - self.map_data.info.origin.position.x) / cell_size)
        current_y = int((current_pose.position.y - self.map_data.info.origin.position.y) / cell_size)

        # Initialize direction (start by moving horizontally)
        direction = "horizontal"

        # Iterate over the map grid
        for i in range(self.map_data.info.height):
            for j in range(self.map_data.info.width):
                # Check if the cell is free and uncovered
                if self.map_data.data[i][j] == 0 and not self.map_data.covered_map[i][j]:
                    # Found an uncovered cell!
                    goal_x = j * cell_size + self.map_data.info.origin.position.x + (cell_size / 2)
                    goal_y = i * cell_size + self.map_data.info.origin.position.y + (cell_size / 2)
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = "map"
                    goal_pose.pose.position.x = goal_x
                    goal_pose.pose.position.y = goal_y
                    goal_pose.pose.orientation.w = 1.0  # Assuming no rotation needed

                    # Update the covered_map (mark cell as covered)
                    self.map_data.covered_map[i][j] = True

                    # Check if coverage threshold is reached
                    # if self.map_data.calculate_coverage() >= coverage_threshold:
                    #     self.exploration_complete = True

                    return goal_pose

            # Switch direction after each row/column
            direction = "vertical" if direction == "horizontal" else "horizontal"

        return None  # No more uncovered cells found

    def move_to_goal(self):
        """Sends the goal pose to the move_base action server."""

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal)

        # Wait for the result and log it (add error handling if needed)
        self.client.wait_for_result()
        result = self.client.get_result()
        # rospy.loginfo("Goal result: {}".format(result))


    def publish_vel(self, msg_type="move"):
        ''' Publish the Twist message '''
        rospy.loginfo(f"{msg_type} movement: linear: {self.vel_msg.linear.x}, angular: {self.vel_msg.angular.z}")
        self.cmd_pub.publish(self.vel_msg)  # Directly publish without the loop


    def clean_shutdown(self):
        '''Shutdown resets the velocity message to a blank Twist object then publishes'''
        self.vel_msg = Twist()
        rospy.loginfo(f"Stopping {self.vel_msg}")
        self.publish_vel("stop")


if __name__ == '__main__':
    try:
        CoverMap()

    except rospy.ROSInterruptException:
        pass
