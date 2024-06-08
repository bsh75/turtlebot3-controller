#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import subprocess
import os

class MapSubscriber:
    def __init__(self):
        rospy.init_node('map_subscriber', anonymous=True)
        self.map_received = False

        # Subscribe to the map topic
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        # Callback function to handle received map
        if not self.map_received:
            rospy.loginfo("Map received!")
            self.map_received = True
            # Launch move_base once map is received
            self.launch_move_base(msg)

    def launch_move_base(self, map_msg):
        # Create a launch file with the received map as parameter
        launch_content = """<launch>
    <!-- Launch move_base with the received map -->
    <node pkg="move_base" type="move_base" name="move_base" output="screen">
        <!-- Override global_costmap's global_frame parameter -->
        <param name="global_costmap/global_frame" value="map"/>
        <!-- Pass the received map as a parameter -->
        <rosparam param="local_costmap/obstacles_layer/map_topic">{}</rosparam>
        <rosparam param="global_costmap/static_layer/map_topic">{}</rosparam>
    </node>
</launch>""".format('/map', '/map')

        # Write launch file to a temporary location
        launch_file_path = os.path.join(rospy.get_param('map_subscriber/temp_dir', '/tmp'), 'move_base_launch_file_with_map.launch')
        with open(launch_file_path, 'w') as launch_file:
            launch_file.write(launch_content)

        # Launch move_base with the generated launch file
        rospy.loginfo("Launching move_base...")
        subprocess.Popen(['roslaunch', launch_file_path])

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        map_subscriber = MapSubscriber()
        map_subscriber.run()
    except rospy.ROSInterruptException:
        pass
