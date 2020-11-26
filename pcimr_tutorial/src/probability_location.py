# Author LiQianhao Qianhao.li@tum.de
#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker
from pcimr_simulation.srv import InitPos


class probability_location:
    """
    A node
    """

    def __init__(self):

        # Initialize Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sensor_sub = rospy.Subscriber('/scan', LaserScan, self.callback_sensor)
        self.move_sub = rospy.Subscriber('/move', String, self.callback_move)

        # Initialize Publisher
        self.point_pub = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.grid_pub = rospy.Publisher('/map_updates', OccupancyGrid, queue_size=10)

    # callback of
    def callback_map(self, OccupancyGrid):
        self.map_input = OccupancyGrid

    # callback of
    def callback_sensor(self, LaserScan):
        self.sensor_input = LaserScan

    # callback of
    def callback_move(self, String):
        self.move_input= String

    def run(self, rate: float = 1):
        while not rospy.is_shutdown():

            if rate:
               rospy.sleep(1/rate)

if __name__ == "__main__":
    rospy.init_node('probability_location')

    probability_location = probability_location()
    probability_location.run(rate = 1)