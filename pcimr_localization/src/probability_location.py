#!/usr/bin/env python
# author Li Qianhao, qianhao.li@tum.de

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker
# from pcimr_simulation.srv import InitPos

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
        # self.grid_pub = rospy.Publisher('/map_update', OccupancyGrid, queue_size=10)

    # callback of map information
    def callback_map(self, OccupancyGrid):
        self.map_input = OccupancyGrid

    # callback of sensor information
    def callback_sensor(self, LaserScan):
        self.sensor_input = LaserScan

    # callback of move command
    def callback_move(self, String):
        self.move_input= String

    # # initialize position at the first step
    # def pos_init(self):


    # # predict position according to previous position and move command using bayes rule
    # def pos_predict(self):
    #     testtest

    # # update position probablity using sensor data
    # def pos_update(self):
    #     testtesttest

    def run(self, rate: float = 1):
        # initialize position
        # pos_init()
        while not rospy.is_shutdown():

            print(self.map_input)
            # pos_predict()
            # pos_update()

            if rate:
               rospy.sleep(1/rate)

if __name__ == "__main__":
    # initialize probability_location node
    rospy.init_node('probability_location')

    probability_location = probability_location()
    probability_location.run(rate = 1)
