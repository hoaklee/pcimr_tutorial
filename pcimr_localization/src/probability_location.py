#!/usr/bin/env python
# author Li Qianhao, qianhao.li@tum.de

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker

class probability_location:

    def __init__(self):

        # Initialize Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sensor_sub = rospy.Subscriber('/scan', LaserScan, self.callback_sensor)
        self.move_sub = rospy.Subscriber('/move', String, self.callback_move)
        self.goal_sub = rospy.Subscriber('/visualization/goal', Marker, self.callback_goal)

        # Initialize Publisher
        self.point_pub = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.grid_pub = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        # initialize parameter
        self.map_width = 20
        self.map_height = 20
        self.pos_right = 0.8
        self.pos_diffone = 0.1
        self.map_prob = np.zeros((self.map_height, self.map_height))
        # self.move_prob = rospy.get_param('~/robot_move_probabilities', [0.7, 0.1, 0.1, 0.0, 0.1])
        self.move_prob = rospy.get_param('~/robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])
        # self.random = rospy.get_param('~/rand_ini_pos', True)
        self.random = rospy.get_param('~/rand_ini_pos', False)

        # initialize data type
        self.map_input = OccupancyGrid()

        self.pub_robot_pos = Point()

        self.pub_ego_marker = Marker()
        self.pub_ego_marker.header.frame_id = "map"
        self.pub_ego_marker.ns = "navigation"
        self.pub_ego_marker.id = 0
        self.pub_ego_marker.type = Marker.CUBE
        self.pub_ego_marker.action = Marker.ADD
        self.pub_ego_marker.scale.x = 1
        self.pub_ego_marker.scale.y = 1
        self.pub_ego_marker.scale.z = 0.2
        self.pub_ego_marker.color.a = 1.0
        self.pub_ego_marker.color.r = 0.0
        self.pub_ego_marker.color.g = 1.0
        self.pub_ego_marker.color.b = 0.0
        self.pub_ego_marker.pose.orientation = Quaternion(0, 0, 0, 1)

        self.pub_grid_map = OccupancyGrid()
        self.pub_grid_map.info.resolution = 1
        self.pub_grid_map.info.height = self.map_height
        self.pub_grid_map.info.width = self.map_width

    # callback of map information
    def callback_map(self, OccupancyGrid):
        self.map_input = np.array(OccupancyGrid.data)
        self.map = self.map_input.reshape(self.map_width, self.map_height)

    # callback of sensor information
    def callback_sensor(self, LaserScan):
        self.sensor_input = np.array(LaserScan.ranges)

    # callback of move command
    def callback_move(self, String):
        self.move_input= String.data

    # callback of goal
    def callback_goal(self, Marker):
        x = Marker.pose.position.x - 0.5
        y = Marker.pose.position.y - 0.5
        self.goal_x = x
        self.goal_y = y

    # get right sensor output of every position
    def sensor_right(self):

        # set each position's sensor output to 0 first
        self.sensor_should = np.zeros((self.map_height, self.map_width, 4))

        # get sensor output of every position
        for row in range(self.map_height):
            for col in range(self.map_width):
                if self.map[row, col] == 0:

                    # here set probability of every empty position at first step
                    # to 1, since it equals to each other
                    self.map_prob[row, col] = 1

                    wall_E = 0
                    wall_W = 0
                    wall_N = 0
                    wall_S = 0

                    # calculate wall in the East
                    r_E = row
                    c_E = col
                    while self.map[r_E, c_E] != 100 and c_E < self.map_width:
                        if c_E < self.map_width-1:
                            wall_E += 1
                            c_E += 1
                        elif c_E == self.map_width-1:
                            wall_E += 1
                            break

                    # calculate wall in the West
                    r_W = row
                    c_W = col
                    while self.map[r_W, c_W] != 100 and c_W > -1:
                        wall_W += 1
                        c_W -= 1

                    # calculate wall in the North
                    r_N = row
                    c_N = col
                    while self.map[r_N, c_N] != 100 and r_N < self.map_height:
                        wall_N += 1
                        r_N += 1

                    # calculate wall in the South
                    r_S = row
                    c_S = col
                    while (self.map[r_S, c_S] != 100 and r_S > -1):
                        wall_S += 1
                        r_S -= 1

                    # assign sensor output of position
                    self.sensor_should[row][col] = np.array([wall_S, wall_W, wall_N, wall_E])

    # from sensor output get probability
    def sensor_prob(self):

        # initialize every probabilty to 0 at first
        self.map_sensor_prob = np.zeros((self.map_height, self.map_height))

        for row in range(self.map_height):
            for col in range(self.map_width):

                # if the sensor output has a 0, then the position is not valid
                if not self.sensor_should[row, col].any() == 0:

                    # calculate the difference between expected sensor output
                    position_difference = np.abs(self.sensor_should[row, col] - self.sensor_input)

                    # delete the point out of error area first
                    if max(position_difference) < 2:
                        if sum(position_difference) == 4:
                            self.map_sensor_prob[row, col] += self.pos_diffone * self.pos_diffone
                        elif sum(position_difference) == 2:
                            self.map_sensor_prob[row, col] += self.pos_diffone * self.pos_right
                        else:
                            self.map_sensor_prob[row, col] += self.pos_right * self.pos_right

    # get_direction_probability
    def get_direction_probability(self):

        # South
        if self.move_input == 'S':
            self.move_S = self.move_prob[0]
            self.move_W = self.move_prob[2]
            self.move_N = self.move_prob[3]
            self.move_E = self.move_prob[1]
        # West
        elif self.move_input == 'W':
            self.move_S = self.move_prob[1]
            self.move_W = self.move_prob[0]
            self.move_N = self.move_prob[2]
            self.move_E = self.move_prob[3]
        # North
        elif self.move_input == 'N':
            self.move_S = self.move_prob[3]
            self.move_W = self.move_prob[1]
            self.move_N = self.move_prob[0]
            self.move_E = self.move_prob[2]
        # East
        elif self.move_input == 'E':
            self.move_S = self.move_prob[2]
            self.move_W = self.move_prob[3]
            self.move_N = self.move_prob[1]
            self.move_E = self.move_prob[0]
        self.move_stay = self.move_prob[4]

    # predict position according to previous position and move command using bayes rule
    def pos_predict(self):

        # get direction probability first
        self.get_direction_probability()

        # make prediction of probability
        map_before = np.copy(self.map_prob)
        for row in range(self.map_height):
            for col in range(self.map_width):
                if map_before[row, col] != 0:

                    # move towards north
                    if self.map[row+1, col] != 100:
                        self.map_prob[row+1,col] += map_before[row, col] * self.move_N
                    else:
                        self.map_prob[row,col] += map_before[row, col] * self.move_N

                    # move towards south
                    if self.map[row-1, col] != 100:
                        self.map_prob[row-1,col] += map_before[row, col] * self.move_S
                    else:
                        self.map_prob[row,col] += map_before[row, col] * self.move_S

                    # move towards east
                    if col < self.map_width-1:
                        if self.map[row, col+1] != 100:
                            self.map_prob[row,col+1] += map_before[row, col] * self.move_E
                        else:
                            self.map_prob[row,col] += map_before[row, col] * self.move_E
                    else:
                        self.map_prob[row,col] += map_before[row, col] * self.move_E

                    # move towards west
                    if col > 0:
                        if self.map[row, col-1] != 100:
                            self.map_prob[row,col-1] += map_before[row, col] * self.move_W
                        else:
                            self.map_prob[row,col] += map_before[row, col] * self.move_W
                    else:
                        self.map_prob[row,col] += map_before[row, col] * self.move_W

    # update position probablity using sensor data
    def pos_correct(self):

        # get sensor probability map first
        self.sensor_prob()

        # fuse sensor probability with move probability
        if not self.goal_reached:
            map_prob_temp = self.map_sensor_prob * self.map_prob
            summe = sum(sum(map_prob_temp))
            # for some special circumstance, just follow move probability to avoid error
            if summe == 0:
                raise RuntimeWarning
            else:
                nomalizer = 1/summe
                self.map_prob = map_prob_temp * nomalizer
        else:
            self.map_prob = self.map_sensor_prob

        # get maximal possible x,y position
        m = self.map_prob.argmax()
        y = int(m / self.map_width)
        x = m % self.map_width

        # Publish robot position to /robot_pos
        self.pub_robot_pos.x = x
        self.pub_robot_pos.y = y
        self.point_pub.publish(self.pub_robot_pos)

        # Publish marker to /visualization/robot_pos
        self.pub_ego_marker.pose.position.x = x + 0.5
        self.pub_ego_marker.pose.position.y = y + 0.5
        self.marker_pub.publish(self.pub_ego_marker)

        # Publish probability map to /robot_pos_map
        grid_map = self.map_prob
        nomal = 1/sum(sum(grid_map))
        grid_map *= nomal * 100
        self.pub_grid_map.data = grid_map.astype(int)
        self.pub_grid_map.data = np.array(self.pub_grid_map.data.flatten())
        self.grid_pub.publish(self.pub_grid_map)

    def run(self, rate: float = 1):

        rospy.wait_for_message('/scan', LaserScan)

        # initialize probability map according to sensor input
        self.goal_reached = False
        self.sensor_right()
        self.pos_correct()

        while not rospy.is_shutdown():

            if self.random:
                # determine if it reaches the goal
                if self.goal_reached:
                    rospy.wait_for_message('/scan', LaserScan)
                    self.pos_correct()

            rospy.wait_for_message('/move', String)
            rospy.wait_for_message('/scan', LaserScan)

            self.pos_predict()
            self.pos_correct()

            if self.random:
                # if it reach the goal, set goal_reached to True
                pos_x = self.pub_robot_pos.x
                pos_y = self.pub_robot_pos.y
                if pos_x == self.goal_x and pos_y == self.goal_y:
                    self.goal_reached = True
                else:
                    self.goal_reached = False

if __name__ == "__main__":
    # initialize probability_location node
    rospy.init_node('probability_location')

    probability_location = probability_location()
    probability_location.run(rate = 1)
