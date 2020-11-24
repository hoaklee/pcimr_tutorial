#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import Point

import rospy

def callback_pos(Point):
    # print(Point.x, Point.y)
    pos = [Point.x, Point.y]
    rospy.loginfo(f'Listener:robot position is:{pos}')

def robot_pos_listener():
    rospy.init_node('robot_pos_listener', anonymous=True)
    rospy.Subscriber('robot_pos', Point, callback_pos)
    rospy.spin()

if __name__ == '__main__':
    robot_pos_listener()