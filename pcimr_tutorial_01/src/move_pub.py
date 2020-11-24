#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import String

import rospy

def run(rate: float = 2):
    # Initialize Publisher
    move = rospy.Publisher('/move', String, queue_size=10)
    rospy.init_node('move_pub', anonymous=True)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        move.publish('N')
        rate.sleep()

if __name__ == '__main__':
    run()
