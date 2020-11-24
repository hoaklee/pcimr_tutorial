#!/usr/bin/env python

from __future__ import print_function
from pcimr_simulation.srv import *

import rospy
import sys

def init_pos_client():
    # waiting for available service
    rospy.wait_for_service('init_pos')
    try:
        # define service
        init_position_client = rospy.ServiceProxy('init_pos', InitPos)
        # call service
        resp = init_position_client.call(2,0)

        rospy.loginfo('Position initialization success')
    except rospy.ServiceException as e:
        rospy.logwarn('Position initialization failed')

if __name__ == "__main__":
    print("Requesting position initialization")
    init_pos_client()
