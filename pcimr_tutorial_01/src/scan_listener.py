#!/usr/bin/env python

from __future__ import print_function
from sensor_msgs.msg import LaserScan

import rospy

def callback_scan(LaserScan):
    LaserRange = LaserScan.ranges
    rospy.loginfo(f'Listener:Laser Range is:{LaserRange}')

def scan_listener():
    rospy.init_node('scan_listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback_scan)
    rospy.spin()

if __name__ == '__main__':
    scan_listener()
