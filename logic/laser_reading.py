#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    rospy.loginfo(msg)


rospy.init_node('laser_reading')

sub = rospy.Subscriber('talos/laser/scan', LaserScan, laser_callback)

rospy.spin()
