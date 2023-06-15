#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock


def callback(msg):
    sim_time = msg.clock
    real_time = rospy.get_rostime()
    rospy.loginfo(f"Simulation time: {sim_time}\t\tReal time: {real_time}")


rospy.init_node('time_scale_factor_node')
rospy.Subscriber('/clock', Clock, callback)
rospy.spin()
