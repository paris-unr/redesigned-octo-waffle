#!/usr/bin/env python3
import rospy
from misty_wrapper.misty_scan import *
"""
Creates Misty-scanning popup window and passes resulting ip to nodes via rosparams
"""

if __name__ == "__main__":
    # idx = rospy.get_param(rospy.get_name() + "/robot_id")
    rospy.init_node("mistyscan")
    idx = rospy.get_param("~robot_id")
    ip = initial_ip_scan_window()
    rospy.loginfo('Using Misty IP: ' + str(ip))
    rospy.set_param("/misty/id_" + str(idx) + "/robot_ip", ip)
