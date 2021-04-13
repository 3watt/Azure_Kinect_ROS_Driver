#!/usr/bin/env python


import rospy
from azure_kinect_ros_driver.srv import initialize
# from azure_kinect_ros_driver.srv import finished, initialize


def handle_result(req):
    print("Request [%s]"%(req.finished))
    return initialize(True)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('result', initialize, handle_result)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()