#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sonar.msg import sonar_msg
from sonar.srv import sonar_srv
sonar_image = None

def server(start):
    global sonar_image
    # try:      
    response = sonar_image(start)
    print response
    return response.theta, response.r, response.status
    # except  rospy.ServiceException, e:
    #     print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Waiting"
    start = True
    rospy.wait_for_service('sonar_image')
    sonar_image = rospy.ServiceProxy('sonar_image', sonar_srv)
    print 'service start'
    while not rospy.is_shutdown():
        # response = sonar_image(start)
        # server(start)    
        # print "theta = %s, r = %s, status = %r" %(theta, r, server(start))
        response = sonar_image(start)
        print response