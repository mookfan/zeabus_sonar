#!/usr/bin/env python

import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

if __name__ == '__main__':

    rospy.init_node('VideoPublisher', anonymous=True)
    pub = rospy.Publisher('/img_sonar/compressed', CompressedImage, queue_size=1)
    cam = cv2.VideoCapture('/home/johny/Desktop/sonar/v_1530_5.avi')
    count = 0
    while not rospy.is_shutdown() :
        meta, frame = cam.read()
        if frame is None:
            continue
        # print(type(frame))
        count += 1
        frame = np.array(frame,np.uint8)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.header.stamp = rospy.Time.now()
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #print msg.data
        pub.publish(msg)
        #print count
        cv2.imshow('img',frame)
        if not cam.isOpened():
            print('video finish')
            break
        k = cv2.waitKey(30) & 0xff
        if k == ord('q'):
            break