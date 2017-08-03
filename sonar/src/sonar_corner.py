#!/usr/bin/env python
from __future__ import division
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

from sonar.msg import sonar_msg
from sonar.srv import sonar_srv

bridge = CvBridge()

Img_frame = None
count = 0
index = 0
p0 = [],

Range = 20 #!!
dis_obj = 2.6	#Navigate = 1.8, 2.6
r_thres = 17 #!!
bin_thres = 90	#Navigate = 75
theta_out = 0
angle_thres = 40
#r_out, theta_out = [], []

r_met = 0
status = False
theta_met = 0

mask = None

def Findobject():
	global p0, Img_frame, Img_sh, r_out, theta_out, min_dis, max_dis
	global Range, dis_obj, r_thres,angle_thres, count
	#r_out, theta_out = [], []
	dis_pos = []
	row, col = Img_frame.shape[0], Img_frame.shape[1]
	x_ref, y_ref = col/2, row
	min_dis, max_dis = 1.1/(float(Range/y_ref)), dis_obj/(float(Range/y_ref)) 
	chk = False

	if count % 5 == 0:
		r_thres = r_thres - 1
		print count, r_thres

	for i in range (0,len(p0)):
		x, y = p0[i][0][0], p0[i][0][1]
		r_met = meters(x, y, x_ref, y_ref)
		if r_met <= r_thres:
			cv2.circle(Img_frame, (x, y), 5, (102, 255, 204), -1)
			dis_pos.append((x, y))
	#pubIm(Img_frame)

	for i in range (0,len(dis_pos)):
		x, y = dis_pos[i][0], dis_pos[i][1]
		cv2.circle(Img_frame, (x, y), 5, (102, 255, 204), -1)
		##__LEFT(+)__RIGHT(-)__##
		angle = ((math.atan2((x_ref - x), (y_ref - y))) * 360) / (2 * np.pi)
		##__Limit__ANGLE___##
		if -(angle_thres) <= angle <= angle_thres:		
			print angle
			for j in range (0, len(dis_pos)):
				xi, yi = dis_pos[j][0], dis_pos[j][1]
				distance = np.sqrt(np.power(x - xi, 2) + np.power(y - yi, 2))			
				if min_dis <= distance <= max_dis:
					if x < xi:
						if y < yi: 
							#print "1"
							theta = 180-((math.atan2((yi - y), (xi - x))) * 360) / (2 * np.pi)
						else: #y > yi
							#print "2"
							theta = ((math.atan2((y - yi), (xi - x))) * 360) / (2 * np.pi)
					else: #x > xi
						if y < yi:
							#print "3"
							theta = ((math.atan2((yi - y), (x - xi))) * 360) / (2 * np.pi)
						else: #y > yi
							#print "4"
							theta = 180-((math.atan2((y - yi), (x - xi))) * 360) / (2 * np.pi)
					if (0 <= theta <= 45 or 135 <= theta <= 180):
						cv2.line(Img_frame, (x, y), (xi, yi), (255, 212, 128), 2)
						chk = True
						#print "=======theta====="
						theta_out = theta
						#print theta
	if chk:
		print "Can't find"
	pubIm(Img_frame)
	return theta_out, chk


def CornerDetect(Img_gray):
	global Img_frame, p0, mask, bin_thres
	feature_params = dict( maxCorners   = 50,
                           qualityLevel = 0.3,
                           minDistance  = 10,
                           blockSize    = 7 )
	blur = cv2.medianBlur(Img_gray, 5)
	rth, Img_gray_th = cv2.threshold(blur, bin_thres, 255, 0)
	p0 = cv2.goodFeaturesToTrack(Img_gray_th, mask=None, **feature_params)
	for i in range (0,len(p0)):
		cv2.circle(Img_frame, (p0[i][0][0], p0[i][0][1]), 5, (142, 164, 255), -1)
	#cv2.imshow('Corner', Img_frame)
	Img_frame = cv2.cvtColor(Img_gray, cv2.COLOR_GRAY2BGR)
	mask = np.zeros_like(Img_frame)
	return p0, mask

def pubIm(im):
	img_show = np.array(im, np.uint8)
	msg1 = CompressedImage()
	msg1.format = "jpeg"
	msg1.header.stamp = rospy.Time.now()
	msg1.data = np.array(cv2.imencode('.jpg', img_show)[1]).tostring()
	pub.publish(msg1)

def meters(x,y,x_ref, y_ref):
	global r_met, theta_met, status
	pix_met = float(20/y_ref)
	y_met, x_met = y*pix_met, x*pix_met
	x_ref, y_ref = x_ref*pix_met, y_ref*pix_met
	r = np.sqrt(np.power(x_met - x_ref, 2) + np.power(y_met - y_ref, 2))
	theta = ((math.atan2((x_ref - x_met), (y_ref - y_met))) * 360) / (2 * np.pi)
	theta = theta
	return r

def Convolution(Img_conv):

	kernel = np.ones((9,9))
	#print kernel
	for i in range (0,3):
		for j in range (0,3):
			kernel[i+2][j+2] = 0

	# print kernel
	"""
	y,x = np.ogrid[-2: 2+1, -2: 2+1]
	kernel = x**2+y**2 <= 1**2
	kernel = kernel.astype(float)
	kernel = np.abs(kernel - 1)
	print kernel"""

	rth, Img_conv = cv2.threshold(Img_conv, bin_thres, 255, 0)
	Img_conv = cv2.filter2D(Img_conv,-1,kernel/49)
	rth, Img_conv = cv2.threshold(Img_conv, 40, 255, 0)
	return Img_conv

def Process():
	global Img_frame, p0, count
	global r_out, theta_out
	last_frame = False
	res = sonar_msg()  #!!

	while(Img_frame is None):
		print "Img_frame: None"
		rospy.sleep(0.01)
		continue

	count += 1
	print count
	left = int(round((Img_frame.shape[1]/2)-(Img_frame.shape[1]/4)))
	right = int(round((Img_frame.shape[1]/2)+(Img_frame.shape[1]/4)))
	
	Img_gray = cv2.cvtColor(Img_frame, cv2.COLOR_BGR2GRAY)

	if count == 1:
		print ("++")
		cv2.imwrite("/home/johny/Desktop/im.jpg", Img_gray)

	Img_conv = Convolution(Img_gray)
	#print "Corner", count
	p0, mask = CornerDetect(Img_conv)
	#print "Find Object"
	tt, st = Findobject()
	tt = int(round(tt))
	
	res.r = []
	res.theta =tt
	res.status = st
	return res

def image_callback(ros_data):
	global Img_frame, index
	index += 1
	#print "index = %s" %index
	try:
		#Img_frame = cv2.resize(bridge.imgmsg_to_cv2(ros_data, "bgr8"),(width, height))
		Img_frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
	except CvBridgeError, e:
		print (e)

def tracking_callback(msg):
	#print msg
	#print "tracking callback"
	return Process()

if __name__ == '__main__':
	rospy.init_node('SonarTracking', anonymous=True)
	
	pub = rospy.Publisher('/image/Tracking', CompressedImage, queue_size=1)
    
	subTopic = '/imaging_sonar'
	sub = rospy.Subscriber(subTopic, Image, image_callback,  queue_size = 1)
	rospy.Service('/sonar_image', sonar_srv(), tracking_callback)
	rospy.spin()		
