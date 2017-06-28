#!/usr/bin/env python
from __future__ import division
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage

img = None
count = 0

def CornerDetect(Img_frame, Img_gray):
	feature_params = dict( maxCorners   = 50,
                           qualityLevel = 0.3,
                           minDistance  = 10,
                           blockSize    = 7 )
	blur = cv2.medianBlur(Img_gray, 5)
	rth, Img_gray = cv2.threshold(blur, 25, 255, 0)
	p0 = cv2.goodFeaturesToTrack(Img_gray, mask=None, **feature_params)
	for i in range (0,len(p0)):
		cv2.circle(Img_frame, (p0[i][0][0], p0[i][0][1]), 5, (142, 164, 255), -1)
	#cv2.imshow('Corner', Img_frame)
	Img_frame = cv2.cvtColor(Img_gray, cv2.COLOR_GRAY2BGR)
	mask = np.zeros_like(Img_frame)
	return p0, mask

def OpticalFlow(Img_gray, preImg_gray, p0):
	lk_params = dict( winSize  = (30,30),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS |cv2.TERM_CRITERIA_COUNT, 10, 0.03))
	print '======================'
	print Img_gray.shape
	print preImg_gray.shape
	print '========++++=========='
	p1, st, err = cv2.calcOpticalFlowPyrLK(preImg_gray, Img_gray, p0, None, **lk_params)
	return p1, st

def FindObject(new_pos, old_pos, Img_gray):
	global count
	count_newLast, index = 0, 0
	est_obj = []
	find_obj = False
	row, col = Img_gray.shape[0], Img_gray.shape[1]
	x_ref, y_ref = col/2, row
	
	Img_frame = cv2.cvtColor(Img_gray, cv2.COLOR_GRAY2BGR)
	if len(new_pos) == 0:
		new_pos = old_pos
	for i in range (0,len(new_pos)):
		count_newLast += 1
		x, y = new_pos[i][0], new_pos[i][1]
		theta = ((math.atan2((x_ref - x), (y_ref - y))) * 360) / (2 * np.pi)
		r = np.sqrt(np.power(x - x_ref, 2) + np.power(y_ref - y, 2))
		r_met, theta_met = meters(x, y, x_ref, y_ref)
		#print theta_met , r_met
		if(-45 < theta < 45):  ##Limit theta
			est_obj.append((x, y))
			find_obj = True
	if find_obj:	
		for j in range (0,len(est_obj)):
			Obj_x, Obj_y = est_obj[j][0], est_obj[j][1]
			if ((j > 0) and (pre_est_x == Obj_x) and (pre_est_y == Obj_y)): ##Avoid Duplicate Position
				print "Duplicate"
			else:
				r_met, theta_met = meters(Obj_x, Obj_y, x_ref, y_ref)
				print "Estimate # %d frame Object # %d: R = %.2f meters, Theta = %.2f" % (count, j + 1, r_met, theta_met)
				cv2.circle(Img_frame, (Obj_x, Obj_y), 5, (102, 255, 204), -1)
				pre_est_x, pre_est_y = Obj_x, Obj_y
		#cv2.imshow('Object n frame', Img_frame)
	else:
		print "Can't find Object"

def meters(x,y,x_ref, y_ref):
	pix_met = float(20/y_ref)
	y_met, x_met = y*pix_met, x*pix_met
	x_ref, y_ref = x_ref*pix_met, y_ref*pix_met
	r = np.sqrt(np.power(x_met - x_ref, 2) + np.power(y_met - y_ref, 2))
	theta = ((math.atan2((x_ref - x_met), (y_ref - y_met))) * 360) / (2 * np.pi)
	theta = theta
	"""print "===meter==="
	print r
	print theta"""
	return r, theta	

class main:

	def __init__(self):
		self.Img_frame = None
		self.preImg_gray = None
		self.new_pos, self.old_pos = [], []
		self.p0, self.p1 = [], []
		self.good_new,self.good_old = [], []
		self.last_frame = False
		
		self.subTopic = '/img_sonar/compressed'
		self.pubTopic = '/processing/compressed'
		self.sub = rospy.Subscriber(self.subTopic, CompressedImage, self.callback,  queue_size = 1)
		#self.pub = rospy.Publisher(self.pubTopic , String, queue_size = 4)

	def callback(self, ros_data):
		width = 500
		height = 200
		np_arr = np.fromstring(ros_data.data, np.uint8)
		self.Img_frame = cv2.resize(cv2.imdecode(np_arr, 1),(width, height))
		

	def Process(self):
		global count
		while(self.Img_frame is None):
			rospy.sleep(0.01)
		
		while not rospy.is_shutdown():
			if self.Img_frame is None:
				continue
			count += 1

			left = int(round((self.Img_frame.shape[1]/2)-(self.Img_frame.shape[1]/4)))
			right = int(round((self.Img_frame.shape[1]/2)+(self.Img_frame.shape[1]/4)))

			#print count
			self.Img_gray = cv2.cvtColor(self.Img_frame, cv2.COLOR_BGR2GRAY) 

			if count == 1:
				self.p0, mask = CornerDetect(self.Img_frame, self.Img_gray)
				self.preImg_gray = self.Img_gray.copy()

			else:
				self.new_pos = []
				self.old_pos = []
				ref_gray = self.Img_gray.copy()
				self.p1, st = OpticalFlow(self.Img_gray, self.preImg_gray, self.p0)
				self.good_new = self.p1[st == 1]
				self.good_old = self.p0[st == 1]
				for i, (new, old) in enumerate(zip(self.good_new, self.good_old)):
					a, b = new.ravel()
					c, d = old.ravel()
					if (a > left) and (a < right):
						self.new_pos.append((a,b))
						self.old_pos.append((c,d))
						cv2.line(mask, (a, b), (c, d), (255, 212, 128), 2)
						cv2.circle(self.Img_frame, (a, b), 5, (77, 77, 255), -1)
				img = cv2.add(self.Img_frame, mask)
				cv2.imshow('frame', img)
				##__Next_Frame__##
				self.p0 = self.good_new.reshape(-1, 1, 2)
				self.preImg_gray = ref_gray.copy()
			FindObject(self.new_pos, self.old_pos, self.Img_gray)
			k = cv2.waitKey(30) & 0xff
			if k == 27:
				break

if __name__ == '__main__':
	rospy.init_node('Tracking', anonymous=True)
	try:
		preImg = main()
		preImg.Process()
	except KeyboardInterrupt:
		print("Shutting down")
