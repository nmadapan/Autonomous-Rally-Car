#!/usr/bin/env python

import rospy
import numpy as np
import math
#import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from std_msgs.msg import String
from RallyCarConstants import *
import time

class LidarScan(object):
	def __init__(self):
		rospy.init_node("lidarscan")

		self.scanListener = rospy.Subscriber('/scan', LaserScan, self.scanCallback)
		self.scanInfo = rospy.Publisher('/distance', String, queue_size=10)
		self.rate = rospy.Rate(LIDAR_SCAN_RATE)
		self.LIDAR_COUNT = 0

	def scanCallback(self, data):
		start_time = time.time()
		if data.header.seq == 0 or self.LIDAR_COUNT == 0:		
			self.LIDAR_COUNT = self.LIDAR_COUNT + 1
			self.ANGLE_INCREMENT = data.angle_increment # in radians
			self.SCAN_TIME_INCREMENT = data.scan_time # in seconds
			if WALL_DIR == 'left':
				self.LIDAR_REF_INDEX = np.int(np.deg2rad(225)/self.ANGLE_INCREMENT) ###--- Check this with Mythra ---##
				self.LIDAR_OFF_INDICES = [self.LIDAR_REF_INDEX - np.int(np.deg2rad(LIDAR_DIFF_THETA)/self.ANGLE_INCREMENT) for LIDAR_DIFF_THETA in LIDAR_DIFF_THETAS]
			elif WALL_DIR == 'right':
				self.LIDAR_REF_INDEX = np.int(np.deg2rad(45)/self.ANGLE_INCREMENT)
				self.LIDAR_OFF_INDICES = [self.LIDAR_REF_INDEX + np.int(np.deg2rad(LIDAR_DIFF_THETA)/self.ANGLE_INCREMENT) for LIDAR_DIFF_THETA in LIDAR_DIFF_THETAS]
			else:
				print('WALL_DIR takes wrong string')
		self.LIDAR_REF_DIST = data.ranges[self.LIDAR_REF_INDEX] # b
		self.LIDAR_OFF_DISTS = [data.ranges[LIDAR_OFF_INDEX] for LIDAR_OFF_INDEX in self.LIDAR_OFF_INDICES]  # a

		packet = '_'.join([str(self.LIDAR_REF_DIST), '_'.join(map(str,self.LIDAR_OFF_DISTS))])
		self.scanInfo.publish(packet)

		#print 'Time taken (s): ', time.time()- start_time ,'Distance to Wall (m): ', self.LIDAR_REF_DIST
		self.rate.sleep()


	def run(self):
		rospy.spin()


if __name__ == '__main__':
	lidarScan = LidarScan()
	lidarScan.run()
