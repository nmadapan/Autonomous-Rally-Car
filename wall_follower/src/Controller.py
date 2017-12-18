#!/usr/bin/env python

import rospy
import numpy as np
import math
import serial
#import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from RallyCarConstants import *
import time

console_ser = serial.Serial("/dev/ttyACM0", baudrate = 115200)
console_ser.close()
console_ser.open()

previousError = 0
LIDAR_FLAG = False
FILE_WRITE_FLAG = True

class Controller(object):
	def __init__(self):
		rospy.init_node("controller")
		self.rate = rospy.Rate(PD_UPDATE_FREQ)
		self.distanceListener = rospy.Subscriber('/distance', String, self.distanceCallback)
		if FILE_WRITE_FLAG: self.fileptr = open('/home/crl1/ros_wsps/car_ws/src/wall_follower/d2wall.txt','w')
		self.start_scan_time = 0

	def computeAngleShift(self, d1, th1, d2, th2):
		nr = d2*math.cos(np.deg2rad(th2)) - d1*math.cos(np.deg2rad(th1))
		dr = d2*math.sin(np.deg2rad(th2)) - d1*math.sin(np.deg2rad(th1))
		return math.atan2(nr, dr)

	def computeDistanceToWall(self, ANGLE_SHIFT):
		return self.LIDAR_REF_DIST * math.cos(ANGLE_SHIFT)
		###--- We need to add this too. If not ask Mythra what they did. 
		#(temp + (RALLY_SPEED/float(PD_UPDATE_FREQ)) * math.sin(ANGLE_SHIFT)) # Ignored because the \Delta(t) is too small. It is basically the 1/clock_frequency

	def distanceCallback(self, data):
		global LIDAR_FLAG
		global previousError
		values = [float(val) for val in data.data.split('_')]
		self.LIDAR_REF_DIST = values[0]
		off_dists = values[1:]
		ANGLE_SHIFTS = []
		for first_idx in range(len(LIDAR_DIFF_THETAS)):
			for second_idx in range(first_idx+1,len(LIDAR_DIFF_THETAS),1):
				ANGLE_SHIFTS.append(self.computeAngleShift(off_dists[first_idx], LIDAR_DIFF_THETAS[first_idx], off_dists[second_idx], LIDAR_DIFF_THETAS[second_idx]))
		self.ANGLE_SHIFT = np.median(ANGLE_SHIFTS)
		#print 'Angle shift: ', np.rad2deg(self.ANGLE_SHIFT)
		self.oldDistance = self.computeDistanceToWall(self.ANGLE_SHIFT)
		#print "Old Distance: ", self.oldDistance
		print 'Time between the LidarScans: ', (time.time()-self.start_scan_time)
		print 'Instantaneous frequency: ', 1.0 / (time.time()-self.start_scan_time)
		print 'Distance at +90 degrees: ', self.LIDAR_REF_DIST
		print ''
		self.start_scan_time = time.time()
		LIDAR_FLAG = True

	def control_rally_car(self, input_str):
		console_ser.write(input_str)

	def run(self):
		global LIDAR_FLAG, previousError
		counter = 0
		while (not rospy.is_shutdown()):			
			if LIDAR_FLAG:	
				counter += 1

				###--- The order in which they execute is different / and *. Put it in the circular braces. 
				self.oldDistance = self.oldDistance + ((RALLY_SPEED / float(PD_UPDATE_FREQ)) * math.sin(self.ANGLE_SHIFT)) #### CHANGE ANGLE SHIFT

				self.distanceToWall = self.oldDistance

				## PD
				self.error = TARGET_DISTANCE - self.distanceToWall
				#print "Error Diff: ",(previousError - self.error), "Current Error: " , self.error, "Distance to Wall: " , self.distanceToWall
				vTheta = KP * self.error + KD * (previousError - self.error)
				if vTheta < 0:
					vTheta = max(vTheta, -1.0 * MAX_TURN)
				else:
					vTheta = min(vTheta, MAX_TURN)					
				
				#print 'D2Wall: ', self.distanceToWall, 'E: ', self.error, 'dE: ', (previousError - self.error), 'V: ', vTheta
				if FILE_WRITE_FLAG: self.fileptr.write(str(self.LIDAR_REF_DIST)+' '+str(self.distanceToWall)+' '+str(self.error)+' '+str(previousError - self.error)+' '+str(vTheta)+'\n')
				previousError = self.error

				## Find out the scale. Converting vTheta into -2048 to 2048
				if abs(vTheta) < 0.95 * MAX_TURN:
					input_str = 'A'+'%+05d'%int(SCALE_FACTOR*vTheta)+'%+05d'%int(FIXED_ENCODED_SPEED)
				else:
					if counter%STOP_FREQ == 0:
						input_str = 'A'+'%+05d'%int(SCALE_FACTOR*vTheta)+'%+05d'%int(0)
					else:
						input_str = 'A'+'%+05d'%int(SCALE_FACTOR*vTheta)+'%+05d'%int(FIXED_ENCODED_SPEED)
				
				#print input_str

				self.control_rally_car(input_str)
				# Resetting counter when it is 1000
				if counter == 10000: counter = 1
				
				self.rate.sleep()

		if rospy.is_shutdown():
			self.control_rally_car('A+0000+0000')
			if FILE_WRITE_FLAG: self.fileptr.flush(); self.fileptr.close()
			console_ser.close()

if __name__ == '__main__':
	controller = Controller()
	controller.run()
