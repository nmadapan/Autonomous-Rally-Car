#!/usr/bin/env python

## ROS
import rospy
from std_msgs.msg import String

## Serial Port
import serial

## Normal Python
import numpy as np
import time 
import math

class IMU():
	def __init__(self):
		## IMU Initialization
		print '--- Initializing the IMU : Start ---'
		self.console_ser = serial.Serial('/dev/ttyACM0',115200)
		self.console_ser.close()
		self.console_ser.open()
		time.sleep(2)
		self.console_ser.flush()
		print self.console_ser.read(62)
		self.console_ser.write('IMU1')
		self.buffer_data = []
		self.buffer_size = 100 # More the better
		self.gravity = 9.81
		self.imu_const = 2 * self.gravity / 32768.0
		self.initialize()

		## Initialize ROS
		rospy.init_node('IMU')
		self.imu_pub = rospy.Publisher('/imu_data', String, queue_size=10)
		self.rate = rospy.Rate(150)
		print '--- Initializing the IMU : Complete ---'
		self.run()
		
	def initialize(self):
		# Find: rotation matrix from Inertial frame to body frame
		# Find: Estimated gravity and initial accelerations
		# Find: Gravity
		#for idx in range(self.buffer_size):
		#	read = self.console_ser.read(22)
		count = 0
		while(len(self.buffer_data)<self.buffer_size):
			if self.console_ser.inWaiting() > 0: 
				read = self.console_ser.read(22)
				ax = self.imu_const * float(read[1:7])
				ay = self.imu_const * float(read[7:13])
				az = self.imu_const * float(read[13:19])
				self.buffer_data.append([ax, ay, az])
				#print "ax:", ax, "ay:", ay, "az:", az
				#print ''	
			else:
				count += 1
				if count > 900000: 
					print 'No data from IMU'
					count = 0
	
		static_acc = np.median(np.array(self.buffer_data), axis=0)
		self.est_gravity = np.linalg.norm(static_acc)
		#print 'Esimated Gravity: ', self.est_gravity
		self.R_I2B = self.estimate_3d_rot((static_acc/self.est_gravity).tolist(), [0,0,1])
		self.ref_acc_imu = np.dot(self.R_I2B, static_acc.tolist())

		# Update gravity and imu constant using reference imu accelerations. 
		self.gravity = self.ref_acc_imu[2]
		self.imu_const = 2 * self.gravity / 32768.0
		#print 'Rotation Matrix: ', self.R_I2B

	def print_all(self):
		print 'gravity: ', self.gravity
		print 'imu_constant: ', self.imu_const
		print 'Rotation Matrix: ', self.R_I2B
		print 'ref_acc_imu: ', self.ref_acc_imu

	def estimate_3d_rot(self, point1, point2):
		# point1 and point2 should be of unit length
		# Returns R such that point2 = R * point1
		a, b = point1, point2
		v = np.cross(a,b)
		c = np.dot(a,b)
		s = np.linalg.norm(v)
		I = np.identity(3)
		vXStr = '{} {} {}; {} {} {}; {} {} {}'.format(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0)
		k = np.array(np.matrix(vXStr))
		if s < 1e-6:
			R = I
		else:
			R = I + k + np.matmul(k,k) * ((1 -c)/(s**2))
		return np.array(R)
		
	def run(self):
		self.print_all()
		print 'Publishing the IMU Data .... '
		while (not rospy.is_shutdown()):
			if self.console_ser.inWaiting() > 0: 
				read = self.console_ser.read(22)
				#print read
				ax = self.imu_const * float(read[1:7])
				ay = self.imu_const * float(read[7:13])
				az = self.imu_const * float(read[13:19])
				self.buffer_data = self.buffer_data[1:]
				self.buffer_data.append([ax,ay,az])
				dynamic_acc = np.median(np.array(self.buffer_data), axis=0)
				dynamic_acc = np.dot(self.R_I2B, dynamic_acc).flatten()
				dynamic_acc -= self.ref_acc_imu
				dynamic_acc = -1 * dynamic_acc
				#print 'After: ', dynamic_acc
				#print ''
			self.imu_pub.publish('_'.join(map(str,dynamic_acc.tolist())))		
			self.rate.sleep()
		
imu = IMU()
imu.run()
rospy.spin()





