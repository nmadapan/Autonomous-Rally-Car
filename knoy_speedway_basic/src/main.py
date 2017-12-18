#!/usr/bin/env python

## ROS
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

## Serial Port
import serial

## Normal Python
import numpy as np
import time 
import math

## User defined
from global_constants import *

class Controller():
	def __init__(self):
		## AMCL Initialization
		self.current_x = 0
		self.current_y = 0
		self.current_yaw = -1
		self.desired_yaw = -1
		self.amcl_count = 0
		self.start_time = time.time()

		## ROS Initialization		
		rospy.init_node("controller")

		## Serial port Initialization
		self.consoleSer = serial.Serial("/dev/ttyACM0", baudrate = 115200)
		self.consoleSer.close()
		self.consoleSer.open()
		self.inputStr = ''

		## PD Controller Initialization
		self.ratePD = rospy.Rate(PD_UPDATE_FREQ)
		self.prev_error_throt = 0
		self.prev_error_steer = 0
		self.error_dist = 0 # Distance between current position to target position # This is like error_throttle
		self.error_steer = 0

		## Waypoint Initialization
		self.wp_count = 0
		self.prev_wp = {'x':0, 'y':0, 'yaw':0}
		self.targ_wp = {'x':0, 'y':0, 'yaw':0}
		self.error_x_prime = 999999999 # Error in which the car is moving
		# Initially, first waypoint is previous and next waypoint is the target. 
		self.prev_wp['x'], self.prev_wp['y'], self.prev_wp['yaw'] = tuple(TARGET[self.wp_count])
		self.targ_wp['x'], self.targ_wp['y'], self.targ_wp['yaw'] = tuple(TARGET[self.wp_count+1])
		self.dist_bw_wps = np.linalg.norm([self.targ_wp['x']-self.prev_wp['x'], self.targ_wp['y']-self.prev_wp['y']]) # Distance between previous and target waypoints

		## IMU Initialization
		self.imu_count = 0
		self.imu_current_x = self.current_x # In meters
		self.imu_current_y = self.current_y # In meters
		self.imu_current_yaw = self.current_yaw # In degrees
		self.imu_ux_prime = 0.0
		self.imu_uy_prime = 0.0
		self.imu_prev_time = time.time()
		self.imu_print_flag = False
		imu_sub = rospy.Subscriber('/imu_data', String, self.imu_callback)

		self.deviation_ahead = 20
		
	def imu_callback(self, data):
		self.imu_count += 1
		# Wait for first amcl reading
		if self.amcl_count > 0 :
			dt = time.time() - self.imu_prev_time	
			# Acceleration in IMU Frame of reference
			ax_prime, ay_prime, az_prime = map(float,data.data.split('_'))
			self.imu_ux_prime += ax_prime * dt
			self.imu_uy_prime += ay_prime * dt
			dx_prime = self.imu_ux_prime * dt + 0.5 * ax_prime * dt * dt
			dy_prime = self.imu_uy_prime * dt + 0.5 * ay_prime * dt * dt

			angle = np.deg2rad(self.imu_current_yaw)
			dx, dy = np.dot([[np.cos(angle), -1*np.sin(angle)],[np.sin(angle), np.cos(angle)]], [[dx_prime],[dy_prime]]).flatten().tolist()
			if np.linalg.norm([dx_prime,dy_prime]) < 1e-0:
				beta = 0
			else:
				beta = np.rad2deg(math.atan2(dy_prime, dx_prime))
			if self.imu_print_flag and self.imu_count%10 == 0:
				print 'Timestamp: ', time.time()-self.start_time
				print 'Acceleration primes: ', 'dt', dt, 'ax_prime', ax_prime, 'ay_prime', ay_prime, 'az_prime', az_prime
				print 'Velocity primes: ', 'imu_ux_prime', self.imu_ux_prime, 'imu_uy_prime', self.imu_uy_prime
				print 'Position primes', 'dx_prime: ', dx_prime, 'dy_prime: ', dy_prime

				print 'Positions: ', 'dx: ', dx, 'dy: ', dy, 'beta: ', beta

			self.imu_current_yaw += beta
			self.imu_current_x += dx
			self.imu_current_y += dy
			if self.imu_print_flag and self.imu_count%10 == 0:
				print 'Positions: ', 'imu_current_x: ', self.imu_current_x, 'imu_current_y: ', self.imu_current_y, 'imu_current_yaw: ', self.imu_current_yaw
				print ''

			self.imu_current_yaw = np.sign(self.imu_current_yaw) * (abs(self.imu_current_yaw)%180)
			self.imu_prev_time = time.time()
		

	def amclCallback(self, data):
		self.amcl_count += 1
		position = data.pose.pose.position
		self.current_x = position.x
		self.current_y = position.y

		orientation = data.pose.pose.orientation
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler_angles = tf.transformations.euler_from_quaternion(quaternion)
		self.current_yaw = np.rad2deg(euler_angles[2])

		# Updating current corrdinates of IMU
		if self.imu_print_flag:
			print 'State: ', 'imu_x', self.imu_current_x, 'imu_y', self.imu_current_y, 'imu_yaw', self.imu_current_yaw
		self.imu_current_x = self.current_x
		self.imu_current_y = self.current_y
		self.imu_current_yaw = self.current_yaw
		self.print_status()
		print ''

	def print_status(self):
		print "AMCL %d: %0.3f"%(self.amcl_count, time.time() - self.start_time)
		print "Current Location: ", self.current_x, self.current_y, "Current Yaw:" , self.current_yaw
		print "Error Dist: ", self.error_dist,  "Error Steer: ", self.error_steer
		print  "error to target waypoint: ", self.error_x_prime
		print 'Input String: ', self.inputStr
		print 'Target waypoint: ', self.targ_wp, 'Desired Yaw: ', self.desired_yaw
		print "" 

	def error_distance(self):
		errorX = (self.current_x - self.targ_wp['x'])**2
		errorY = (self.current_y - self.targ_wp['y'])**2
		return (errorX + errorY)**0.5
		
	def run(self):
		while (not rospy.is_shutdown()):
			
			## Throttle Control
			self.error_dist = self.error_distance() # This value is alwasy +ve irrespective of whether or not we past the target. 
			correctionTh = KP_GAS * self.error_dist + KD_GAS * (self.prev_error_throt - self.error_dist)
			self.prev_error_throt = self.error_dist
			

			## Steer Control
			self.desired_yaw = np.rad2deg(math.atan2(self.targ_wp['y']-self.current_y, self.targ_wp['x']-self.current_x))
			self.error_steer = self.desired_yaw - self.current_yaw # current_yaw is already in degrees. 
			
			if self.error_steer > 180:
				self.error_steer = -360 + self.error_steer
			elif self.error_steer < -180:
				self.error_steer = 360 + self.error_steer

			correctionSt = -1 * (KP_STEER * self.error_steer + KD_STEER * (self.error_steer - self.prev_error_steer)) # -1 bcoz rally car and AMCL angles are opposite.
			correctionSt = np.sign(correctionSt) * min(2048.0, abs(correctionSt))
			self.prev_error_steer = self.error_steer

			theta = math.atan2(self.targ_wp['y']-self.prev_wp['y'], self.targ_wp['x']-self.prev_wp['x'])

			R = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
			self.error_x_prime = np.dot(R, [self.targ_wp['x']-self.current_x, self.targ_wp['y']-self.current_y])[0]
			
			# Things to handle: 1. breaking or not when waypoints are over (Done), 2. sign of ex is important(Done), 3. tune the absolute of ex
			#if (abs(self.error_x_prime) < WP_UPDATE_LIMIT * self.dist_bw_wps) and self.wp_count < len(TARGET)-2:
			if (self.error_dist < DISTANCE_THRESHOLD  and self.wp_count < len(TARGET)-2):
				self.wp_count += 1
				self.prev_wp['x'], self.prev_wp['y'], self.prev_wp['yaw'] = tuple(TARGET[self.wp_count])
				self.targ_wp['x'], self.targ_wp['y'], self.targ_wp['yaw'] = tuple(TARGET[self.wp_count+1])
				self.dist_bw_wps = np.linalg.norm([self.targ_wp['x']-self.prev_wp['x'], self.targ_wp['y']-self.prev_wp['y']])	
				theta = math.atan2(self.targ_wp['y']-self.prev_wp['y'], self.targ_wp['x']-self.prev_wp['x'])
				R = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
				self.error_x_prime = np.dot(R, [self.targ_wp['x']-self.current_x, self.targ_wp['y']-self.current_y])[0]		
				#print('Updating the waypoint: ', self.targ_wp.values())


			if (abs(self.error_x_prime) < WP_UPDATE_LIMIT * self.dist_bw_wps or self.error_x_prime < 0.0) and (self.wp_count == len(TARGET)-2):
				#print  "self.error_x_prime in stopping condition: ", self.error_x_prime, 'self.dist_bw_wps: ', self.dist_bw_wps
				#print 'Stopping ...'
				self.consoleSer.write('A+0000+0000')
				correctionSt = 0
				correctionTh = 0
				#self.consoleSer.close()
				#rospy.on_shutdown(nothing)
				break

			if self.wp_count < int(math.floor((len(TARGET) - DECISION_WINDOW))):
				decison_angles = []
				for wpidx in range(self.wp_count, self.wp_count+DECISION_WINDOW-3):
					decison_angles.append(abs(np.rad2deg(math.atan2(TARGET[wpidx+1][1]-TARGET[wpidx][1], TARGET[wpidx+1][0]-TARGET[wpidx][0]))))
				self.deviation_ahead = abs(np.max(decison_angles) - np.mean(decison_angles))
				
			if abs(float(correctionSt)) >= STEER_THRESH or self.deviation_ahead > DECISION_AHEAD_THRESHOLD:
				correctionTh = min(correctionTh, 200)
			else:
				correctionTh = min(correctionTh, MAX_SPEED)
 			
			self.inputStr = 'A'+'%+05d'%int(float(correctionSt))+'%+05d'%int((float(correctionTh)))
			self.consoleSer.write(self.inputStr)
			self.ratePD.sleep()

		if rospy.is_shutdown():
			self.consoleSer.write('A+0000+0000')
			self.consoleSer.close()			

TARGET = np.load(waypoints_path)['waypoints'].tolist()

control_obj = Controller()
#print "Starting time: ", control_obj.start_time
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, control_obj.amclCallback)
control_obj.run()
rospy.spin()



	
	
