#!/usr/bin/env python

## ROS
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

## Serial Port
import serial

## Normal Python
import numpy as np
import time 
import math

from global_constants import waypoints_path

class Plot_on_map():
	def __init__(self):
		rospy.init_node('plot_waypoints_on_map')
		self.wpt_pub = rospy.Publisher('/waypts', Marker, queue_size = 100)
		self.rate = rospy.Rate(100)
		self.waypoints = np.load(waypoints_path)['waypoints'].tolist()
		## Initialize Marker
		self.markers_list = []
		self.wpt_marker = Marker()
		self.wpt_marker.header.frame_id = "/map"
		self.wpt_marker.type = self.wpt_marker.POINTS
		self.wpt_marker.action = self.wpt_marker.ADD
		self.wpt_marker.scale.x = 0.1
		self.wpt_marker.scale.y = 0.1
		self.wpt_marker.scale.z = 0.1
		self.wpt_marker.color.a = 0.8
		self.wpt_marker.color.r = 1.0;
		self.wpt_marker.color.g = 0.0;
		self.wpt_marker.color.b = 0.2; 
		self.wpt_marker.pose.orientation.w = 1.0
		self.wpt_marker.lifetime = rospy.Duration(0)
		self.plot_waypoints_on_map()


	def plot_waypoints_on_map(self):
		for idx in range(len(self.waypoints)):
			point = Point()
			point.x = self.waypoints[idx][0]
			point.y = self.waypoints[idx][1]
			self.wpt_marker.points.append(point)

		while not rospy.is_shutdown():
			self.wpt_pub.publish(self.wpt_marker)
			self.rate.sleep()

plt = Plot_on_map()
rospy.spin()

