#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np

import math

import os

from interpolation_module import *

from global_constants import waypoints_path

plot_flag = True
dist_incr = 0.6
waypoint_list = np.array([[0.0,0.0]])

## Initial Turn : ID = 1
print 'Initial Turn : ID = 1'
tangent1 = [0.0, 0.0, 0]
tangent2 = [1.8, 1.0, 90]
apex_point = [0.8, 0.4]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list =np.append(waypoint_list[:-1], temp, axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None
# point2 is not clearred on purpose. For continuity

## Long Straight Line : ID = 2
print 'Long Straight Line : ID = 2'
point1 = point2
point2 = [3.1, 32.0]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[1:], axis = 0)
point1, point2, temp = None, None, None

## Left Turn : ID = 3
print 'Left Turn : ID = 3'
tangent1 = [3.1, 35.5, 90]
tangent2 = [1.13, 37.9, 180]
apex_point = [2.8, 35.6]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp, axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None

## Middle Straight Line : ID = 4
print 'Straight Line : ID = 4'
point1 = point2
point2 = [-17.0, 38.5]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[1:-2], axis = 0)
point1, point2, temp = None, None, None

# One additional point since there is more space
waypoint_list = np.append(waypoint_list, [[-16.74, 39.0]], axis = 0)


## Left Turn : ID = 5
print 'Left Turn : ID = 5'
tangent1 = [-16.74, 38.5, 180]
tangent2 = [-19.4, 37.0, -90]
apex_point = [-18.17, 37.2]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp[2:], axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None

##  Straight Line : ID = 6
print 'Straight Line : ID = 6'
point1 = point2
point2 = [-19.4, 23.5]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[2:], axis = 0)
point1, point2, temp = None, None, None

# One additional point
waypoint_list = np.append(waypoint_list, [[-19.4, 22.8]], axis = 0)

## Zig Zag Part 1 : ID = 7
print 'Zig Zag Part 1 : ID = 7'
tangent1 = [-19.1, 21.0, -90]
tangent2 = [-18.0, 20.0, 0]
apex_point = [-18.03, 20.66]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp[:-1], axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None

## Zig Zag Part 2 : ID = 8
print 'Zig Zag Part 2 : ID = 8'
tangent1 = [-18.0, 19.5, 0]
tangent2 = [-16.4, 18.0, -90]
apex_point = [-17., 18.75]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp[4:], axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None

## Short Straight Line : ID = 9
print 'Straight Line : ID = 9'
point1 = point2
point2 = [-16.0, 13.77]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[1:], axis = 0)
point1, point2, temp = None, None, None


## 45 degree turn : ID = 10
print 'Turn : ID = 10'
tangent1 = [-16.0, 13.77, -90]
tangent2 = [-15.07, 10.89, -45]
apex_point = [-15.63, 11.79]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp, axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None


## 45 degree line : ID = 11
print 'Straight Line : ID = 11'
point1 = point2
point2 = [-8.01, 3.48]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[1:], axis = 0)
point1, temp = None, None

## 45 degree line Extension : ID = 11.5
print '45 degree line Extension : ID = 11.5'
point1 = point2
point2 = [-6.78, 1.14]
temp = interp_st_line(point1, point2, thresh_dist=dist_incr)
waypoint_list = np.append(waypoint_list, temp[:-1], axis = 0)
point1, temp = None, None

'''
## Slight Turn : ID = 12
print 'Slight Turn : ID = 12'
tangent1 = [-8.5, 3.69, -45.0]
tangent2 = [-7, 1.73, -90.0]
apex_point = [-7.2, 2.52]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp, axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None
'''
## Slight Turn 2 : ID = 13
print 'Slight Turn : ID = 13'
tangent1 = [-6.91, 1.73, -90.0]
tangent2 = [-5.24, 0.7, 0.0]
apex_point = [-6.49, 0.98]
circle_center, point1, point2 = compute_circle_center(tangent1, tangent2, apex_point)
temp = interpolate_circle(circle_center, point1, point2, dist_incr=dist_incr, offset=0)
waypoint_list = np.append(waypoint_list[:-1], temp[2:-1], axis = 0)
tangent1, tangent2, apex_point, circle_center, point1, temp = None,None,None,None,None,None

## Final Straight Line : ID = 14
print 'Final Straight Line : ID = 14'
point1 = point2
point2 = [0.0, 0.6]
temp = interp_st_line(point1, point2, thresh_dist=1.5)
waypoint_list = np.append(waypoint_list, temp[1:], axis = 0)
point1, point2, temp = None, None, None

## Preprocessing
#actual_wps = [waypoint_list[0].tolist(), waypoint_list[1].tolist()]
#for idx in range(2,waypoint_list.shape[0]):
#	alph1 = math.atan2(waypoint_list[idx-1][1]-waypoint_list[idx-2][1], waypoint_list[idx-1][0]-waypoint_list[idx-2][0])
#	alph2 = math.atan2(waypoint_list[idx][1]-waypoint_list[idx-1][1], waypoint_list[idx][0]-#waypoint_list[idx-1][0])
#	if ((alph2 - alph1) < 90) or ((alph2 - alph1) > -90):
#		actual_wps.append([waypoint_list[idx][0], waypoint_list[idx][1]])
#
#$waypoint_list = np.array(actual_wps)

## Appending zeros at the end
waypoint_list = np.append(waypoint_list, np.zeros((waypoint_list.shape[0],1)), axis = 1)

## Printing
print 'No. of waypoints: ', waypoint_list.shape[0]
print 'Waypoints: '
print waypoint_list

temp = np.linalg.norm(waypoint_list[:-1] - waypoint_list[1:], axis=1).flatten()
print 'Min Dist: ', np.min(temp), np.argmin(temp)
print 'Max Dist: ', np.max(temp), np.argmax(temp)
print 'Mean Dist: ', np.mean(temp)

np.savez(waypoints_path, waypoints=waypoint_list)

## Plotting
if plot_flag:
	wps = np.load(waypoints_path)['waypoints']
	plt.plot(wps[:,0], wps[:,1])
	plt.grid()
	plt.show()



