import numpy as np
import math
import sympy as smp

# Contains functions for the following:
# 1. Interpolate a straight line, given two 2D points. [interp_st_line]
# 2. Estimte equation of circle, given two tangents and an apex point. [compute_circle_center]
# 3. Interpolate the circle, given center of circle and two points. [interpolate_circle]

def interp_st_line(point1, point2, thresh_dist=0.5):
	## Input arguments
	# 'point1' and 'point2' are 2D coordinates w.r.t any origin
	# 'thresh_dist' is the distance between two subsequent points after interpolation. 
	#
	## Return
	# A list of points interpolated between 'point1' and 'point2', with an approximate distance of 'thresh_dist' between consecutive points.  
	x1, y1 = point1
	x2, y2 = point2
	dist = np.linalg.norm([x1-x2, y1-y2]) # Distance b/w the end points
	theta = math.atan2(y2-y1, x2-x1) # Orientation of the line
	N = int(math.ceil(dist / thresh_dist)) # No. of intermediate points excluding the end points
	thresh_dist = dist / (N+1) # Re-estimate the 'thresh_dist', so that both end points are included in the list of points
	points = []
	for idx in range(N+2):
		temp = [float('%0.2f'%(x1 + idx * thresh_dist * np.cos(theta))), float('%0.2f'%(y1 + idx * thresh_dist * np.sin(theta)))]
		points.append(temp)
	return points

def compute_circle_center(tangent1, tangent2, apex_point):
	## Input arguments
	# 'tangent1' and 'tangent2' are of the form [x, y, theta]. It is 2D coordinates (x,y) and the orientation of the tangent (theta).
	# 'apex_point' is the 2D coordinates [x,y]
	# Objective: Find the circle that passes through the 'apex_point' and is tangent to 'tangent1' and 'tangent2'
	#
	## Return
	# A tuple (circle_center, point1, point2)
	# 'circle_center' : 2D coordinates of center of the circle
	# 'point1' : Intersection of circle with 'tangent1'
	# 'point2' : Intersection of circle with 'tangent2'

	x1, y1, th1 = tuple(tangent1)
	x2, y2, th2 = tuple(tangent2)
	a, b = tuple(apex_point)

	# Coefficients of line 1
	a1, b1, c1 = np.sin(np.deg2rad(th1)), -1*np.cos(np.deg2rad(th1)), y1*np.cos(np.deg2rad(th1))-x1*np.sin(np.deg2rad(th1))
	a1, b1, c1 = float("{0:.2f}".format(a1)), float("{0:.2f}".format(b1)), float("{0:.2f}".format(c1))

	# Coefficients of line 2
	a2, b2, c2 = np.sin(np.deg2rad(th2)), -1*np.cos(np.deg2rad(th2)), y2*np.cos(np.deg2rad(th2))-x2*np.sin(np.deg2rad(th2))
	a2, b2, c2 = float("{0:.2f}".format(a2)), float("{0:.2f}".format(b2)), float("{0:.2f}".format(c2))

	### Solving for Center of Circle
	x, y = smp.symbols('x, y', complex=False)
	mod_a1, mod_b1 = a1/smp.sqrt(a1**2+b1**2), b1/smp.sqrt(a1**2+b1**2)
	mod_a2, mod_b2 = a2/smp.sqrt(a2**2+b2**2), b2/smp.sqrt(a2**2+b2**2)
	f = (mod_a1*x + mod_b1*y + c1)**2 # distance to line 1
	g = (mod_a2*x + mod_b2*y + c2)**2 # distance to line 2
	h = (x-a)**2 + (y-b)**2 # distance to apex_point
	sols = list(smp.nonlinsolve([f-h,g-h], [x,y]))
	sols = [sol for sol in sols if smp.simplify(sol[0]).is_real] # Removing imaginary solutions
	sol_id = np.argmax([h.subs([(x,sol[0]), (y,sol[1])]) for sol in sols]) # Pick the circle with highest radius
	circle_center = sols[sol_id]
	radius_sqr = h.subs([(x,circle_center[0]), (y,circle_center[1])])

	### Solving for intersection of tangent and the circle
	x, y = smp.symbols('x, y', complex=False)
	line1 = a1*x + b1*y + c1
	line2 = a2*x + b2*y + c2
	circle = (x-float(str(circle_center[0])))**2 + (y-float(str(circle_center[1])))**2 - radius_sqr-0.000001 # just to account for overflow errors
	# Finding the intersection of tangent1 and the circle	
	sols = list(smp.nonlinsolve([line1, circle], [x,y]))
	sols = [sol for sol in sols if smp.simplify(sol[0]).is_real]
	sol_id = np.argmax([circle.subs([(x,sol[0]), (y,sol[1])]) for sol in sols])
	point1 = sols[sol_id]	
	# Finding the intersection of tangent1 and the circle	
	sols = list(smp.nonlinsolve([line2, circle], [x,y]))
	sols = [sol for sol in sols if smp.simplify(sol[0]).is_real]
	sol_id = np.argmax([circle.subs([(x,sol[0]), (y,sol[1])]) for sol in sols])
	point2 = sols[sol_id]	

	point1 = [float(str(point1[0])), float(str(point1[1]))]
	point2 = [float(str(point2[0])), float(str(point2[1]))]
	circle_center = [float(str(circle_center[0])),float(str(circle_center[1]))]

	return circle_center, point1, point2

def interpolate_circle(circle_center, point1, point2, dist_incr=0.5, offset=0):
	## Input arguments
	# 'circle_center' : 2D coordinates of the center of the circle
	# 'point1' : 2D coordinates of one point on the circle
	# 'point2' : 2D coordinates of another point on the circle
	# 'dist_incr' : distance between two consecutive points on the circle after interpolation
	# 'offset' : How many points to consider after and before the endpoints. Default to 0.

	xc, yc = tuple(circle_center)
	x1, y1 = tuple(point1)
	x2, y2 = tuple(point2)

	#print 'Circle Center: ', circle_center
	#print 'Circle - Tangent1', point1
	#print 'Circle - Tangent2', point2

	# Radius of circle
	radius = np.linalg.norm([x1-xc, y1-yc])
	#print 'radius: ', radius

	# Small theta increments
	theta_incr = dist_incr / radius # in radians

	# start point
	xs, ys = x1-xc, y1-yc

	# end point
	xe, ye = x2-xc, y2-yc

	# Overall theta
	temp = np.dot([xs, ys], [xe, ye]) / (np.linalg.norm([xs, ys]) * np.linalg.norm([xe, ye]))
	theta_overall = np.arccos(temp)

	# Number of increments 
	num_incrs = int(math.ceil( theta_overall / theta_incr ))
	#print 'num_incrs: ', num_incrs

	# Sign of theta increment
	sign = np.sign(np.cross([xs, ys], [xe, ye]))

	# Assign a direction to theta increment
	theta_incr = sign * abs(theta_incr)

	points = []
	for k in range(-1*offset,num_incrs+offset,1):
		R = np.array([[np.cos(k*theta_incr), -1*np.sin(k*theta_incr)],[np.sin(k*theta_incr), np.cos(k*theta_incr)]])
		temp = np.dot(R, np.array([[xs],[ys]])) + np.array([[xc],[yc]])
		points.append(temp.flatten().tolist())

	return points

