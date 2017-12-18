## Global constants. 
# All of the files in the package use these parameters. 
# Note: Change the paths when the files in the package are moved to another package.

## Paths
waypoints_rviz_path = '/home/crl1/ros_wsps/car_ws/src/knoy_speedway_basic/Backup/waypoints_rviz.npz' # The file path to save the waypoints clicked on the map in rviz. 
waypoints_path = '/home/crl1/ros_wsps/car_ws/src/knoy_speedway_basic/waypoints.npz' # The file path to save the interpolated waypoints.

## Controller constants
PD_UPDATE_FREQ = 150

## PD Throttle control
KP_GAS = 800 
KD_GAS = 500

## PD Steer control
KP_STEER = 18 
KD_STEER = 30 
STEER_THRESH = 1200.0 # If absolute value of 'STEER_THRESH' is greater than 1200, reduce the speed. 

## Position and Orientation Constants
INITAL_X = 0
INITAL_Y = 0
INITAL_YAW = -1

## Waypoint is updated when the car is 70% close to the target .
WP_UPDATE_LIMIT = 0.7

## Distance threshold
DISTANCE_THRESHOLD = 1.1

DECISION_WINDOW = 30# MUST be an integer
DECISION_AHEAD_THRESHOLD = 12

MAX_SPEED = 480
