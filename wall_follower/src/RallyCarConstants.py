
## LIDAR constants
LIDAR_DIFF_THETAS = range(-20,20,5) # in degrees  ###--- Change this thing too. It might estimate really bad during the turns. 
WALL_DIR = 'left'
LIDAR_SCAN_RATE = 40

## Rally car constants
RALLY_SPEED = 1 # in m/sec # ~ 10% ###--- The minimum speed was 1m/s when 182 is sent to serial port.
FIXED_ENCODED_SPEED = 200
GAS = 10 # in percent

## Controller constants
TARGET_DISTANCE = 0.5 # in meters
PD_UPDATE_FREQ = 150
MAX_TURN = 50

# Tuning parameters
KP = 168
KD = 800
KI = 0
SCALE_FACTOR = 2048/50.0
#2048 / float(KP * 0.5 + KD * 0.05) # multiply the result of PD with this scale factor

STOP_FREQ = 12 # A no. between 0 and 10: (3-1)/3 of the speed

## LOG
# (1000,2000) is working okay. Taking too large curvatures
# 1500, 3000 worked well
# 1600, 11000 worked really well
# 1500, 15500 worked okay

# 40,370 is great
