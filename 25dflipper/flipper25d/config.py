import numpy as np
# Arena
EXP_FOLDER = ['exp']
EXP_ROT_RANGE = [0,5,10,15,20,25,30,35,40,45]
STEP_ARENA_NAME_LIST = ['step_'+str(ang) for ang in EXP_ROT_RANGE]
RAMP_ARENA_NAME_LIST = ['ramp_'+str(ang) for ang in EXP_ROT_RANGE] 
IRAMP_ARENA_NAME_LIST = ['iramp_'+str(ang) for ang in EXP_ROT_RANGE]  
#ARENA_NAME = STEP_ARENA_NAME_LIST[0]

#Robot config (unit: m)
WHEEL_RADIUS = 0.035
WIDTH = 0.15
WIDTH_EPSILON = 0.003
LEN_S0S1 = 0.135
LEN_S1S0 = LEN_S0S1
LEN_S1S2 = 0.145
LEN_S2S1 = LEN_S1S2
LEN_S2S3 = 0.135
LEN_S2S1S0 = LEN_S0S1 + LEN_S1S2

HEIGHT_EPSILON = 0.01


# png map to height map to expand map
PIXEL_RESO = 0.003
WHEEL_RADIUS = 0.035
KERNEL_SIZE = 27
RAMP_HEIGHT_LOW = 0.02
RAMP_HEIGHT_HIGH = 0.179#0.1607695154586736
#RAMP_RATIO = 0.1607695154586736/(100-30) 

# angle range
YAWS = [ang*np.pi/4-np.pi*3/4 for ang in range(8)]
PITCHS = [ang*np.pi/10 - np.pi/2 for ang in range(1,10)]
ROLLS = [-np.pi/4, 0, np.pi/4]

PITCH_IT = np.pi/20


PITCH_UB = 60./180*np.pi
PITCH_LB = -PITCH_UB


# flipper angle
DEFAULT_ALPHA = 0.57
DEFAULT_BETA = 0.57

FLIPPER_ANGLE_LB = -np.pi/2

ALPHA_ITS = 0.01

H_ITS = 0.025



#center for man4 and man6
#the pixel index where is the localtion of xyz origin
#ORIGIN = [0,0,WHEEL_RADIUS]
DIST_S2_ROT_AXIS = 0.54
# step, ramp, iramp  800x800
ORIGIN_ = [399+int(DIST_S2_ROT_AXIS/PIXEL_RESO), 399, WHEEL_RADIUS]#[2332,837,WHEEL_RADIUS]
TARGET_ = [350, 399, WHEEL_RADIUS]

# con_ramp 2000x800
#ORIGIN_ = [999+int(DIST_S2_ROT_AXIS/PIXEL_RESO), 399, WHEEL_RADIUS]#[2332,837,WHEEL_RADIUS]
#TARGET_ = [160, 399, WHEEL_RADIUS] 


# on the division
EPSILON_DIV = 1e-16

# one the planar for roll
EPSILON_H_PLANE = 0.0


# vis.py
WINDOW_SIZE = 195


# path_search.py
RANGE_SIZE = 1 #PIXEL only 1 because we have eight yaw choice and robot can only move in this direction
VALID_PAIR = [(1,0,np.pi),\
                (1,1,np.pi*3./4),\
                (0,1,np.pi/2),\
                (-1,1,np.pi/4),\
                (-1,0, 0.),\
                (-1,-1,-np.pi/4),\
                (0,-1,-np.pi/2),\
                (1,-1,-np.pi*3./4)]


R_RANGE = list(range(-RANGE_SIZE,RANGE_SIZE+1))
C_RANGE = list(range(-RANGE_SIZE,RANGE_SIZE+1)) 
RC_ANGLE_RANGE = np.pi/2

ALPHA_RANGE = np.pi/4
H_RANGE = 0.05


#GO STRAIGHT ITS in get_neighbours
GO_STRAIGHT_ITS = 5



#########################################################
######## get_data.sh add file things delete later
#########################################################
#ARENA_NAME = IRAMP_ARENA_NAME_LIST[3]
ARENA_NAME = 'con_ramp'
