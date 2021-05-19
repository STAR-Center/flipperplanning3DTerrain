# -*- coding: utf-8 -*-
#!/usr/bin/env python2
'''
    no worry about the delta flipper angle

    but delta_m might be complicated:
        from yaw_t-1, 0., 0. to yaw_t, pitch_t, roll_t,
        delta_m = delta_m_from_yaw_roll_dist + delta_m_from_roll

        roll will make it a triangular
    
'''
from __future__ import division
import rospy
import numpy as np
from message_filters import Subscriber,ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from dynamixel_workbench_controllers.msg import XM2
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray, Header
from flipper.msg import Float64StampedMultiArray
import tf
import copy
from path_search import *
#from traverse_d import CSpace

#from apscheduler.scheduler import Scheduler

#sched = Scheduler()
#sched.start()

'''
    feedback pose with OptTrack
    
    Initiallly, 
    Robot orientation is on x with 
        ^ x
        |
   y <--|
          -
            -
               z


    The OptTrack is with
   z <---
        | -
        |   - y
       x

    but the topic in /vrpn_client_node/test1/pose is
        |----->y
        | -
        |   -
        x     - z
'''




first_frame = True

first_xm3 = 0
first_xm4 = 0
cur_xm3 = 0
cur_xm4 = 0
cur_wheel_pose = PoseStamped()

first_posi = None
first_euler = None
cur_vrpn_pose = PoseStamped()
CAR_WIDTH = 0.12

# the xm3, xm4 when we plan and run
st_xm3 = 0
st_xm4 = 0

# opt tracking
init_OptPosi = None
init_OptOri = None

#in_planning = False # if we are in planning run
on_planning_run = False# first frame we on planning run

#path_lst = []# path that will be updated in the first trigger frame
current_pt = None
next_pt = None
last_point = -1 #-1 next is not last, 0 next is last, 1 curr is last
last_m_right = 0 #m from start to currentpoint
last_m_left = 0
first_gt_R = True # first time a > R

D_START = 0.40#0.40
H_START = 0.095-0.049#0.028#0.095 - 0.0
R = 0.035

REACH_ESPILON = 1e-2
STEADY_VELOCITY = 0.8
STEADY_ANGULAR = 0#angular = 0 means not rotate
DELTA_ANGLE_CODE = 0.01 * 1.5

alpha_commandCode = 0.
back_alpha_commandCode = 0.
velocity = 0.
angular = 0

global control_pub, record_param_pub

def alpha2commandCode(alpha_in):
    alpha = min(56/180*np.pi, alpha_in)
    alpha = max(-np.pi/2, alpha)
    code = (-alpha+56/180*np.pi) / np.pi
    #code = code if code < 0.81111 else 0.81111
    code = code if code < 1. else 1.

    return code
def commandCode2alpha(comm):
    alpha = 56/180*np.pi - comm * np.pi
    return alpha
def reach_target(delta_m, wanted_delta_m):
    if wanted_delta_m >= 0:
        if delta_m - wanted_delta_m > 1e-6:
            return True
        else:
            return False
    else:
        if wanted_delta_m - delta_m > 1e-6:# negative move
            return True
        else:
            return False

def euler_from_quaternion(orient, seq='rzyx'):
    quaternion = (orient.x, orient.y, orient.z, orient.w)
    return tf.transformations.euler_from_quaternion(quaternion,seq)
def publish():
    global velocity, angular,lalpha_commandCode, lback_alpha_commandCode, ralpha_commandCode, rback_alpha_commandCode, control_pub
    command = Float64MultiArray()
    #rospy.loginfo("command is %f, %f;  v=%f, w=%f"%(lalpha_commandCode, lback_alpha_commandCode,velocity, angular))
    command.data.append(velocity)
    command.data.append(angular)

    command.data.append(lalpha_commandCode)#
    command.data.append(ralpha_commandCode)#

    command.data.append(lback_alpha_commandCode)#note that the range is 0~1 that is assume to be pi/2~-pi/2
    command.data.append(rback_alpha_commandCode)#
    
    control_pub.publish(command)

def publish_param():
    '''
        also publish the target Sl2, Sr2, yaw,pitch,roll, l_alpha,r_alpha,l_beta,r_beta and current alpha, beta.
        array[6+7+4]
        '''
    global next_pt, lalpha_commandCode, lback_alpha_commandCode, ralpha_commandCode, rback_alpha_commandCode
    record_data = Float64StampedMultiArray()
    record_data.header = Header()
    record_data.header.stamp = rospy.get_rostime()
    record_data.header.frame_id = 'robotInnerConfig'

    for i in range(3):
        record_data.array.data.append(next_pt[0][i])
        record_data.array.data.append(next_pt[1][i])
    for i in range(2,9):
        record_data.array.data.append(next_pt[i])
    record_data.array.data.append(commandCode2alpha(lalpha_commandCode))
    record_data.array.data.append(commandCode2alpha(ralpha_commandCode))
    record_data.array.data.append(commandCode2alpha(lback_alpha_commandCode))
    record_data.array.data.append(commandCode2alpha(rback_alpha_commandCode)) 
    record_param_pub.publish(record_data)

def callback(xm3, xm4, OptPoseStamped):
    global st_xm3, st_xm4, in_planning, on_planning_run, path_lst, control_pub, current_pt, next_pt, ralpha_commandCode, rback_alpha_commandCode, lalpha_commandCode, lback_alpha_commandCode, velocity, angular, last_point, first_gt_R, last_m_right, last_m_left, init_OptPosi, init_OptOri
    velocity = STEADY_VELOCITY
    angular = STEADY_ANGULAR
    
    
    if in_planning:
        if not on_planning_run:
            # in the first frame we make plan and get path
            on_planning_run = True
            st_xm3 = xm3.Position_Trajectory
            st_xm4 = xm4.Position_Trajectory
            
            cur_l, cur_r = middle_to_S2([ORIGIN_, 0.,0.,0.,0.])  #(D_START, R, 56/180*np.pi, 56*180/np.pi, 0., 0.145)#path_lst.pop()
            current_pt = [cur_l[0], cur_r[0], 0., 0., 0., DEFAULT_ALPHA, DEFAULT_ALPHA, DEFAULT_BETA, DEFAULT_BETA]
            #path_lst = path_lst[50:]
            #current_pt = path_lst.pop()
            next_pt = path_lst.pop()
            
            last_m_right = 0
            last_m_left = 0
            
            #d,a,alpha,back_alpha,theta,t = current_pt
            p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha,r_alpha,l_beta,r_beta = current_pt
            
            # since the upper bound is 56, so we the allowed code is in 56 to -90
            # the inital angle code should be 0
            lalpha_commandCode = alpha2commandCode(l_alpha)
            lback_alpha_commandCode = alpha2commandCode(l_beta)
            ralpha_commandCode = alpha2commandCode(r_alpha)
            rback_alpha_commandCode = alpha2commandCode(r_beta)


            init_OptOri = euler_from_quaternion(OptPoseStamped.pose.orientation)
            init_OptPosi = -OptPoseStamped.pose.position.x - LEN_S1S2/2*np.cos(init_OptOri[1])


        else:

            el_ori = euler_from_quaternion(OptPoseStamped.pose.orientation)
            el_posi = -OptPoseStamped.pose.position.x - LEN_S1S2/2*np.cos(el_ori[1])

            #rospy.loginfo('%f %f %f'%(pitch_dist, l_yaw_roll_dist, r_yaw_roll_dist))
            p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha,r_alpha,l_beta,r_beta = next_pt 
            #compute the angle to set angular and velocity
            #rospy.loginfo("%f %f %f"%(el_ori[0], el_ori[1], el_ori[2]))
            real_angle = -el_ori[0]#(m_r_minus-m_l_minus)/WIDTH#the angle to compute angular
            wanted_t = .3
            ang = real_angle/wanted_t
            #(vr-vl/WIDTH) = ang, vr/vl = m_r_minus/m_l_minus
            if False:#abs(ang > 0.1):
                vel = 0.3
            else:
                vel = STEADY_VELOCITY
            m_minus = (ORIGIN_[0]-p_S2_l[0])*PIXEL_RESO
            #rospy.loginfo('The loc %f %f %f'%(OptPoseStamped.pose.position.x, OptPoseStamped.pose.position.y,OptPoseStamped.pose.position.z))


            rospy.loginfo('%f %f %f'%(el_posi-init_OptPosi, m_minus, el_ori[1])) 
            if reach_target(el_posi-init_OptPosi, m_minus):
                rospy.loginfo('reach target %f'%el_posi)
                #rospy.loginfo('########################################################################################m_right %f, laft_m %f, m_minus %f'%(m_right, last_m, m_minus))
                current_pt = next_pt
                if len(path_lst) == 0:
                    if last_point == -1:
                        last_point=0
                    elif last_point == 0:
                        last_point = 1
                else:
                    next_pt = path_lst.pop()
                if last_point == 1:
                    velocity = 0.
            else:
                velocity = vel
                angular = ang
            #rospy.loginfo('vel ang %f %f'%(velocity, angular ))

        #d,a,alpha,back_alpha,theta, t = next_pt
        p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha,r_alpha,l_beta,r_beta = next_pt 
        next_lalpha_commandCode = alpha2commandCode(l_alpha)
        next_lback_alpha_commandCode = alpha2commandCode(l_beta-0.06981317007977318)
        next_ralpha_commandCode = alpha2commandCode(r_alpha)
        next_rback_alpha_commandCode = alpha2commandCode(r_beta-0.06981317007977318) 
    if abs(next_lalpha_commandCode - lalpha_commandCode) < DELTA_ANGLE_CODE:
        lalpha_commandCode = next_lalpha_commandCode
    else:
        if next_lalpha_commandCode > lalpha_commandCode:
            lalpha_commandCode += DELTA_ANGLE_CODE
        else:
            lalpha_commandCode -= DELTA_ANGLE_CODE
    if abs(next_lback_alpha_commandCode - lback_alpha_commandCode) < DELTA_ANGLE_CODE:
        lback_alpha_commandCode = next_lback_alpha_commandCode
    else:
        if next_lback_alpha_commandCode > lback_alpha_commandCode:
            lback_alpha_commandCode += DELTA_ANGLE_CODE
        else:
            lback_alpha_commandCode -= DELTA_ANGLE_CODE
    if abs(next_ralpha_commandCode - ralpha_commandCode) < DELTA_ANGLE_CODE:
        ralpha_commandCode = next_ralpha_commandCode
    else:
        if next_ralpha_commandCode > ralpha_commandCode:
            ralpha_commandCode += DELTA_ANGLE_CODE
        else:
            ralpha_commandCode -= DELTA_ANGLE_CODE
    if abs(next_rback_alpha_commandCode - rback_alpha_commandCode) < DELTA_ANGLE_CODE:
        rback_alpha_commandCode = next_rback_alpha_commandCode
    else:
        if next_rback_alpha_commandCode > rback_alpha_commandCode:
            rback_alpha_commandCode += DELTA_ANGLE_CODE
        else:
            rback_alpha_commandCode -= DELTA_ANGLE_CODE 

    #rospy.loginfo("A::command is %f, %f, next_command is %f, %f  on theta %f, alpha %f, back_alpha %f"%(alpha_commandCode, back_alpha_commandCode, next_alpha_commandCode, next_back_alpha_commandCode, theta, alpha, back_alpha))
    publish()
    publish_param()


def listener():
    global control_pub, record_param_pub
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    #rospy.Subscriber('/camera/imu', Imu, callback1)
    #rospy.Subscriber('/dynamixel/XM3', XM2, callback2)
    #rospy.Subsriber('/dynamixel/XM4', XM2, callback3)
    #rospy.Subscriber('/vrpn_client_node/camera1/pose', PoseStamped, callback4)
    
    ts = ApproximateTimeSynchronizer( [Subscriber('/dynamixel/right', XM2),
                                       Subscriber('/dynamixel/left', XM2),
                                       Subscriber('/vrpn_client_node/test1/pose', PoseStamped)], queue_size=100, slop = 0.1 )
    ts.registerCallback(callback)
                                       
    control_pub = rospy.Publisher('Wheel',Float64MultiArray,queue_size=10)
    record_param_pub = rospy.Publisher('record_inner_param',Float64StampedMultiArray,queue_size=10)

                                       # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    #--- navigation and trigger, but now only fix and trigger
    global path_lst, in_planning
    
    #cspace = CSpace(h=H_START, di=D_START
    #path_lst = [cspace.tuple3_to_tuple6(p) for p in path_lst]
    import scipy.io
    mat = scipy.io.loadmat('exp/conf_path/'+ARENA_NAME+'expanded_config_path.mat')
    rospy.loginfo('len is%f'%(len(mat['path'].tolist())))
    path_lst = mat['path'].tolist()
    path_lst = [[l[0][0], l[1][0], l[2], l[3], l[4], l[5], l[6], l[7], l[8]]  for l in path_lst]
    path_lst = path_lst[::-1]


    #path_lst = path_lst[::-1]
    
    in_planning = True
    #---
    try:
        listener()
    except Exception as e:
        print(e)
    finally:
        print('hahahahahahahaha, no problem, I should be on the stair!')
