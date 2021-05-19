# -*- coding: utf-8 -*-
#!/usr/bin/env python2  
'''
   actually, when record the bag, I miss recog the node. 
   the recorded [fl,fr,bl,br] should actually be [bl, fl, br, fr]
'''
import rospy
import numpy as np
from message_filters import Subscriber,ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped, Pose
from joy_control.msg import Float64StampedMultiArray
import tf 

exp_name = 'iramp_35'
dire = '../0726_bag/'

pose_lst = []
pose_time_lst = []
param_lst = []
param_time_lst = []

def callback(center):
    '''
       output is Nx4x3
    '''
    global pose_lst, pose_time_lst
    pt = center
    geo = np.zeros((2,3))
    orient = center.pose.orientation
    quaternion = (orient.x, orient.y, orient.z, orient.w) 
    euler_ang = tf.transformations.euler_from_quaternion(quaternion,'rzyx')

    geo[0,0] = pt.pose.position.x
    geo[0,1] = pt.pose.position.y
    geo[0,2] = pt.pose.position.z

    geo[1,0] = euler_ang[0]
    geo[1,1] = euler_ang[1]
    geo[1,2] = euler_ang[2]

    pose_lst.append(geo)
    pose_time_lst.append(pt.header.stamp.secs+pt.header.stamp.nsecs*1e-9)

def param_callback(param):
    global param_lst, param_time_lst
    param_lst.append(param.array.data)
    param_time_lst.append(param.header.stamp.secs+param.header.stamp.nsecs*1e-9)

def listener():
    rospy.init_node('listerner', anonymous=True)

    '''
    ts = ApproximateTimeSynchronizer(\
            [Subscriber('/vrpn_client_node/fl/pose', PoseStamped),\
            Subscriber('/vrpn_client_node/fr/pose', PoseStamped),\
            Subscriber('/vrpn_client_node/bl/pose', PoseStamped),\
            Subscriber('/vrpn_client_node/br/pose', PoseStamped)], queue_size=100, slop=0.1)
    '''
    rospy.Subscriber('/record_inner_param', Float64StampedMultiArray, param_callback)
    rospy.Subscriber('/vrpn_client_node/test1/pose',PoseStamped, callback)
    #ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    finally:
        print('writing mat..')
        import scipy.io
        scipy.io.savemat('./mat/'+exp_name+'.mat', {'poses': np.stack(pose_lst), 'poses_time':np.array(pose_time_lst),'params':np.stack(param_lst), 'params_time':np.array(param_time_lst)})
        print('saved the mat')
