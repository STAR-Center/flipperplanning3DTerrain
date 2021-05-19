'''
    1. loc bias
    2. roll, pitch, yaw bias

   world coordinate: (OptTrack will provide the pose of robot base center)
    ----->y
    | -
    |   -
    |     -z
    x

    experimental coordinate:
            ^ x
            |
            |
            |
    <-------|
    y        -
                -
                  - z
'''
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.io
from flipper25d.config import *
import pdb


def get_Smiddle2(p_loc,yaw,pitch,line_len):
    '''
        from p_, to the orientation of (yaw,pitch) with line_len
        here pitch is neg pitch
    '''
    delta_y = np.sin(yaw)*line_len*np.cos(pitch)
    delta_x = np.cos(yaw)*line_len*np.cos(pitch)
    delta_h = line_len*np.sin(pitch)
    #on the h, it is possible the h is little smaller than map h, then we use the map h
    return [p_loc[0]+delta_x,p_loc[1]+delta_y,p_loc[2]+delta_h]
 

def getRotation(rx,ry,rz, wx=np.array([[1,0,0]]).T, wy=np.array([[0,1,0]]).T, wz=np.array([[0,0,1]]).T):
    '''
       from world coordinate to reference coordinate
       input are four 3x1 vectors

       return M that ref_ori = M x world_ori


       note:wx=[1,0,0]'
            wy=[0,1,0]'
            wz=[0,0,1]'
    '''
    M = np.zeros((3,3))
    M[0,0] = np.dot(rx.T,wx)
    M[0,1] = np.dot(rx.T,wy)
    M[0,2] = np.dot(rx.T,wz)
    M[1,0] = np.dot(ry.T,wx)
    M[1,1] = np.dot(ry.T,wy)
    M[1,2] = np.dot(ry.T,wz)
    M[2,0] = np.dot(rz.T,wx)
    M[2,1] = np.dot(rz.T,wy)
    M[2,2] = np.dot(rz.T,wz)
    return M

def getTransformation(ro, wo=np.array([[0,0,0]]).T):
    '''
       return T that ref_loc in world R = T + world_loc
    '''
    T = wo-ro
    return T

def transform(M, T, loc, ori):
    '''
       loc: world_loc
       ori: world_ori
       M:
       T:
    '''
    ref_loc = np.dot(M, T+loc)
    ref_ori = np.dot(M, ori)
    return ref_loc, ref_ori

def getOriginAndOrientation(ref_pose, use_s2 = True):

    if use_s2:#s2 as the origin
        ro = (ref_pose[0,:] + ref_pose[2,:])/2
    else:#center as the origin
        ro = np.mean(ref_pose,axis=0)

    rx = (ref_pose[1,:]-ref_pose[0,:])/2+ (ref_pose[3,:]-ref_pose[2,:])/2
    rx = rx/np.sqrt(np.sum(rx**2))

    ry = (ref_pose[1,:]-ref_pose[3,:])/2+ (ref_pose[0,:]-ref_pose[2,:])/2
    ry = ry/np.sqrt(np.sum(ry**2))

    rz = np.cross(rx,ry)
    return ro.reshape((3,1)), rx.reshape((3,1)), ry.reshape((3,1)), rz.reshape((3,1))
 
def getRefOAndOri(posi, use_s2 = True):
    ref_o_list = []
    ref_ori_list = []
    
    # get the reference origin and orientation
    ref_pose = posi[0,:,:]#4x3
    ro0,rx0,ry0,rz0 = getOriginAndOrientation(ref_pose, use_s2 = use_s2)
    # get transform matrix
    M = getRotation(rx0,ry0,rz0)
    T = getTransformation(ro0)
    print('M',M)
    print('T',T)
    
    for i in range(posi.shape[0]):
        ro,rx,_,_ = getOriginAndOrientation(posi[i,:,:], use_s2 = use_s2)
        ref_o, ref_ori = transform(M,T,ro,rx)
        ref_o_list.append(ref_o)
        ref_ori_list.append(ref_ori)

    ref_o = np.stack(ref_o_list)
    ref_ori = np.stack(ref_ori_list)
    return ref_o, ref_ori
def plotDiffHeight(use_s2 = False):
    folder = 'mat/'
    mat9_5 = scipy.io.loadmat(folder+'0delta.mat')
    mat4_5 = scipy.io.loadmat(folder+'49lift.mat')
    mat6_7 = scipy.io.loadmat(folder+'28lift.mat')
    ref_o95, ref_ori95 = getRefOAndOri(mat9_5['poses'], use_s2 = use_s2)
    ref_o45, ref_ori45 = getRefOAndOri(mat4_5['poses'], use_s2 = use_s2) 
    ref_o67, ref_ori67 = getRefOAndOri(mat6_7['poses'], use_s2 = use_s2)


    '''
    # 3D location
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.scatter(ref_o[:,0,0].tolist(),ref_o[:,1,0].tolist(),ref_o[:,2,0].tolist(),'r.',s=0.1)
    '''
    # 2D location
    plt.figure()
    plt.title('Location-x')
    plt.xlabel('x / m')
    plt.ylabel('z / m')
    plt.plot(ref_o45[:,0,0], ref_o45[:,2,0], 'r.', label='h=0.046m')
    plt.plot(ref_o67[:,0,0], ref_o67[:,2,0], 'g.', label='h=0.067m') 
    plt.plot(ref_o95[:,0,0], ref_o95[:,2,0], 'b.', label='h=0.095m') 
    plt.legend()
    
    # elivation
    plt.figure()
    plt.title('Elivation Angle-x')
    plt.xlabel('x / m')
    plt.ylabel('elivation angle / degree')
    plt.plot(ref_o45[:,0,0],np.arccos(ref_ori45[:,0,0]/np.sqrt(ref_ori45[:,0,0]**2+ref_ori45[:,2,0]**2))/np.pi*180,'r.',label='h=0.046m')
    plt.plot(ref_o67[:,0,0],np.arccos(ref_ori67[:,0,0]/np.sqrt(ref_ori67[:,0,0]**2+ref_ori67[:,2,0]**2))/np.pi*180,'g.',label='h=0.067m') 
    plt.plot(ref_o95[:,0,0],np.arccos(ref_ori95[:,0,0]/np.sqrt(ref_ori95[:,0,0]**2+ref_ori95[:,2,0]**2))/np.pi*180,'b.',label='h=0.095m') 
    plt.legend()
    plt.show()
              
def plotd(name='0delta'):
    folder = 'mat/'
    mat = scipy.io.loadmat(folder+name+'.mat')
    ref_o, ref_ori = getRefOAndOri(mat['poses'])
    pose_time = mat['poses_time']

    param = mat['params']
    param_time = mat['params_time']
    plt.title('d-t')
    plt.xlabel('time')
    plt.ylabel('d / m')
    plt.ylim([0,0.4]) 
    plt.plot(pose_time[0,:],0.4-ref_o[:,0,0],'b.',label='tracked')  
    plt.plot(param_time[0,:], param[:,0],'r.',label='target')
    plt.legend()
    plt.show()
def plotAlpha(name='0delta'):
    folder = 'mat/'
    mat = scipy.io.loadmat(folder+name+'.mat')
    param = mat['params']
    param_time = mat['params_time']
    plt.title('alpha-t')
    plt.xlabel('time')
    plt.ylabel('alpha / rad')
    plt.plot(param_time[0,:],param[:,6],'r.',label='current alpha')  
    plt.plot(param_time[0,:], param[:,2],'y.',label='target alpha')
    plt.plot(param_time[0,:],param[:,7],'b.',label='current back alpha')  
    plt.plot(param_time[0,:], param[:,3],'c.',label='target back alpha')
    plt.legend()
    plt.show() 
def plotTheta(name='0delta'):
    folder = 'mat/'
    mat = scipy.io.loadmat(folder+name+'.mat')
    ref_o, ref_ori = getRefOAndOri(mat['poses'])
    pose_time = mat['poses_time']

    param = mat['params']
    param_time = mat['params_time']
    plt.title('Theta-t')
    plt.xlabel('time')
    plt.ylabel('theta / degree')
    plt.ylim([0,50]) 
    plt.plot(pose_time[0,:],np.arccos(ref_ori[:,0,0]/np.sqrt(ref_ori[:,0,0]**2+ref_ori[:,2,0]**2))/np.pi*180,'b.',label='tracked')
    plt.plot(param_time[0,:], param[:,4]/np.pi*180,'r.',label='target')
    plt.legend()
    plt.show()
def plotDiffAngle(use_s2 = False):
    folder='./mat/'
    mat_name_list = [str(i)+'delta' for i in range(9)]
    mat_list = [scipy.io.loadmat(folder+mat_name+'.mat') for mat_name in mat_name_list]

    ref_o_list, ref_ori_list = {},{}
    for i, mat in enumerate(mat_list):
        ref_o, ref_ori = getRefOAndOri(mat['poses'],use_s2 = use_s2)
        ref_o_list[i] = ref_o
        ref_ori_list[i] = ref_ori

    # 3D location
    fig = plt.figure() 
    ax = Axes3D(fig)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_ylim([-0.03,0.03])
    for i in range(9):
        ref_o = ref_o_list[i]
        ax.scatter(ref_o[:,0,0].tolist(),ref_o[:,1,0].tolist(),ref_o[:,2,0].tolist(),s=0.1, label='Angle=%d deg'%(i*5))
    plt.legend()
    plt.show() 
def plotRealTargetLoc(filename='test1', use_s2 = False):
    '''
       plot the OptTrack position and target position into the robot's coordinate system
    '''
    folder='./mat/'
    mat_name = filename
    mat = scipy.io.loadmat(folder+mat_name+'.mat')

    #real robot pose from opttrack coordinate to robot's coordinate
    r_center_posi = mat['poses'][:,0,:]-mat['poses'][:1,0,:]
    r_center_posi[:,:2] = -r_center_posi[:,:2]


    #get bad point
    rolls = mat['poses'][:,1,2]
    yaws = mat['poses'][:,1,0]
    pitchs = mat['poses'][:,1,1]

    yaw_pos_bad = yaws>np.pi/2
    yaw_neg_bad = yaws<-np.pi/2 

    yaws[yaw_pos_bad] -= np.pi
    yaws[yaw_neg_bad] += np.pi
    pitchs[yaw_pos_bad] *= -1
    pitchs[yaw_neg_bad] *= -1
    rolls[yaw_pos_bad] *= -1
    rolls[yaw_neg_bad] *= -1

    roll_pos_bad = rolls>np.pi/2
    roll_neg_bad = rolls<-np.pi/2

    rolls[roll_pos_bad] -= np.pi
    rolls[roll_neg_bad] += np.pi

    pitchs *= -1
    rolls *= -1
 

    S_middle2s = []
    for i in range(r_center_posi.shape[0]):
        p_loc = r_center_posi[i,:]
        #yaw,pitch,roll = mat['poses'][i,1,:]
        yaw,pitch,roll = yaws[i], pitchs[i], rolls[i]
        S_middle2s.append(get_Smiddle2(p_loc,yaw+np.pi,pitch,LEN_S2S1/2))
    S_middle2s = np.stack(S_middle2s)
    S_middle2s -= S_middle2s[:1,:]
    S_middle2s[:,2] += WHEEL_RADIUS

    #robot's pose from image frame to robot coordinate
    Target_middle2s = np.zeros((mat['params'].shape[0],3))
    Target_middle2s[:,0] = ((mat['params'][:,0]+mat['params'][:,1]) - 2* ORIGIN_[0]) * PIXEL_RESO / 2 * -1
    Target_middle2s[:,1] = ((mat['params'][:,2]+mat['params'][:,3]) - 2* ORIGIN_[1]) * PIXEL_RESO / 2 * -1
    Target_middle2s[:,2] = (mat['params'][:,4]+mat['params'][:,5])/2
    
    #1. 3D location
    fig = plt.figure() 
    ax = Axes3D(fig)
    ax.set_xlabel('x',fontsize=20)
    ax.set_ylabel('y',fontsize=20)
    ax.set_zlabel('z',fontsize=20)
    ax.set_ylim([-0.1,0.1])
    ax.view_init(elev=30., azim=270)
    ax.scatter(S_middle2s[:,0].tolist(),S_middle2s[:,1].tolist(),S_middle2s[:,2].tolist(),s=0.5, label='Real robot loc')
    ax.scatter(Target_middle2s[:,0].tolist(),Target_middle2s[:,1].tolist(),Target_middle2s[:,2].tolist(),s=0.5,color='red',label='Target robot loc') 
    plt.legend(prop={'size': 18})
    plt.savefig('im/'+filename+'_loc.png')
#    plt.show() 

    # 2. yaw, pitch, roll
    # time-yaw
    fig = plt.figure() 
    plt.xlabel('time-t',fontsize=20)
    plt.ylabel('yaw',fontsize=20) 
    plt.ylim([-np.pi/2,np.pi/2])
    plt.plot(mat['poses_time'][0,:].tolist(), yaws.tolist(), '.b', label='Real robot yaw')  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,6].tolist(), '.r', label='Target robot yaw')  
    plt.legend(prop={'size': 18})
    plt.savefig('im/'+filename+'_yaw.pdf')
#    plt.show()   

    # time-pitch
    fig = plt.figure() 
    plt.xlabel('time-t',fontsize=20)
    plt.ylabel('pitch',fontsize=20)
    plt.ylim([-np.pi/2,np.pi/2])
    plt.plot(mat['poses_time'][0,:].tolist(), pitchs.tolist(), '.b', label='Real robot pitch')
    plt.plot(mat['params_time'][0,:].tolist(),(mat['params'][:,7]*-1).tolist(), '.r', label='Target robot pitch') 
    plt.legend(prop={'size': 18})
    plt.savefig('im/'+filename+'_pitch.pdf')
#    plt.show()  


    # time-roll
    fig = plt.figure() 
    plt.xlabel('time-t',fontsize=20)
    plt.ylabel('roll',fontsize=20) 
    plt.ylim([-np.pi/2,np.pi/2])
    plt.plot(mat['poses_time'][0,:].tolist(), rolls.tolist(), '.b', label='Real robot roll')  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,8].tolist(), '.r', label='Target robot roll')  
    plt.legend(prop={'size': 18})
    plt.savefig('im/'+filename+'_roll.pdf')
#    plt.show()  

    # 3. flippers
    fig = plt.figure() 
    plt.xlabel('time-t',fontsize=20)
    plt.ylabel('angle-rad',fontsize=20) 
    #plt.ylim([-np.pi/2,np.pi/2])
    plt.plot(mat['params_time'][0,:].tolist(), mat['params'][:,13].tolist(), 'r^', label='Real robot left alpha', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,9].tolist(), 'rs', label='Target robot left alpha', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(), mat['params'][:,14].tolist(), 'b^', label='Real robot right alpha', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,10].tolist(), 'bs', label='Target robot right alpha', markersize=.5, alpha=.5)   
 

    # the beta should +0.06981317007977318 for in optfollowing.py there's a -0.06981317007977318 to calibrate the robot angle
    plt.plot(mat['params_time'][0,:].tolist(), (mat['params'][:,15]+0.06981317007977318).tolist(), 'y^', label='Real robot left beta', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,11].tolist(), 'ys', label='Target robot left beta', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(), (mat['params'][:,16]+0.06981317007977318).tolist(), 'm^', label='Real robot right beta', markersize=.5, alpha=.5)  
    plt.plot(mat['params_time'][0,:].tolist(),mat['params'][:,12].tolist(), 'ms', label='Target robot right beta', markersize=.5, alpha=.5)   
    plt.legend(prop={'size': 18})
    plt.savefig('im/'+filename+'_flipper.png') 
     
         
if __name__ == '__main__':
    '''
    plotAlpha()
    plotAlpha('28lift') 
    plotAlpha('49lift') 
    plotd()
    plotd('28lift')
    plotd('49lift') 
    plotTheta()
    plotTheta('28lift')
    plotTheta('49lift')
    plotDiffHeight(use_s2 = False)
    plotDiffAngle(use_s2 = False)
    '''
    arena_names=['step','ramp','iramp']
    angs = [i*5 for i in range(8)]

    for an in arena_names:
        for ag in angs:
            try:
                print(an,ag)
                plotRealTargetLoc(an+'_'+str(ag))
            except:
                print('not find')

