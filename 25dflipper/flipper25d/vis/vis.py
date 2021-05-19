import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from flipper25d.config import *
import numpy as np
import pdb


def draw_arena(ax,local_map):
    rcdata = np.mgrid[-(WINDOW_SIZE//2):WINDOW_SIZE//2+1:5,-(WINDOW_SIZE//2):WINDOW_SIZE//2+1:5].reshape((2,-1))
    z = local_map[(rcdata+WINDOW_SIZE//2).tolist()]

    #on xyz
    ax.scatter3D(-rcdata[0,:]*PIXEL_RESO,-rcdata[1,:]*PIXEL_RESO,z,c=z,cmap='Greens')
def get_S1(p_,yaw,pitch):
    '''
        from p_, to the orientation of (yaw,pitch) with line_len
    '''
    delta_c = int(np.sin(yaw)*LEN_S2S1/PIXEL_RESO*np.cos(pitch))
    delta_r = int(-np.cos(yaw)*LEN_S2S1/PIXEL_RESO*np.cos(pitch))
    delta_h = LEN_S2S1*np.sin(pitch)
    return [p_[0]+delta_r,p_[1]+delta_c,p_[2]+delta_h]


def rot_x(angle):
    return np.array([[1.,0.,0.],
                    [0.,np.cos(angle),-np.sin(angle)],
                    [0.,np.sin(angle),np.cos(angle)]])
def rot_y(angle):
    return np.array([[np.cos(angle),0.,np.sin(angle)],
                    [0.,1.,0.],
                    [-np.sin(angle),0.,np.cos(angle)]]) 
def rot_z(angle):
    return np.array([[np.cos(angle),-np.sin(angle),0.],
                    [np.sin(angle),np.cos(angle),0.],
                    [0.,0.,1.]]) 

def rot_vector(u, phi):
    '''
        Rodrigues rotation formula
        follow:
            https://math.stackexchange.com/questions/142821/matrix-for-rotation-around-a-vector
    '''
    W = np.array([[0.,-u[2],u[1]],
                [u[2],0.,-u[0]],
                [-u[1],u[0],0.]])

    R = np.eye(3)+np.sin(phi)*W+(2*np.sin(phi/2)**2)*np.dot(W,W)
    return R


def get_S0(p_S1, yaw, roll, pitch, alpha):
    '''
       vector in xyz

    '''
    #v_S2S1 = [-(p_S1[1]-p_S2[1]),-(p_S1[0]-p_S2[0]),p_S1[2]-p_S2[2]]

    v_oS0 = [LEN_S1S0*np.cos(alpha),0.,LEN_S1S0*np.sin(alpha)]
    # wrong: detail is in doc/rot_transform.jpeg
    #v_oS0_ = np.dot(rot_z(-yaw),np.dot(rot_y(-pitch),np.dot(rot_x(roll),np.array(v_oS0).reshape((3,1)))))

    # correct:rot_x,rot_y,rot_z are all on original coordinate system we should use Rodrigues rotation formula to rotate on certain axis
    R_o_y = rot_vector([0.,0.,1.],-yaw)
    R_o_p = rot_vector([-np.sin(yaw),-np.cos(yaw),0.],pitch)
    R_o_r = rot_vector([np.cos(yaw)*np.cos(pitch),-np.sin(yaw)*np.cos(pitch),np.cos(pitch)], roll)
    v_oS0_ = np.dot(R_o_r,np.dot(R_o_p,np.dot(R_o_y,np.array(v_oS0).reshape((3,1)))))

    p_S0 = [p_S1[0]-int(v_oS0_[0]/PIXEL_RESO), p_S1[1]-int(v_oS0_[1]/PIXEL_RESO),p_S1[2]+v_oS0_[2][0]]

    return p_S0



def draw_skele(ax, p_h, yaw,pitch,roll,alpha):
    p_S2 = [0.,0.,p_h]
    p_S1 = get_S1(p_S2, yaw,pitch)
    p_S0 = get_S0(p_S1, yaw, roll, pitch, alpha)
    #print(p_S2,p_S1,p_S0)
    #plot is on xyz
    ax.plot([-p_S2[0]*PIXEL_RESO,-p_S1[0]*PIXEL_RESO], [-p_S2[1]*PIXEL_RESO,-p_S1[1]*PIXEL_RESO],zs=[p_S2[2],p_S1[2]])
    ax.plot([-p_S1[0]*PIXEL_RESO,-p_S0[0]*PIXEL_RESO],[-p_S1[1]*PIXEL_RESO,-p_S0[1]*PIXEL_RESO],zs=[p_S1[2],p_S0[2]])


def plot_morphology(local_map,p_h, yaw,pitch,roll, alpha, imname='tmp'):
    '''
        plot on rc

        p_ is on the center of this local_map with size [WINDOW_SIZE, WINDOW_SIZE]
        reference from: 
            https://jakevdp.github.io/PythonDataScienceHandbook/04.12-three-dimensional-plotting.html
    '''

    #ax = plt.axes(projection='3d')
    fig = plt.figure()

    ax = Axes3D(fig)
    ax.set_xlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_ylim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_zlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    # 1. draw arena
    draw_arena(ax,local_map)

    # 2. draw skele
    draw_skele(ax,p_h,yaw,pitch,roll,alpha)


    fig.savefig('morpho/'+imname+'.png')

