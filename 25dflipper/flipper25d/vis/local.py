import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from flipper25d.config import *
import numpy as np 
from flipper25d.path_search import * 
from flipper25d.vis.vis import draw_arena, get_S1, get_S0

def draw_skele_line(ax, p_from, p_to):
    ax.plot([-p_from[0]*PIXEL_RESO,-p_to[0]*PIXEL_RESO], [-p_from[1]*PIXEL_RESO,-p_to[1]*PIXEL_RESO],zs=[p_from[2],p_to[2]],linewidth=4.) 

def draw_skele(ax, pt):
    p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha, r_alpha, l_beta, r_beta = pt

    p_S2_l = p_S2_l.tolist()
    p_S2_r = p_S2_r.tolist()
    
    mid = [(p_S2_l[0]+p_S2_r[0])/2,(p_S2_l[1]+p_S2_r[1])/2]
    p_S2_l[0] -= mid[0]
    p_S2_r[0] -= mid[0]
    p_S2_l[1] -= mid[1]
    p_S2_r[1] -= mid[1]

    p_S1_l = get_S1(p_S2_l, yaw,pitch)
    p_S1_r = get_S1(p_S2_r, yaw,pitch)

    p_S0_l = get_S0(p_S1_l, yaw, roll, pitch, l_alpha)
    p_S0_r = get_S0(p_S1_r, yaw, roll, pitch, r_alpha) 

    neg_yaw = yaw-np.pi if yaw > 0 else yaw+np.pi 
    p_S3_l = get_S0(p_S2_l, neg_yaw, -roll, -pitch, l_beta)
    p_S3_r = get_S0(p_S2_r, neg_yaw, -roll, -pitch, r_beta)  

    # S0S1, S1S2, S2S3
    draw_skele_line(ax, p_S1_l, p_S2_l)
    draw_skele_line(ax, p_S2_l, p_S3_l) 
    draw_skele_line(ax, p_S1_l, p_S0_l) 

    draw_skele_line(ax, p_S1_r, p_S2_r)
    draw_skele_line(ax, p_S2_r, p_S3_r) 
    draw_skele_line(ax, p_S1_r, p_S0_r)  

    # SlSr
    draw_skele_line(ax, p_S1_l, p_S1_r)
    draw_skele_line(ax, p_S2_l, p_S2_r)


def plot_local_morphology(local_map, pt, imname, fig):
    '''
       local_map is using the middle as origin
       pt is with (p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha, r_alpha, l_beta, r_beta)
    '''
    #fig = plt.figure()

    ax = Axes3D(fig)
    ax.set_xlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_ylim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_zlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    # 1. draw arena
    draw_arena(ax,local_map)

    # 2. draw skele
    draw_skele(ax,pt)
    ax.view_init(elev=30., azim=270)


    fig.savefig('morpho/'+imname+'.png')
 
if __name__ == '__main__':
    mat = scipy.io.loadmat('../exp/conf_path/'+ARENA_NAME+'expanded_config_path.mat')
    path_lst = mat['path'].tolist()
    path_lst = [[l[0][0], l[1][0], l[2][0][0], l[3][0][0], l[4][0][0], l[5][0][0], l[6][0][0], l[7][0][0], l[8][0][0]]  for l in path_lst]
    #path_lst = path_lst[::-1]

    fig = plt.figure()
    for i,pt in enumerate(path_lst):
        #fig = plt.figure()
        r = int((pt[0][0]+pt[1][0])/2)
        c = int((pt[0][1]+pt[1][1])/2) 
        plot_local_morphology(exp_map[r-WINDOW_SIZE//2:r+WINDOW_SIZE//2+1,c-WINDOW_SIZE//2:c+WINDOW_SIZE//2+1],pt,str(i), fig) 
        print(pt)
        #plt.show()
        plt.draw()
        plt.pause(.05)
        fig.clear()
     
