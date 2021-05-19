import matplotlib.pyplot as plt
import numpy as np
from flipper25d.path_search import *  
from flipper25d.vis.vis import draw_arena, get_S1, get_S0 
def draw_posi(pt):
    p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha, r_alpha, l_beta, r_beta = pt

    p_S2_l = p_S2_l.tolist()
    p_S2_r = p_S2_r.tolist()
    
    mid = [(p_S2_l[0]+p_S2_r[0])/2,(p_S2_l[1]+p_S2_r[1])/2]
 
if __name__ == '__main__':
    mat = scipy.io.loadmat('../expanded_config_path.mat')
    path_lst = mat['path'].tolist()
    path_lst = [[l[0][0], l[1][0], l[2][0][0], l[3][0][0], l[4][0][0], l[5][0][0], l[6][0][0], l[7][0][0], l[8][0][0]]  for l in path_lst]
    #path_lst = path_lst[::-1]
    show_im = exp_map.copy()
    plt.figure()
 
    for pt in path_lst:

        p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha, r_alpha, l_beta, r_beta = pt

        p_S2_l = p_S2_l.tolist()
        p_S2_r = p_S2_r.tolist()
    
        mid = [int((p_S2_l[0]+p_S2_r[0])/2),int((p_S2_l[1]+p_S2_r[1])/2)]

        show_im[mid[0]-3:mid[0]+4, mid[1]-3:mid[1]+4] = 1.
     
    plt.imshow(show_im)
    plt.draw()
    plt.pause(.1)
     
    for pt in path_lst:
        show_pt = show_im.copy()

        p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha, r_alpha, l_beta, r_beta = pt

        p_S2_l = p_S2_l.tolist()
        p_S2_r = p_S2_r.tolist()
    
        mid = [int((p_S2_l[0]+p_S2_r[0])/2),int((p_S2_l[1]+p_S2_r[1])/2)]

        show_pt[mid[0]-15:mid[0]+16, mid[1]-15:mid[1]+16] = 1.
         
        plt.imshow(show_pt)
        plt.draw()
        plt.pause(.1)
