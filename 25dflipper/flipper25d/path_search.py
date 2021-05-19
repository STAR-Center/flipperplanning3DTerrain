import numpy as np
from flipper25d.config import *
from flipper25d.util import *
from flipper25d.cspace.cspace import *
from flipper25d.vis.vis import rot_vector
import pdb

from scipy.misc import imsave

import sys
import os 


'''
    we use the left side S2 as the pivot
        right side S2 can be uniquely defined with negative_y direction on robo coordinate system

    get neighbour: get_neighbours
    get constrained point: right_S2
    get cost: cost_fun
    DFS path search: get_path_store and retrieve_path

    t <- (p_, yaw, pitch, roll, alpha)
    neighbour is (t, t_r)
    q is (neighbour, cost, store_id)

'''


global touched

def right_S2(p_, yaw, pitch, roll):
    v_oRS2 = [0.,WIDTH,0.]
    R_o_y = rot_vector([0.,0.,1.],-yaw)
    R_o_p = rot_vector([np.sin(yaw),np.cos(yaw),0.],-pitch)
    R_o_r = rot_vector([np.cos(yaw)*np.cos(pitch),-np.sin(yaw)*np.cos(pitch),np.sin(pitch)], -roll)
    v_oRS2_ = np.dot(R_o_r,np.dot(R_o_p,np.dot(R_o_y,np.array(v_oRS2).reshape((3,1)))))
    p_oRS2_ = [p_[0]-int(v_oRS2_[0,0]/PIXEL_RESO),p_[1]-int(v_oRS2_[1,0]/PIXEL_RESO), v_oRS2_[2,0]+p_[2]]
    return p_oRS2_

def have_same_pitch(pitchs1, pitchs2):
    '''the version pitchs is a list of possible pitchs
    paired_list = []
    for i in range(len(pitchs1)):
        for j in range(len(pitchs2)):
            if (pitchs1[i] == pitchs2[j]):
                paired_list.append([i,j])
    if len(paired_list) == 0:
        return None
    else:
        return paired_list[0]#later maybe have a rank
    '''

    #this version pitchs is either [pitch] or [pitch_ub, pitch_lb]
    if len(pitchs1) == 0 or len(pitchs2) == 0:
        return None
    elif len(pitchs1) == 1 and len(pitchs2) == 1:
        if pitchs1[0] == pitchs2[0]:
            return pitchs1[0]

    elif len(pitchs1) == 1 or len(pitchs2) == 1:
        if len(pitchs1) == 1:
            if pitchs1[0] >= pitchs2[1] and pitchs1[0] <= pitchs2[0]:
                return pitchs1[0]
            else:
                return None
        else:
            if pitchs2[0] >= pitchs1[1] and pitchs2[0] <= pitchs1[0]:
                return pitchs2[0]
            else:
                return None 
    else:#both len==2
        #use its smallest intersection
        if pitchs1[1] > pitchs2[0] or pitchs1[0] < pitchs2[1]:
            return None
        else:
            return min(pitchs1[1],pitchs2[1])





def cost_fun(t_ori, neiblr, p_tgt_):#ano_t_ori,p_tgt_):
    '''
        t <- (p_, yaw, pitch, roll, alpha)
        the cost of one point algo related to its ancester


        #actually only use p_, yaw, pitch
    '''
    neibl,neibr = neiblr
    yaw,pitch = neibl[1],neibl[2]
    ano_t_ori = S2_to_middle(neibl, neibr)[0] 

    diff_height = (exp_map[ano_t_ori[0],ano_t_ori[1]]-ano_t_ori[2])**2
    for i in range(10):
        p_center = get_point(ano_t_ori, yaw=yaw, pitch=pitch, line_len=LEN_S1S2/10*(i+1))
        diff_height += (exp_map[p_center[0],p_center[1]]-p_center[2])**2

    #pitch_dist = neibl[2] ** 2

    #return dist_ano+diff_height
    return diff_height# + pitch_dist
#diff*a1+dist*a2 + ang_diff


def check_point_on_ground(p_):
    if (exp_map[p_[0],p_[1]]+HEIGHT_EPSILON > p_[2]):
        return True
    else:
        return False

#def get_neighbours(p_,yaw,pitch,roll,alpha):
def get_neighbours(t_l,t_r,p_tgt_):
    '''
        here the naming rule:
            p_ is for the current point
            ano_p_ is for the neighbour piint

            p_/ano_p_ r is for the right side

            p_tgt_ is used to compute the cost


        note: following find_valid_neighbour_points.jpeg to find neighbour
    '''
    global touched
    p_l_,yaw,pitch,roll,_ = t_l
    p_r_ = t_r[0]

    p_l_S1 = get_point(p_l_, yaw=yaw, pitch=pitch, line_len=LEN_S1S2)
    p_r_S1 = get_point(p_r_, yaw=yaw, pitch=pitch, line_len=LEN_S1S2)

    center = [(p_l_[0]+p_r_[0]+p_l_S1[0]+p_r_S1[0])/4,\
            (p_l_[1]+p_r_[1]+p_l_S1[1]+p_r_S1[1])/4,\
            (p_l_[2]+p_r_[2]+p_l_S1[2]+p_r_S1[2])/4]

    norm_robo_x = (np.cos(yaw)*np.cos(pitch),-np.sin(yaw)*np.cos(pitch),np.sin(pitch))
    norm_z = np.array((-np.cos(yaw)*np.sin(pitch),np.sin(yaw)*np.sin(pitch),np.cos(pitch)))
    R_roll = rot_vector(norm_robo_x, roll)
    rolled_norm_z = np.dot(R_roll,norm_z.reshape((3,1)))[:,0]

    v_o_p_l_ = [-(p_l_[0] - center[0])*PIXEL_RESO, -(p_l_[1] - center[1])*PIXEL_RESO,p_l_[2] - center[2]]



    #check if four points on the ground
    four_point_on_ground_tag = False
    if check_point_on_ground(p_l_) and check_point_on_ground(p_r_) and check_point_on_ground(p_l_S1) and check_point_on_ground(p_r_S1):
        four_point_on_ground_tag = True

    #p_r_ = right_S2(p_, yaw, pitch, roll)
    neighbours = []
    #costs = []
    neib_id = 0
    for y in YAWS:
        turn_left = None
        if y != yaw:
            turn_left = True
            continue#now only allows for go straight
        if turn_left is None:
            dr,dc = int(-np.cos(yaw)*GO_STRAIGHT_ITS), int(np.sin(yaw)*GO_STRAIGHT_ITS)
        else:
            if not four_point_on_ground_tag:
                continue
            else:
                R_y = rot_vector(rolled_norm_z, yaw - y)
                '''
                if turn_left:

                    dr,dc = int(-np.cos(yaw)*WIDTH/PIXEL_RESO), int(np.sin(yaw)*WIDTH/PIXEL_RESO)
                else:
                    dr,dc = int(-np.cos(yaw)*WIDTH/PIXEL_RESO), int(np.sin(yaw)*WIDTH/PIXEL_RESO)
                '''
                roted_v = np.dot(R_y, np.array(v_o_p_l_).reshape((3,1)))[:,0]
                dr, dc, dh = -(roted_v[0]/PIXEL_RESO), -(roted_v[1]/PIXEL_RESO), roted_v[2]

        for h_it in range(4):
            #use both left and right S2 as pivot
            for on_left_Pivot in [True,False]:
                if turn_left is not None:
                    if h_it > 0 or not four_point_on_ground_tag:
                        continue
                    ano_p_ = [int(center[0]+dr),int(center[1]+dc),center[2]+dh]
                else:
                    d_h = h_it * H_ITS
             
                    if on_left_Pivot:
                        ano_p_ = [p_l_[0]+dr,p_l_[1]+dc,exp_map[p_l_[0]+dr,p_l_[1]+dc]+d_h]
                        p_ = p_l_  
                    else:
                        ano_p_ = [p_r_[0]+dr,p_r_[1]+dc,exp_map[p_r_[0]+dr,p_r_[1]+dc]+d_h]
                        p_ = p_r_ 

                pitchs = get_pitch(ano_p_, yaw)

                #t = [p_, y, pitch, 0.,0.]
                for pt in pitchs:
                    # check pitch
                    if pt < PITCH_LB or pt > PITCH_UB:
                        continue

                    ano_t = [ano_p_, y, pt, 0., 0.]
                    #cost = cost_fun(t,ano_t,p_tgt_, is_left_side=y<=yaw)#+cost_fun(t_r,ano_t_r,p_tgt_)
                    if turn_left is not None:
                        neighbours.append([ano_t,None])
                    else:
                        if on_left_Pivot:
                            neighbours.append([ano_t, None]) 
                        else:
                            neighbours.append([None, ano_t])
                            #costs.append(cost)
    return neighbours#, costs

def middle_to_S2(t_ori):
    p_, yaw, pitch,roll,_ = t_ori

    v_oLS2 = [0.,WIDTH/2,0.]
    v_oRS2 = [0.,-WIDTH/2,0.] 
    R_o_y = rot_vector([0.,0.,1.],-yaw)
    R_o_p = rot_vector([np.sin(yaw),np.cos(yaw),0.],-pitch)
    R_o_r = rot_vector([np.cos(yaw)*np.cos(pitch),-np.sin(yaw)*np.cos(pitch),np.sin(pitch)], -roll)
    v_oLS2_ = np.dot(R_o_r,np.dot(R_o_p,np.dot(R_o_y,np.array(v_oLS2).reshape((3,1))))) 
    v_oRS2_ = np.dot(R_o_r,np.dot(R_o_p,np.dot(R_o_y,np.array(v_oRS2).reshape((3,1)))))
    p_oRS2_ = [p_[0]-int(v_oRS2_[0,0]/PIXEL_RESO),p_[1]-int(v_oRS2_[1,0]/PIXEL_RESO), v_oRS2_[2,0]+p_[2]]
    p_oLS2_ = [p_[0]-int(v_oLS2_[0,0]/PIXEL_RESO),p_[1]-int(v_oLS2_[1,0]/PIXEL_RESO), v_oLS2_[2,0]+p_[2]] 
    return [p_oLS2_, yaw,pitch,roll,0.], [p_oRS2_, yaw,pitch,roll,0.]
 
def S2_to_middle(neibl, neibr):
    middle = [int((neibl[0][0]+neibr[0][0])/2),int((neibl[0][1]+neibr[0][1])/2)]
    return [[middle[0],middle[1],(neibl[0][2]+neibr[0][2])/2], neibl[1], neibl[2], neibl[3], neibl[4]]
def reach_target(middle, p_tgt_):
    p_ = middle
    if (p_[0] - p_tgt_[0])**2 <= 100:
        return True
    else:
        return False

def get_path_store(t_ori, p_tgt_):
    '''
        from ORIGIN to TARGET on arena man

        Note: here when we compute the path, only x,y,z,yaw,pitch will be considered.

        input: t_ori, p_tgt_

        Detail
            1. given xyz(middle),yaw,pitch,roll
            2. find a next step xyz,yaw,pitch, then check if roll valid


        Note: the yaw is fixed, p_tgt_ is only used to check reach target
    '''

    global touched
    t_ori_l, t_ori_r = middle_to_S2(t_ori)
    #p_ori_r = right_S2(t_ori[0],t_ori[1],t_ori[2],t_ori[3])
    '''
    if t_ori_r is None:#it should be origin
        Q = [[(t_ori,(p_ori_r,t_ori[1],t_ori[2],t_ori[3],t_ori[4])),0]]#in Q, it will remember its store_id
    else:
    '''
    Q = [(t_ori_l,t_ori_r)]
    store = [Q[0]]#in store, it will remember its ancester's store_id

    reach_target_tag = False
    before_last = 0

    while len(Q)>0:
        q = Q.pop()
        t_ = q
        print(t_[0],'\t\t\t',t_[1])

        t_l, t_r= t_
        t_middle = S2_to_middle(t_l,t_r)
        neighbours= get_neighbours(t_l,t_r,p_tgt_)
        #ids = np.argsort(costs)#cost from small to large
        if len(neighbours) == 0:
            print('no neighbour')
            continue

        nbso = []
        costs = []
        for i in range(len(neighbours)):
            neibl,neibr = neighbours[i]
            if neibl is None:
                roll, _ = get_roll(neibr[0],neibr[1],neibr[2],False)
                if roll is None:
                    continue
                ano_p_ = get_point(neibr[0], yaw=neibr[1]-np.pi/2, pitch=roll, line_len=WIDTH)
                neibl = [ano_p_, neibr[1],neibr[2], roll, 0.]
                neibr[3] = roll
            else:
                roll, _ = get_roll(neibl[0],neibl[1],neibl[2],True)
                if roll is None:
                    continue
                ano_p_ = get_point(neibl[0], yaw=neibl[1]+np.pi/2, pitch=-roll, line_len=WIDTH)
                neibr = [ano_p_, neibl[1],neibl[2], roll, 0.]
                neibl[3] = roll 
            #cost = cost_fun(t_middle, S2_to_middle(neibl,neibr),p_tgt_)

            cost = cost_fun(t_middle, (neibl,neibr),p_tgt_)
            #print(cost, neibl,neibr)
            
            nbso.append([neibl,neibr])
            costs.append(cost)

        ids = np.argsort(costs)

        nbs = [nbso[i] for i in ids[::-1]]

        middle = int((nbs[-1][0][0][0]+nbs[-1][1][0][0])/2),int((nbs[-1][0][0][1]+nbs[-1][1][0][1])/2)
        #check if the smallest cost is target
        Q = [nbs[-1]]
        store.append(nbs[-1])
        if reach_target(middle, p_tgt_):
            store.append(nbs[-1])
            return store 
    return store
def retrieve_path(store):
    path = store
    return path




if __name__ == '__main__':
    t_ori = (ORIGIN_, 0., 0., 0., DEFAULT_ALPHA)
    entire_path = []
    p_tgt_ = TARGET_#[1512,800] 
    print('######            From ', t_ori, ' to ', p_tgt_, '#################') 
    store = get_path_store(t_ori, p_tgt_)
    entire_path = retrieve_path(store)                                      
    import scipy.io
    scipy.io.savemat('exp/conf_path/'+ARENA_NAME+'config_path.mat',{'path':np.array(entire_path)})
    expanded_entire_path = [expand_param(p) for p in entire_path]
    scipy.io.savemat('exp/conf_path/'+ARENA_NAME+'expanded_config_path.mat',{'path':np.array(expanded_entire_path)})




