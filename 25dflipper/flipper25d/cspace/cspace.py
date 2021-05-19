import numpy as np
import matplotlib.pyplot as plt
from flipper25d.config import *
from flipper25d.util import line_trace, get_plane
from flipper25d.vis.vis import plot_morphology, rot_vector
import scipy.io
import pdb 
'''
    the parameter space is (x,y,z,yaw,roll,alpha)


   p(x,y,z), r(yaw, pitch, roll)

   with yaw \in [0,2pi]
        pitch \in (-pi/2,pi/2)
        roll \in [-pi/4, pi/4]


        the yaw we use it the negative direction of the right hand


        in this implementation, our variable yaw, pitch is opposite to the right hand angle
                                            roll is on the right hand angle

   coordinate system:
            ^x
            |
            |    p                   p_
            |
    <-------|                      ------> c
    y        -                     | -
               -                   |   -
                - z                |     - h
                                   |r

'''
global exp_map
#mat = scipy.io.loadmat('/home/yijun/Documents/roboGit/25dflipper/flipper25d/step/step_0.mat')   #man4.mat')
#mat = scipy.io.loadmat('/Users/yijun/Documents/roboGitLab/25dflipper/flipper25d/exp/dilated_map/'+ARENA_NAME+'.mat')
try:
    mat = scipy.io.loadmat('exp/dilated_map/'+ARENA_NAME+'.mat')
except:
    mat = scipy.io.loadmat('../exp/dilated_map/'+ARENA_NAME+'.mat')
    print('call cspace in subfolder')
exp_map = mat['exp_map'] 

def loc_to_pixelLoc(p):
    '''
       p to p_
    '''
    return [int(-p[1]/PIXEL_RESO)+ORIGIN_[0],\
            int(-p[0]/PIXEL_RESO)+ORIGIN_[1],\
            p[2]]


def on_ground(p_):
    return True if p_[2] == exp_map[p_[0],p_[1]] else False

def get_point(p_,yaw,pitch,line_len):
    '''
        from p_, to the orientation of (yaw,pitch) with line_len
    '''
    delta_c = int(np.sin(yaw)*line_len/PIXEL_RESO*np.cos(pitch))
    delta_r = int(-np.cos(yaw)*line_len/PIXEL_RESO*np.cos(pitch))
    delta_h = line_len*np.sin(pitch)
    #on the h, it is possible the h is little smaller than map h, then we use the map h
    return [p_[0]+delta_r,p_[1]+delta_c,max(exp_map[p_[0]+delta_r,p_[1]+delta_c],p_[2]+delta_h)]



def check_on_flat_ground(p_,yaw,line_len):
    p = p_
    line_ids = line_trace(yaw, line_len)
    pitch_list = [np.arctan((exp_map[p[0]+ids[0], p[1]+ids[1]]-exp_map[p[0],p[1]])/ \
            np.sqrt((ids[0]*PIXEL_RESO)**2+(ids[1]*PIXEL_RESO)**2)) \
            for ids in line_ids]
    return pitch_list[0] if len(set(pitch_list)) == 1 else None

def find_largest_angle(p_, yaw, line_len, roll, pitch = None):
    '''
       p_ is (r,c,h)

       return the largest pitch and the pixel_interval

       note: roll is zero in this case
    '''
    global exp_map

    p = p_
    if pitch is None or abs(roll) < 0.087:# to find pitch if roll < 5 deg, we consider it is 0
        #just simply find the line and trace back to find largest pitch
        line_ids = line_trace(yaw, line_len)
        pitchs = []
        for ids in line_ids:
            another_p = [p[0]+ids[0],p[1]+ids[1],0]
            another_p[2] = exp_map[another_p[0],another_p[1]]
            if (ids[0]*PIXEL_RESO)**2+(ids[1]*PIXEL_RESO)**2+(another_p[2]-p[2])**2 <= line_len**2:
                pitch1 = np.arctan((another_p[2]-p[2])/np.sqrt((ids[0]*PIXEL_RESO)**2+(ids[1]*PIXEL_RESO)**2))#positive or negative
                pitchs.append(pitch1)
            else:
                #if one point not reachable, then points behind also not reachable
                #on this point touched the wall of it
                if (another_p[2]>p[2]):
                    planar_dist = np.sqrt((ids[0]*PIXEL_RESO)**2+(ids[1]*PIXEL_RESO)**2)
                    if planar_dist<= line_len:
                        pitch1 = np.arccos(planar_dist / line_len)
                        #print(pitch)
                        pitchs.append(pitch1)
                break
        if len(pitchs) == 0:
            if pitch is not None:
                return FLIPPER_ANGLE_LB,None
            else:
                return None,None
        largest_id = len(pitchs)-1
        for i in range(len(pitchs)-1):
            if pitchs[i] > pitchs[largest_id]:
                largest_id = i
        if pitch is not None:# we solve alpha
            return pitchs[largest_id] - pitch, line_ids[largest_id]
        else:
            return pitchs[largest_id], line_ids[largest_id]
    else:# find alpha
        #roll != 0, it is for S1S0, the alpha
        #more complicated. it is given S1, S2S1 have roll, find angle on flipper S1S0
        #note:  we have a circle with radius line_len on xy planar.
        #       so with a roll on S2S1, flipper range is not verticle. it is a inclined plane
        #detail in doc/get_angle.jpeg
        #1. for each p in this circle, compute its z on the inclined plane. if z > p[2], ignore it. if dist > line_len, ignore it.
        plane_list = get_plane(line_len)
        #before roll
        norm_robo_x = (np.cos(yaw)*np.cos(pitch),-np.sin(yaw)*np.cos(pitch),np.sin(pitch))#on the eucliean coordinate system
        norm_robo_neg_y = (-np.sin(yaw),-np.cos(yaw),0.)
        norm_robo_z = (-np.cos(yaw)*np.sin(pitch),np.sin(yaw)*np.sin(pitch),np.cos(pitch))
        #then roll
        R_roll = rot_vector(norm_robo_x, roll)
        norm_robo_neg_y = np.dot(R_roll, np.array(norm_robo_neg_y).reshape((3,1)))[:,0]
        norm_robo_z = np.dot(R_roll, np.array(norm_robo_z).reshape((3,1)))[:,0]

        plane_h_list = [(-cpt[0]*PIXEL_RESO*norm_robo_neg_y[0]-cpt[1]*PIXEL_RESO*norm_robo_neg_y[1])/(-norm_robo_neg_y[2]) for cpt in plane_list]

        valid_id = []
        for i in range(len(plane_h_list)):
            '''
                add all of the point that intersect with disc and then find the biggest pi
            '''
            dist_p_to_center = (plane_list[i][0]*PIXEL_RESO)**2+(plane_list[i][1]*PIXEL_RESO)**2+plane_h_list[i]**2
            if (dist_p_to_center <= line_len**2):
                if (plane_h_list[i]+p[2]<=exp_map[plane_list[i][0]+p[0],plane_list[i][1]+p[1]]+EPSILON_H_PLANE):
                    valid_id.append(i)
                else:
                    #compute the possible S0, find if it is smaller than S0
                    scale_rate = line_len/np.sqrt(dist_p_to_center)
                    r0 = int(scale_rate * plane_list[i][0])
                    c0 = int(scale_rate * plane_list[i][1])
                    h0 = scale_rate* plane_h_list[i]
                    if (h0+p[2]<=exp_map[r0+p[0],c0+p[1]]+EPSILON_H_PLANE): 
                        valid_id.append(i)
        #valid_id = [i if (plane_h_list[i]+p[z]<=exp_map[plane_list[i][0]+p[0],plane_list[i][1]+p[1]]) and ((plane_list[i][0]*PIXEL_RESO)**2+(plane_list[i][1]*PIXEL_RESO)**2+plane_h_list[i]**2 <= line_len**2) for i in range(len(plane_h_list))]
        if len(valid_id) == 0:
            #return None, None
            return FLIPPER_ANGLE_LB,None
        #2. for the left point, find the point with largest angle.
        angles = []
        ids = []
        for i in valid_id:
            numerator = (norm_robo_x[0]*(-plane_list[i][0]*PIXEL_RESO)+norm_robo_x[1]*(-plane_list[i][1]*PIXEL_RESO)+norm_robo_x[2]*plane_h_list[i])
            #numerator = (norm_robo_z[0]*(-plane_list[i][0]*PIXEL_RESO)+norm_robo_z[1]*(-plane_list[i][1]*PIXEL_RESO)+norm_robo_z[2]*plane_h_list[i]) 
            denominator = (np.sqrt((plane_list[i][0]*PIXEL_RESO)**2+(plane_list[i][1]*PIXEL_RESO)**2+plane_h_list[i]**2)+EPSILON_DIV)
            #angle = np.arcsin(numerator/denominator)

            angle = np.arccos(numerator/denominator)
            if angle > np.pi/2:
                continue
            else:
                if (norm_robo_z[0]*(-plane_list[i][0]*PIXEL_RESO)+norm_robo_z[1]*(-plane_list[i][1]*PIXEL_RESO)+norm_robo_z[2]*plane_h_list[i]) < 0:
                    angle = -angle
            ids.append(i)
            angles.append(angle)
        '''
        angles = [np.arccos(\
                norm_robo_x[0]*plane_list[i][0]*PIXEL_RESO+norm_robo_x[1]*plane_list[i][1]*PIXEL_RESO/ \
                #+norm_robo_x[2]*plane_h_list[i]
                np.sqrt((plane_list[i][0]*PIXEL_RESO)**2+(plane_list[i][1]*PIXEL_RESO)**2+plane_h_list[i]**2) )\
                    for i in valid_id]
        '''
        if len(angles) == 0:
            #return None, None
            return FLIPPER_ANGLE_LB, None
        max_angles_id = np.argmax(angles)
        return angles[max_angles_id], plane_list[valid_id[max_angles_id]]


def get_pitch(p_, yaw):
    '''
       get a range of possible pitch
       if turn right, p_ is right, otherwise is left
    '''
    if on_ground(p_):
        #check if S2S1S0 on flat ground
        on_flat_angle = check_on_flat_ground(p_,yaw,LEN_S2S1+LEN_S1S0)
        #if check_on_flat_ground(p_,yaw,LEN_S2S1+LEN_S1S0):
        if on_flat_angle is not None:
            return [on_flat_angle]
        else:
            #get upperbound
            #check if S2S1 on ground (on the flat ground)
            on_flat_angle = check_on_flat_ground(p_,yaw,LEN_S2S1)
            #if check_on_flat_ground(p_,yaw,LEN_S2S1):
            if on_flat_angle is not None:
                #then should on that angle
                p_S1_lb = get_point(p_,yaw,on_flat_angle,LEN_S2S1)
                pitch_lb = on_flat_angle
                
            else:
                pitch_lb, touch_p = find_largest_angle(p_,yaw,LEN_S2S1,roll=0)
                if pitch_lb is None:
                    return []
                p_S1_lb = get_point(p_,yaw,pitch_lb,LEN_S2S1)
            pitch_ub,touch_p = find_largest_angle(p_,yaw,LEN_S2S1+LEN_S1S0,roll=0)
            if pitch_ub is None:
                return []
            return np.arange(pitch_lb,pitch_ub+PITCH_IT,PITCH_IT).tolist()
    else:
        pitch, touch_p = find_largest_angle(p_,yaw,LEN_S2S1,roll=0)
        if pitch is None:
            return []
        return [pitch]
     
def get_roll(p_,yaw,pitch,is_left_pivot=True):
    norm_S2S1 = np.array([np.cos(pitch)*np.cos(yaw), np.cos(pitch)*(-np.sin(yaw)),np.sin(pitch)])
    norm_neg_y = np.array((-np.sin(yaw),-np.cos(yaw),0.))
    norm_z = np.array((-np.cos(yaw)*np.sin(pitch),np.sin(yaw)*np.sin(pitch),np.cos(pitch)))

    points = get_plane((WIDTH*2+LEN_S1S2**2)**0.5)#get_cylinder_points(is_left_pivot)
    rolls = []
    ids = []
    for i,dp in enumerate(points):
        dr,dc = dp
        try:# it is possible that it will be out of bound
            v_S2t = np.array([-dr*PIXEL_RESO,-dc*PIXEL_RESO,exp_map[p_[0]+dr,p_[1]+dc]-p_[2]])
        except:
            continue
        l_x = np.sum(v_S2t*norm_S2S1)
        if l_x >=0 and l_x<=LEN_S2S1 + WIDTH_EPSILON:
            v_S2t_yz = v_S2t - l_x*norm_S2S1
            d_S2S1_t = np.sqrt(np.sum(v_S2t_yz**2))
            #if d_S2S1_t > WIDTH - WIDTH_EPSILON and d_S2S1_t < WIDTH + WIDTH_EPSILON:
            if d_S2S1_t <= WIDTH + WIDTH_EPSILON and d_S2S1_t >= WIDTH-WIDTH_EPSILON:
                numerater = np.sum(norm_z*v_S2t)
                denominator = np.sum(norm_neg_y*v_S2t)
                if is_left_pivot:
                    if denominator <= 0:
                        continue
                    else:
                        roll = -np.arctan(numerater/denominator)
                else:
                    if denominator >= 0:
                        continue
                    else:
                        roll = np.arctan(numerater/(-denominator))
                #roll = np.arctan(np.sum(norm_z*v_S2t) / np.sum(norm_neg_y*v_S2t) )
                ids.append(i)
                rolls.append(roll)

    if len(rolls) == 0:
        return None, None
    if is_left_pivot:
        min_rolls_id = np.argmin(rolls)
    else:
        min_rolls_id = np.argmax(rolls)
    min_rolls_point_i = ids[min_rolls_id]
    dr,dc = points[min_rolls_point_i]
    p_t_ = [p_[0]+dr,p_[1]+dc,exp_map[p_[0]+dr,p_[1]+dc]]
    if abs(rolls[min_rolls_id]) < 0.01:
        return 0., p_t_
    return rolls[min_rolls_id], p_t_






def gen_param(p_,yaw,is_left_pivot = True):#param for two side
    '''
        p is with [x,y,z]
        p_ is with [r,c,z]
        get p and r, return the parameter that is alpha and beta of the flipper
        p_ is the  pixel_p


        pitch --geometry-> roll --linetrace-> alpha


        we have pitch, but not give it
        return [alpha] or alphas (list)
            if len(alpha) == 2: then the min and max of its range
        
        the detail is in doc/gen_param.jpeg
    '''
    global exp_map
    if on_ground(p_):
        #check if S2S1S0 on flat ground
        on_flat_angle = check_on_flat_ground(p_,yaw,LEN_S2S1+LEN_S1S0)
        #if check_on_flat_ground(p_,yaw,LEN_S2S1+LEN_S1S0):
        if on_flat_angle is not None:
            roll, p_anotherside_ = get_roll_from_twoside(p_,pitch=on_flat_angle, is_left_pivot=is_left_pivot)
            #check if anotherside also ...
            ano_on_flat_angle = check_on_flat_ground(p_anotherside_,yaw,LEN_S2S1+LEN_S1S0)
            if ano_on_flat_angle is not None:
                if ano_on_flat_angle == on_flat_angle:
                    return ([DEFAULT_ALPHA],[on_flat_angle]), ([DEFAULT_ALPHA],[on_flat_angle])

            p_S1_anotherside_ = get_S1()
            alpha_anotherside, touch_p = find_largest_angle(p_S1_anotherside_, yaw, LEN_S1S0, roll=roll, pitch = on_flat_angle)
            return ([DEFAULT_ALPHA],[on_flat_angle]), ([alpha_anotherside],[on_flat_angle])
        else:
            #get upperbound
            #check if S2S1 on ground (on the flat ground)
            on_flat_angle = check_on_flat_ground(p_,yaw,LEN_S2S1)
            #if check_on_flat_ground(p_,yaw,LEN_S2S1):
            if on_flat_angle is not None:
                #then should on that angle
                p_S1_lb = get_point(p_,yaw,on_flat_angle,LEN_S2S1)
                pitch_lb = on_flat_angle
                
            else:
                pitch_lb, touch_p = find_largest_angle(p_,yaw,LEN_S2S1,roll=0)
                if pitch_lb is None:
                    return [],[]
                p_S1_lb = get_point(p_,yaw,pitch_lb,LEN_S2S1)
            alpha_ub, touch_p = find_largest_angle(p_S1_lb, yaw, LEN_S1S0, roll=roll, pitch = pitch_lb)
            if alpha_ub is None:
                return [],[]
            #get lowerbound
            pitch_ub,touch_p = find_largest_angle(p_,yaw,LEN_S2S1+LEN_S1S0,roll=0)
            if pitch_ub is None:
                return [],[]
            alpha_lb = 0.
            if alpha_lb > alpha_ub:
                alphas = [alpha_ub,alpha_lb]
                pitchs = [pitch_lb,pitch_ub]
            else:
                alphas = [alpha_lb,alpha_ub]
                pitchs = [pitch_ub,pitch_lb]
            #alphas = []#np.arange(min(alpha_lb,alpha_ub), max(alpha_lb,alpha_ub),ALPHA_ITS)
            return alphas, pitchs
    else:
        pitch, touch_p = find_largest_angle(p_,yaw,LEN_S2S1,roll=0)
        if pitch is None:
            return [],[]
        p_S1 = get_point(p_,yaw,pitch,LEN_S2S1)
        alpha, touch_p = find_largest_angle(p_S1, yaw, LEN_S1S0, roll=roll, pitch = pitch)
        if alpha is None:
            return [],[]
        return [alpha],[pitch]

def expand_param(path_p):
    '''
        this function is to generate alpha and beta given a path point


        given path point path_p=(neib_l, neib_r)

        neib_l is [p_neib_l_, yaw, pitch, roll, 0.]
    '''
    p_S2_l, yaw, pitch, roll, _ = path_p[0]
    p_S2_r, _, _, _, _ = path_p[1]
    p_S1_l = get_point(p_S2_l, yaw, pitch, LEN_S1S2)
    p_S1_r = get_point(p_S2_r, yaw, pitch, LEN_S1S2)

    l_alpha, _ = find_largest_angle(p_S1_l,yaw,LEN_S0S1,roll=roll,pitch=pitch)
    r_alpha, _ = find_largest_angle(p_S1_r,yaw,LEN_S0S1,roll=roll,pitch=pitch)
    neg_yaw = yaw-np.pi if yaw > 0 else yaw+np.pi
    l_beta, _ = find_largest_angle(p_S2_l,neg_yaw,LEN_S0S1,roll=-roll,pitch=-pitch)
    r_beta, _ = find_largest_angle(p_S2_r,neg_yaw,LEN_S0S1,roll=-roll,pitch=-pitch)

    return (p_S2_l, p_S2_r, yaw, pitch, roll, l_alpha,r_alpha,l_beta,r_beta)

if __name__ == '__main__':
    #global exp_map
    mat = scipy.io.loadmat('man4.mat')
    exp_map = mat['exp_map']
    rnm,cnm = exp_map.shape

    #test case 1
    p_ = [ORIGIN_[0],ORIGIN_[1],0.035]
    r,c,_ = ORIGIN_
    for yaw in YAWS:
        for roll in ROLLS: 
            alpha,pitch = gen_param(p_,yaw,roll) 
            '''
            if len(pitch)>0:
                pdb.set_trace()
                plot_morphology(exp_map[r-WINDOW_SIZE//2:r+WINDOW_SIZE//2+1,c-WINDOW_SIZE//2:c+WINDOW_SIZE//2+1],p_[2],yaw,pitch[0],roll,alpha[0])
            '''
                
            print(alpha)
    #test case 2
    r = 1963
    c = ORIGIN_[1] 
    for yaw in YAWS:
        for roll in ROLLS:
            dh = 0
            for dh in range(8):
                h = exp_map[r,c]+dh*H_ITS
                p_ = (r,c,h) 
                '''
                if(h == 0.2802847783257797 and yaw == 3.141592653589793 and roll == 0):
                    pdb.set_trace() 
                '''
                pdb.set_trace()
                alpha,pitch = gen_param(p_,yaw,roll)
                print(p_,exp_map[r,c],roll,pitch,yaw,alpha) 
                plot_morphology(exp_map[r-WINDOW_SIZE//2:r+WINDOW_SIZE//2+1,c-WINDOW_SIZE//2:c+WINDOW_SIZE//2+1],p_[2],yaw,pitch[0],roll,alpha[0])
    pdb.set_trace()




    for r in range(rnm):
        alpha_list = []
        for c in range(cnm):
            if exp_map[r,c] >=0.5: 
                continue
            for dh in range(8):
                h = exp_map[r,c]+dh*H_ITS
                p_ = (r,c,h)
                for yaw in YAWS:
                    for roll in ROLLS:
                        alpha = gen_param(p_, yaw, roll)
                        alpha_list += alpha
        print(alpha_list)
