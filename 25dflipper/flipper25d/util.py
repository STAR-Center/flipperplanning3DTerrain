from flipper25d.config import *
import os
import pdb


if os.path.exists('line_trace.mat'):
    line_trace_dict = scipy.io.loadmat('line_trace.mat')
else:
    line_trace_dict = {}

def save_line_trace_dict():
    scipy.io.savemat('line_trace.mat',line_trace_dict)

def line_trace(yaw, linelen):
    if (yaw,linelen) in line_trace_dict.keys():
        return line_trace_dict[(yaw,linelen)]
    '''
            ^ x
            |
            |
       <----|
       y
    '''
    if yaw == 0:
        line = list(range(1,int(linelen/PIXEL_RESO)+1)) 
        return [[-l,0] for l in line]
    elif yaw == np.pi or yaw == -np.pi: 
        line = list(range(1,int(linelen/PIXEL_RESO)+1)) 
        return [[l,0] for l in line] 
    elif yaw == np.pi/2:
        line = list(range(1,int(linelen/PIXEL_RESO)+1))  
        return [[0,l] for l in line]  
    elif yaw == -np.pi/2:
        line = list(range(1,int(linelen/PIXEL_RESO)+1))  
        return [[0,-l] for l in line]

    # from zero
    c_i = 0
    r_i = 0

    if yaw > 0:#c > 0
        c_inc = 1
        if yaw <= np.pi/2:#r < 0
            r_inc = -1
        else:#r > 0
            r_inc = 1
    else:
        c_inc = -1
        if yaw >= -np.pi/2:#r < 0
            r_inc = -1
        else:#r > 0
            r_inc = 1

    tan = abs(np.tan(yaw))
    if tan > 1.0000002:
        c_i += c_inc
    else:
        r_i += r_inc

    line = [[0,0]]
    while np.sqrt(c_i**2+r_i**2)<int(linelen/PIXEL_RESO)+1:
        line.append([r_i,c_i])
        try:
            if (r_i != 0):
                if abs(c_i/r_i) < tan:
                    c_i += c_inc
                else:
                    r_i += r_inc
            else:
                r_i += r_inc
        except:
            print(yaw, c_i, r_i)
            return []
    #remove the L corner
    line_rm_L = []
    rm_flag = True
    for i in range(1,len(line)-1):
        if (line[i][0] == line[i-1][0] and line[i][1] == line[i+1][1])\
                or (line[i][1] == line[i-1][1] and line[i][0] == line[i+1][0]):
            if rm_flag == True:
                rm_flag = False
            else:
                line_rm_L.append(line[i])
                rm_flag = True
        else:
            line_rm_L.append(line[i])
            rm_flag = True
    if rm_flag == False:
        line_rm_L.append(line[-1])

    return line_rm_L


def get_plane(line_len):
    #this function will provide a ids from a round disc with radius line_len
    max_id = int(line_len//PIXEL_RESO)
    plane_list = np.mgrid[-max_id:max_id+1,-max_id:max_id+1].reshape((2,-1))
    sqr_sum_plane_list = np.sum((plane_list*PIXEL_RESO)**2,0)
    plane_list = plane_list.T.tolist()
    final_plane_list = []
    for i in range(sqr_sum_plane_list.shape[0]):
        if sqr_sum_plane_list[i]<=line_len**2:
            final_plane_list.append(plane_list[i])
    #final_plane_list = [plane_list[i] if sqr_sum_plane_list[i]<=line_len**2 for i in range(len(plane_list))]
    #remove origin
    final_plane_list.remove([0.,0.])
    return final_plane_list

if __name__ == '__main__':

    for i in range(8):
        yaw = i*np.pi/4 - np.pi/2
        line = line_trace(yaw,0.135)
        print(line)

    line = line_trace(0.1,0.135)
    print(line)


    #get_plane
    print(get_plane(0.135))
