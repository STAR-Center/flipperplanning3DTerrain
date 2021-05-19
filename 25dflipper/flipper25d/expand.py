import numpy as np
from scipy.misc import imread
from scipy.io import savemat
import matplotlib.pyplot as plt
from config import *
import pdb
'''
Space expansion for man4 and man6

resolution: 0.003m/pixel
Pixel value of MAN4: // with 15 degree ramps
value_floor = 30; // general ground
value_ramp = [30, 100];  // ramp from 30 to 100
value_wall = 255; // the height of wall
value_outside = 128;

Pixel value of MAN6: 
value_floor = 30; // general ground
value_bar = 100;  // height of bar
value_wall = 255; // height of wall
value_outside = 128; // 

---------------------- On obstacle -------------------


resolution = 0.003m/pixel

200 pixel = 0.6 m

for step:
value_step = 100;

for ramp:
value_ramp = 100; %[30, 100] 
'''

def kernel(size):
    #size: size in pixel;
    #        here each pixel is with PIXEL_RESO, so the real size is size * PIXEL_RESO
    #return:
    #   k: kernel
    #   k_mask: true/false table tell if > 0
    k = np.zeros((size,size))
    mid = size//2#mid id in k
    for r in range(size):
        for c in range(size):
            k[r,c] = np.sqrt(np.max([0.,WHEEL_RADIUS**2 - ((r-mid)*PIXEL_RESO)**2-((c-mid)*PIXEL_RESO)**2]))
    return k, k>0

def expand_map(height_map):
    '''
        get 2.5D map and expand it
    '''
    k, k_mask = kernel(KERNEL_SIZE)
    kernel_radius = KERNEL_SIZE//2


    #parallel later
    origin = height_map.copy()
    rnm, cnm = origin.shape
    '''
    for r in range(rnm):
        for c in range(cnm):
    '''
    for r in range(kernel_radius,rnm-kernel_radius-1):
        for c in range(kernel_radius,cnm-kernel_radius-1): 
            if origin[r,c] < 0.5:#not wall not unknown
                local_expan = k + k_mask * height_map[r,c]#only the ball region will be covered
                origin[r-kernel_radius:r+kernel_radius+1, c-kernel_radius:c+kernel_radius+1] = \
                        np.maximum(origin[r-kernel_radius:r+kernel_radius+1, c-kernel_radius:c+kernel_radius+1],\
                                    local_expan)
    return origin




def png_to_map(png, map_name):
    '''
       get png and transform it to the 2.5D map

       the wall is 0.5m, the outside should be 1m
    '''
    height_map = np.zeros(png.shape)
    if map_name ==  'man4' or 'ramp' in map_name:#[ ramp, bar ]
        height_map[png == 128] = 1.
        height_map[png == 255] = .5
        height_map[(png >= 30) * (png <= 100)] = (png[(png >= 30 )*( png <= 100)] - 30)/70 * (RAMP_HEIGHT_HIGH-RAMP_HEIGHT_LOW) + RAMP_HEIGHT_LOW
        return height_map
    elif map_name == 'man6' or 'step' in map_name:
        height_map[png == 128] = 1.
        height_map[png == 255] = .5
        height_map[png == 100] = .095# step only have 100 and 0
        height_map[png == 0] = 0.
        return height_map
    else:
        assert(False)

if __name__ == "__main__":
    #map_list = ['man4', 'man6']
    png_name = ARENA_NAME

    png_map = imread('exp/map/'+png_name+'.png')
    height_map = png_to_map(png_map, png_name)
    expanded_map = expand_map(height_map)

    savemat('exp/dilated_map/'+png_name+'.mat', {'exp_map': expanded_map})


