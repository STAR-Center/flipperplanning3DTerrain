from flipper25d.vis.vis import draw_arena
from flipper25d.expand import png_to_map, expand_map
from flipper25d.config import *

from scipy.misc import imread 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 
import pdb

if __name__ == '__main__':
    png_map = imread('../map/'+ARENA_NAME+'.png') 
    height_map = png_to_map(png_map, ARENA_NAME)
    expand_map = expand_map(height_map)

    pdb.set_trace()
    local_height_map = height_map[399-WINDOW_SIZE//2:399+WINDOW_SIZE//2+1, 399-WINDOW_SIZE//2:399+WINDOW_SIZE//2+1]
    local_expand_map = expand_map[399-WINDOW_SIZE//2:399+WINDOW_SIZE//2+1, 399-WINDOW_SIZE//2:399+WINDOW_SIZE//2+1]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_ylim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_zlim3d([-LEN_S2S1S0,LEN_S2S1S0]) 
    draw_arena(ax,local_height_map) 
    plt.show()

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_ylim3d([-LEN_S2S1S0,LEN_S2S1S0])
    ax.set_zlim3d([-LEN_S2S1S0,LEN_S2S1S0]) 
    draw_arena(ax,local_expand_map) 
    plt.show() 
