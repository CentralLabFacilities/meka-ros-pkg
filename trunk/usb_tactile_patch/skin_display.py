#! /usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from usb_tactile_patch.msg import UsbTactilePatch
import cv

max_surf=9

#initial_reading = True
initial_reading = False
num_values = 12
surf_value = [0]*num_values
initial_value = [0]*num_values

def callback(data):
    global surf_value
    global initial_reading
    global initial_value
    
    if initial_reading:
        initial_reading = False    
        for i in range(num_values):
            initial_value[i] = data.tactile_value[i]
    
    for i in range(num_values):
        surf_value[i] = data.tactile_value[i] - initial_value[i]
        
    
plt.interactive(True)    
    

fig = plt.figure()
#ax = fig.gca(projection='3d') for matplotlib ver 1.01
ax = Axes3D(fig)


rospy.init_node('skin_display', anonymous=True)
rospy.Subscriber("/usb_tactile_patch", UsbTactilePatch, callback)

size = 6
scale = 4

bigger = np.zeros((size*scale, size*scale))
row = np.zeros((len(bigger),len(bigger)))
col = np.zeros((len(bigger),len(bigger)))
for row_num in range(len(bigger)):
        for col_num in range(len(bigger)):
            row[row_num,col_num] = row_num
            col[row_num,col_num] = col_num
surf = ax.plot_surface(row, col,bigger, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
plt.draw()

#TODO: add cubic interpolation s.t. there are 4 times as many points and also make color map to max of scale

print "Starting node_leaf visualizer.  Press Ctrl-C to exit."

while not rospy.is_shutdown():
    '''if max(surf_value)>max_surf:
        max_surf=max(surf_value)'''
    
    var = 1 * np.array([[0, 0, 0, 0, 0, 0],
        [0, surf_value[7], surf_value[8], surf_value[10], 0, 0],
        [0, surf_value[6], surf_value[9], surf_value[11], 0, 0],
        [0, surf_value[3], surf_value[0], surf_value[5], 0, 0],
        [0, surf_value[4], surf_value[1], surf_value[2], 0, 0],
        [0, 0, 0, 0, 0, 0]],np.int32)
    
    '''col = 1 * np.array([[0, 0, 0, 0, 0, 0],
        [0, surf_value[7]/max_surf, surf_value[8]/max_surf, surf_value[10]/max_surf, 0, 0],
        [0, surf_value[6]/max_surf, surf_value[9]/max_surf, surf_value[11]/max_surf, 0, 0],
        [0, surf_value[3]/max_surf, surf_value[0]/max_surf, surf_value[5]/max_surf, 0, 0],
        [0, surf_value[4]/max_surf, surf_value[1]/max_surf, surf_value[2]/max_surf, 0, 0],
        [0, 0, 0, 0, 0, 0]],np.int32)'''
        
    palette = plt.matplotlib.colors.LinearSegmentedColormap('jet3',plt.cm.datad['jet'],max_surf)

    surf.remove()
    
    '''big_cv = cv.fromarray(bigger)
    var_cv = cv.fromarray(var)'''
    
    big_cv = cv.CreateImage((size*scale,size*scale), cv.IPL_DEPTH_8U, 1)
    var_cv = cv.CreateImage((size,size), cv.IPL_DEPTH_8U, 1)

    for i in range(len(var)):
        for j in range(len(var)):
            var_cv[i,j] = var[i,j]
    
    #cv.Resize(var, bigger, cv.CV_INTER_CUBIC)
    cv.Resize(var_cv, big_cv, cv.CV_INTER_CUBIC)
    
    for i in range(len(bigger)):
        for j in range(len(bigger)):
            if big_cv[i,j] > max_surf:
                bigger[i,j] = max_surf
            else:
                bigger[i,j] = big_cv[i,j]
    #bigger = np.asarray(big_cv)
    surf = ax.plot_surface(row, col, bigger, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
    ax.set_zlim3d(0,max_surf)
    plt.draw()
    #print surf_value
    #print max_surf
    rospy.sleep(0.1)

    
