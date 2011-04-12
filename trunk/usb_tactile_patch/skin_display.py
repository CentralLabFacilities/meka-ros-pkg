#! /usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from usb_tactile_patch.msg import UsbTactilePatch

initial_reading = True
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
    
max_surf=500;
fig = plt.figure()
#ax = fig.gca(projection='3d') for matplotlib ver 1.01
ax = Axes3D(fig)


rospy.init_node('skin_display', anonymous=True)
rospy.Subscriber("/usb_tactile_patch", UsbTactilePatch, callback)

var = np.zeros((6,6))
row = np.zeros((len(var),len(var)))
col = np.zeros((len(var),len(var)))
for row_num in range(len(var)):
        for col_num in range(len(var)):
            row[row_num,col_num] = row_num
            col[row_num,col_num] = col_num
surf = ax.plot_surface(row, col, var, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
plt.draw()

while not rospy.is_shutdown():
    '''if max(surf_value)>max_surf:
        max_surf=max(surf_value)'''
        
    var = 1 * np.array([[0, 0, 0, 0, 0, 0],
        [0, surf_value[7], surf_value[8], surf_value[10], 0, 0],
        [0, surf_value[6], surf_value[9], surf_value[11], 0, 0],
        [0, surf_value[3], surf_value[0], surf_value[5], 0, 0],
        [0, surf_value[4], surf_value[1], surf_value[2], 0, 0],
        [0, 0, 0, 0, 0, 0]],np.int32)
    surf.remove()
    surf = ax.plot_surface(row, col, var, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
    ax.set_zlim3d(0,max_surf)
    plt.draw()
    #print surf_value
    #print max_surf
    rospy.sleep(0.1)

    
