#! /usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from usb_tactile_patch.msg import UsbTactilePatch

max_surf=200;
num_values = 12
initial_reading = True
initial_value = [0.0]*num_values
fig = []


def callback(data):
    if initial_reading:
        initial_reading = False    
        for i in range(num_values):
            initial_value[i] = data.tactile_value[i]
    surf_value = [0.0]*num_values
    for i in range(num_values):
        surf_value = data.tactile_value[i] - initial_value[i]
        
    if max(surf_value)>max_surf:
        max_surf=max(surf_value)
        
    x = np.array([[0, 0, 0, 0, 0, 0],
        [0 surf_value[8], surf_value[9], surf_value[11], surf_value[13], 0]
        [0 surf_value[7], surf_value[10], surf_value[12], surf_value[14], 0]
        [0 surf_value[4], surf_value[1], surf_value[6], surf_value[15], 0]
        [0 surf_value[5], surf_value[2], surf_value[3], surf_value[16], 0]
        [0, 0, 0, 0, 0, 0]],
        ,np.int32)
    
    temp_var=-[0 0 0 0 0 0
                [0 surf_value([8 9 11 13]) 0];...
                [0 surf_value([7 10 12 14]) 0];...
                [0 surf_value([4 1 6 15]) 0];...
                [0 surf_value([5 2 3 16]) 0];...
                0 0 0 0 0 0]


rospy.init_node('skin_display', anonymous=True)
rospy.Subscriber("/usb_tactile_patch", UsbTactilePatch, callback)

fig = plt.figure()
#ax = fig.gca(projection='3d') for matplotlib ver 1.01
ax = Axes3D(fig)

rospy.spin()