#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from collections import deque
import numpy as np

# init ROS
rospy.init_node('force_helper')
pub = rospy.Publisher('/force_helper', WrenchStamped, queue_size=1)

# hyperparameters
window_size = 10
step_size = 0.01
wrench_queue = deque(maxlen=window_size*2)

def _on_new_force_torque(wrench):
    global window_size
    global wrench_queue
    global old_wrench

    wrench_queue.append(wrench)

    if len(wrench_queue) == window_size*2:
        f_x = 0
        f_y = 0
        f_z = 0
        f_old_x = 0
        f_old_y = 0
        f_old_z = 0

        for i,w in enumerate(wrench_queue):
            if i < window_size:
                f_old_x += w.wrench.force.x
                f_old_y += w.wrench.force.y
                f_old_z += w.wrench.force.z
            else:
                f_x += w.wrench.force.x
                f_y += w.wrench.force.y
                f_z += w.wrench.force.z

        f_x /= window_size
        f_y /= window_size
        f_z /= window_size
        f_old_x /= window_size
        f_old_y /= window_size
        f_old_z /= window_size

        ws = WrenchStamped()
        ws.header.stamp = rospy.Time.now()

        ws.wrench.force.x = (f_x-f_old_x)/(window_size*step_size)
        ws.wrench.force.y = (f_y-f_old_y)/(window_size*step_size)
        ws.wrench.force.z = (f_z-f_old_z)/(window_size*step_size)
        current_force = np.array([ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z])

        ws.wrench.torque.x = np.linalg.norm(current_force)
        ws.wrench.torque.y = np.sum(current_force)

        pub.publish(ws)



# subscribers
wrench_sub = rospy.Subscriber('/meka_ros_pub/m3loadx6_ma29_l0/wrench', WrenchStamped, _on_new_force_torque)
rospy.spin()
