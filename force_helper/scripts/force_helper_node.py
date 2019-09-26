#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from collections import deque
from std_msgs.msg import Float32
import numpy as np

from control_msgs.msg import JointTrajectoryControllerState

# init ROS
rospy.init_node('force_helper')
pub_force = rospy.Publisher('/force_helper', WrenchStamped, queue_size=1)
pub_arm = rospy.Publisher('/force_helper/arm', Float32, queue_size=1)

# hyperparameters
window_size = 15
step_size = 0.01
wrench_queue = deque(maxlen=window_size*2)
arm_queue = deque(maxlen=window_size*2)
current_command = None
last_command = None
arm_avg = 0
last_receive_time = None
arm_state_pos = None

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

        pub_force.publish(ws)

def _on_new_arm_command(command):
    global arm_queue
    global current_command
    global last_command
    global last_receive_time

    last_command = current_command
    current_command = command.points[0]
    last_receive_time = rospy.Time.now()

    arm_queue.append(command.points[0])


def _on_arm_timer(event):
    global arm_queue
    global current_command
    global last_command
    global arm_avg
    global last_receive_time
    global arm_state_pos

    if current_command == None or last_command == None:
        return

    vel = np.linalg.norm(np.subtract(arm_state_pos,current_command.positions))/current_command.time_from_start.to_sec()
    if(rospy.Time.now() > last_receive_time + current_command.time_from_start*2):
        vel =0

    n = 0.9
    arm_avg = (n*arm_avg+(1-n)* np.clip(vel*8,0,1))

    pub_arm.publish(arm_avg)

def _on_new_arm_state(state):
    global arm_state_pos

    arm_state_pos = state.actual.positions

# subscribers
wrench_sub = rospy.Subscriber('/meka_ros_pub/m3loadx6_ma29_l0/wrench', WrenchStamped, _on_new_force_torque)
arm_command_sub = rospy.Subscriber('/meka_roscontrol/right_arm_position_trajectory_controller/command', JointTrajectory, _on_new_arm_command)
arm_sate_sub = rospy.Subscriber('/meka_roscontrol/right_arm_position_trajectory_controller/state', JointTrajectoryControllerState, _on_new_arm_state)

rospy.Timer(rospy.Duration(0.03), _on_arm_timer, oneshot=False)
rospy.spin()
