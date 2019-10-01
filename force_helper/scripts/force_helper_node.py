#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from collections import deque
from std_msgs.msg import Float32
import numpy as np

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal

from threading import Lock
lockxyz = Lock()
lockcomm = Lock()
lockstate = Lock()
# init ROS
rospy.init_node('force_helper')
pub_force = rospy.Publisher('/force_helper', WrenchStamped, queue_size=1)
pub_arm = rospy.Publisher('/force_helper/arm', Float32, queue_size=1)
pub_raw = rospy.Publisher('/force_helper/raw', Float32, queue_size=1)
pub_result = rospy.Publisher('/force_helper/result', Float32, queue_size=1)

# hyperparameters
window_size = 15
command_window_size = 200
step_size = 0.01
wrench_queue = deque(maxlen=window_size*2)
command_queue = deque(maxlen=window_size*2)

sq = [-0.10714, -0.07143, -0.03571 ,0, 0.03571, 0.07143, 0.10714]
sq = [-0.03297, -0.02747, -0.02198, -0.01648, -0.01099, -0.00549, 0, 0.00549, 0.01099, 0.01648, 0.02198, 0.02747, 0.03297]
sq_queue = deque(maxlen=len(sq))

arm_queue = deque(maxlen=window_size*2)
current_command = JointTrajectoryPoint() #7*[0]+
current_command.positions = 7*[0]
last_command = JointTrajectoryPoint() #7*[0]
last_command.positions = 7*[0]
arm_avg = 0
last_receive_time = rospy.Time.now()
arm_state_pos = 7*[0]
filtered_x = 0
filtered_y = 0
filtered_z = 0

def _on_new_force_torque(wrench):
    global window_size
    global wrench_queue
    global filtered_x
    global filtered_y
    global filtered_z
    lockxyz.acquire()
    wrench_queue.append(wrench)
    sq_queue.append(wrench.wrench.force)

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

        # f_x /= window_size
        # f_y /= window_size
        # f_z /= window_size
        # f_old_x /= window_size
        # f_old_y /= window_size
        # f_old_z /= window_size

        ws = WrenchStamped()
        ws.header.stamp = rospy.Time.now()

        ws.wrench.torque.x = (f_x-f_old_x) /(window_size*window_size)
        ws.wrench.torque.y = (f_y-f_old_y) /(window_size*window_size)
        ws.wrench.torque.z = (f_z-f_old_z) /(window_size*window_size)
        #current_force = np.array([ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z])


        filtered_x = 0
        filtered_y = 0
        filtered_z = 0

        for i,w in enumerate(sq_queue):
            filtered_x += sq[i] * w.x
            filtered_y += sq[i] * w.y
            filtered_z += sq[i] * w.z

        #filtered_x *= 100
        #filtered_y *= 100
        #filtered_z *= 100

        ws.wrench.force.x = filtered_x #/(window_size*step_size)
        ws.wrench.force.y = filtered_y #/(window_size*step_size)
        ws.wrench.force.z = filtered_z #/(window_size*step_size)        #https://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Derivative-Estimation/derivative-estimation.htm#least-squares
        #y(k) = .3 x(k) + .1        x(k - 1) - .1        x(k - 2) - .3        x(k - 3)
        pub_force.publish(ws)
    lockxyz.release()

def _on_new_arm_command(command):
    global current_command
    global last_receive_time
    lockcomm.acquire()
    if len(command.points) > 0:
        current_command = command.points[0]
        last_receive_time = rospy.Time.now()
    lockcomm.release()


def _on_new_arm_traj(traj):
    global current_command
    global last_receive_time
    lockcomm.acquire()
    current_command = traj.goal.trajectory.points[0] #TODO: iterate
    last_receive_time = rospy.Time.now()
    lockcomm.release()



def _on_arm_timer(event):
    global current_command
    global last_command
    global arm_avg
    lockcomm.acquire()

    if current_command == None:
        return

    lockstate.acquire()
    vel = np.linalg.norm(np.subtract(arm_state_pos,current_command.positions))/current_command.time_from_start.to_sec()
    lockstate.release()
    if(rospy.Time.now() > last_receive_time + current_command.time_from_start*2):
        vel =0
    vel = np.clip(vel*2,0,1.0)
    command_queue.append(vel)
    lockcomm.release()

    n = 0.9
    N=20
    arm_avg = ((N-1)*arm_avg+vel )/N
    arm_avg = sum(command_queue) / len(command_queue) 
    if (arm_avg < vel):
        arm_avg = vel
    lockxyz.acquire()
    val = np.linalg.norm(np.array([np.clip(filtered_x, -99, 99), np.clip(filtered_y, -99, 99)])) # , filtered_z*0.5])) #* (0.5+arm_avg)
    lockxyz.release()
    #rospy.loginfo(arm_avg)

    pub_arm.publish(arm_avg)
    pub_raw.publish(val)
    
    val = val / (0.5+arm_avg)
    pub_result.publish(val)

def _on_new_arm_state(state):
    global arm_state_pos
    lockstate.acquire()
    arm_state_pos = state.actual.positions
    lockstate.release()

# subscribers
wrench_sub = rospy.Subscriber('/meka_ros_pub/m3loadx6_ma29_l0/wrench', WrenchStamped, _on_new_force_torque)
arm_command_sub = rospy.Subscriber('/meka_roscontrol/right_arm_position_trajectory_controller/command', JointTrajectory, _on_new_arm_command)
arm_traj_sub = rospy.Subscriber('/meka_roscontrol/right_arm_position_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, _on_new_arm_traj)

arm_sate_sub = rospy.Subscriber('/meka_roscontrol/right_arm_position_trajectory_controller/state', JointTrajectoryControllerState, _on_new_arm_state)

rospy.Timer(rospy.Duration(0.01), _on_arm_timer, oneshot=False)
rospy.spin()
