#! /usr/bin/python
# -*- coding: utf-8 -*-


#M3 -- Meka Robotics Robot Components
#Copyright (C) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import time
import os
import roslib; roslib.load_manifest('meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def shm_humanoid(joint_states):
    for i in range(len(joints)):
      for j in range(len(joint_states.name)):
	if joints[i] == joint_states.name[j]:
	  positions[i] = joint_states.position[j]
	  velocities[i] = joint_states.velocity[j]
	  effort[i] = joint_states.effort[j]
	  

rospy.init_node("joint_state_publisher")
pub = rospy.Publisher("/joint_states", JointState)


joints = []
positions = []
velocities = []
effort = []

joints.append('head_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)



header = Header(0,rospy.Time.now(),'0')
pub.publish(JointState(header, joints, positions, velocities, effort))

rospy.Subscriber("/humanoid_state", JointState, shm_humanoid)


try:
    while not rospy.is_shutdown():
	time.sleep(0.01)	
	header = Header(0,rospy.Time.now(),'0')
	pub.publish(JointState(header, joints, positions, velocities, effort))
except (KeyboardInterrupt,EOFError,rospy.ROSInterruptException):
    pass




