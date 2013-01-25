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


rospy.init_node("joint_state_publisher")
pub = rospy.Publisher("/joint_states", JointState)


joints = []
positions = []

joints.append('X')
positions.append(0.0)

joints.append('Y')
positions.append(0.0)

joints.append('yaw')
positions.append(0.0)

joints.append('zlift_z')
positions.append(0.0)

joints.append('m3joint_ma16_j0')
positions.append(0.0)

joints.append('m3joint_ma16_j1')
positions.append(0.0)

joints.append('m3joint_ma16_j2')
positions.append(0.0)

joints.append('m3joint_ma16_j3')
positions.append(0.0)

joints.append('m3joint_ma16_j4')
positions.append(0.0)

joints.append('m3joint_ma16_j5')
positions.append(0.0)

joints.append('m3joint_ma16_j6')
positions.append(0.0)


header = Header(0,rospy.Time.now(),'0')
pub.publish(JointState(header, joints, positions, [0]*len(positions), [0]*len(positions)))

try:
    while True:
	time.sleep(1.0)
	if not rospy.is_shutdown():
	    header = Header(0,rospy.Time.now(),'0')
	    pub.publish(JointState(header, joints, positions, [0]*len(positions), [0]*len(positions)))	    
except (KeyboardInterrupt,EOFError):
    pass




