#!/usr/bin/env python

import threading

import rospy
import time

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
FollowJointTrajectoryGoal

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectoryPoint
from math import radians


class MekaPostureExecution(object):

    joint_names = {}
    joint_names ['right'] = ['right_arm_j0', 'right_arm_j1', 'right_arm_j2', 'right_arm_j3', 'right_arm_j4', 'right_arm_j5', 'right_arm_j6']
    postures = {}
    postures['right'] = {}
    postures['right']['zero'] = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    postures['right']['pointing_right'] = [0.67984317485, 0.167468029297, 0.650150248168, 0.967911767026, 1.256168726, -0.0985858390576, -0.13327324632]
    def __init__(self, name):
        rospy.init_node('meka_posture_execution', anonymous=True)
        self._name = name
        self._prefix = "meka_roscontrol"
        self._client = {}
        self._set_up_action_client('right')

        threading.Thread(None, rospy.spin)

    def _set_up_action_client(self,group_name):
        """
        Sets up an action client to communicate with the trajectory controller
        """

        self._client[group_name] = SimpleActionClient(
            self._prefix + "/"+ group_name + "_position_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        if self._client[group_name].wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to %s action server in 4 sec",group_name)
            raise

    def execute(self, group_name, posture_name, time_from_start=2.0):
        """
        Executes
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names[group_name]
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(time_from_start)
        point.positions=self.postures[group_name][posture_name]
        
        goal.trajectory.points = []
        goal.trajectory.points.append(point)

        self._client[group_name].send_goal(goal)



if __name__ == "__main__":
    meka_posture = MekaPostureExecution("whatever")
    meka_posture.execute('right','zero',3.0)
    # TODO: wait for result of action lib
    time.sleep(5.0)
    meka_posture.execute('right','pointing_right',2.0)
    rospy.spin()
