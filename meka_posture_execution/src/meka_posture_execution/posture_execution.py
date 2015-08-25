#!/usr/bin/env python

import threading
import yaml
import time
import logging

from os import sys, path
from optparse import OptionParser
from math import radians

import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
FollowJointTrajectoryGoal

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectoryPoint

import interfaces

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"


class MekaPostureExecution(object):
    
    def __init__(self, name):        

        rospy.init_node('meka_posture_execution', anonymous=True, log_level=rospy.DEBUG)
        
        self._name = name
        self._prefix = "meka_roscontrol"
        self._client = {}
        self.joints = {}
        
        threading.Thread(None, rospy.spin)

    def _set_up_action_client(self,group_name):
        """
        Sets up an action client to communicate with the trajectory controller
        """

        self._client[group_name] = SimpleActionClient(
            self._prefix + "/"+ group_name + JNT_TRAJ_SRV_SUFFIX,
            FollowJointTrajectoryAction
        )

        if self._client[group_name].wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to %s action server in 4 sec",group_name)
            raise


    def execute(self, group_name, posture_name, time_from_start=2.0):
        """
        Executes
        """
        
        if group_name not in self.joints:
            rospy.logerr("%s group not available", group_name)
            return
        if posture_name not in self.postures[group_name]:
            rospy.logerr("%s posture not available for group %s", posture_name, group_name)
            return
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joints[group_name]
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(time_from_start)
        point.positions=self.postures[group_name][posture_name]
        
        goal.trajectory.points = []
        goal.trajectory.points.append(point)
        
        if group_name not in self._client:
            rospy.logerr("Action client for %s not initialized. Trying to initialize it...", group_name)
            try:
                self._set_up_action_client(group_name)
            except:
                rospy.logerr("Could not set up action client for %s.", group_name)
                return
            
        self._client[group_name].send_goal(goal)

    def load_joints(self, path):
        # TODO: read this from the running system instead a cfg file
        with open(path, 'r') as infile:
            self.joints = yaml.load(infile)
        
    def load_postures(self, path):
        # TODO: maybe read this from the moveit cfg / srdf
        with open(path, 'r') as infile:
            self.postures = yaml.load(infile)
        
    def handle(self, event):
                
        rospy.logdebug("Received event: %s" % event)
        
        call_str = event.data.split()
        des_jnt, des_pos = call_str[0], call_str[1]
        
        self.execute(des_jnt, des_pos)
        
def main():
    FORMAT = "%(levelname)s %(asctime)-15s %(name)s %(module)s - %(message)s"
    logging.basicConfig(format=FORMAT)

    parser = OptionParser()
    parser.add_option("--joints", help="Path to joints made available", default = sys.path[0]+"/../../cfg/joints.yml",
        dest="joint_path")
    parser.add_option("--postures", help="Path to postures made available", default = sys.path[0]+"/../../cfg/postures.yml",
        dest="posture_path")
    parser.add_option("--scope", help="Scope to listen to for remote calls", default = "/meka/posture_execution",
        dest="scope")
    
    (opts, args_) = parser.parse_args()
        
    meka_posture = MekaPostureExecution("whatever")
    
    meka_posture.load_joints(opts.joint_path)
    meka_posture.load_postures(opts.posture_path)
    
    try:
        rsbiface =  interfaces.RSBInterface(opts.scope, meka_posture.handle)
    except Exception, e:
        logging.error("Interface not brought up! Error was: \"%s\"", e)
    
if __name__ == "__main__":
    main()
