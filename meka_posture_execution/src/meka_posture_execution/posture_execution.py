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

from meka_posture.meka_posture import MekaPosture

import interfaces

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"


class MekaPostureExecution(object):
    
    def __init__(self, name):        

        rospy.init_node('meka_posture_execution', anonymous=True, log_level=rospy.DEBUG)
        
        self._name = name
        self._prefix = "meka_roscontrol"
        self._client = {}
        self._meka_posture = MekaPosture("mypostures")
        
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

    def rescale_time_from_start(self, goal, timescale):
        if timescale != 0.0:
            for point in goal.trajectory.points:
                point.time_from_start /= timescale

    def execute(self, group_name, posture_name, timescale=1.0):
        """
        Executes
        @param group_name: group to control
        @param posture_name: posture in this group
        @param timescale: factor to scale the time_from_start of each point
        """
        
        if group_name == "all":
            rospy.loginfo("Calling all the groups")
            groups = ["right_arm", "right_hand", "left_arm","left_hand","torso", "head"]
            for names in groups:
                self.execute(names, posture_name)
        else:
            goal = self._meka_posture.get_trajectory_goal(group_name, posture_name)
            if goal is not None:
                if timescale != 1.0:
                    # rescale the time_from_start for each point
                    self.rescale_time_from_start(goal, timescale)

                if group_name not in self._client:
                    rospy.logerr("Action client for %s not initialized. Trying to initialize it...", group_name)
                    try:
                        self._set_up_action_client(group_name)
                    except:
                        rospy.logerr("Could not set up action client for %s.", group_name)
                        return
                self._client[group_name].send_goal(goal)
            else:
                rospy.logerr("No goal found for posture %s in group  %s.", posture_name, group_name)

    def load_postures(self, path):
        self._meka_posture.load_postures(path)
        
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
        
    meka_posture_exec = MekaPostureExecution("whatever")
    
    meka_posture_exec.load_postures(opts.posture_path)
    
    time.sleep(1)
    #meka_posture_exec.execute("head", "nodding_twice");
    print "Ready"
    
    try:
        rsbiface =  interfaces.RSBInterface(opts.scope, meka_posture_exec.handle)
    except Exception, e:
        logging.error("Interface not brought up! Error was: \"%s\"", e)
        
    
    
if __name__ == "__main__":
    main()
    
    #meka_posture_exec = MekaPostureExecution("whatever")
    
    #parser = OptionParser()
    #parser.add_option("--joints", help="Path to joints made available", default = sys.path[0]+"/../../cfg/joints.yml",
        #dest="joint_path")
    #parser.add_option("--postures", help="Path to postures made available", default = sys.path[0]+"/../../cfg/postures.yml",
        #dest="posture_path")
    #parser.add_option("--scope", help="Scope to listen to for remote calls", default = "/meka/posture_execution",
        #dest="scope")
    
    #(opts, args_) = parser.parse_args()
    
    #meka_posture_exec.load_postures(opts.posture_path)
    #print meka_posture_exec._meka_posture.list_postures("right_arm")
    #meka_posture_exec.execute("right_arm", "zero")
    #rospy.sleep(4)
    #meka_posture_exec.execute("right_arm", "pointing_right",0.5)
    #rospy.sleep(4)
