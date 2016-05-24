#!/usr/bin/env python

import threading
import yaml
import time
import logging
import json

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

from functools import partial

from meka_posture_execution.interfaces import rsbServer

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"


class MekaPostureExecution(object):
    
    def __init__(self, name):        

        rospy.init_node('meka_posture_execution', anonymous=True, log_level=rospy.DEBUG)
        
        self._name = name
        self._prefix = "meka_roscontrol"
        self._client = {}
        self._movement_finished = {}
        self._meka_posture = MekaPosture("mypostures")
        self.all_done = True;
        
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
            del self._client[group_name]
            raise
        else:
            self._movement_finished[group_name] = True

    def rescale_time_from_start(self, goal, timescale):
        if timescale != 0.0:
            for point in goal.trajectory.points:
                point.time_from_start /= timescale

    def on_done(self, group_name, *cbargs):
        msg = cbargs[1]
        #if msg.error_code == 0:
        self._movement_finished[group_name] = True
        all_finished = True
        for name in self._movement_finished:
            if not self._movement_finished[name]:
                all_finished = False 
                break
        if all_finished:
            self.all_done_callback()
            
        
    def all_done_callback(self):
        """
        triggers when all the movement finished
        """
        self._posture_when_done = "waiting"
        self._movement_finished = {}
        rospy.loginfo("All movement finished")
        if self._previous_posture != self._posture_when_done:
            self.execute("all", self._posture_when_done)
        else:
            self.all_done = True

    def execute(self, group_name, posture_name, timescale=1.0):
        """
        Executes
        @param group_name: group to control
        @param posture_name: posture in this group
        @param timescale: factor to scale the time_from_start of each point
        """
        self._previous_posture = posture_name
        if group_name == "all":
            self._movement_finished = {}
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
                
                self._movement_finished[group_name] = False
                self._client[group_name].send_goal(goal, done_cb=partial(self.on_done, group_name))
                self.all_done = False;
            else:
                rospy.logerr("No goal found for posture %s in group  %s.", posture_name, group_name)

    def load_postures(self, path):
        self._meka_posture.load_postures(path)
        
    def handle(self, event):
        rospy.logdebug("Received event: %s" % event)

        call_str = event.data.split()
        des_jnt, des_pos = call_str[0], call_str[1]    

        self.execute(des_jnt, des_pos)
    
    def execute_rpc(self, request):
        print "rpc called"
        
        call_str = request.split()
        des_jnt, des_pos = call_str[0], call_str[1]

        self.execute(des_jnt, des_pos)
        while not self.all_done:
            print "not done"
            time.sleep(4)
        return True
        
        
    def get_postures(self, ev):
        print "called get postures"
        d = self._meka_posture.list_postures()    
        return json.dumps(d)
        
        
def main():
    FORMAT = "%(levelname)s %(asctime)-15s %(name)s %(module)s - %(message)s"
    logging.basicConfig(format=FORMAT)

    parser = OptionParser()
    parser.add_option("--joints", help="Path to joints made available", default = "/vol/meka/nightly/share/meka_posture_execution/config/joints.yml",
        dest="joint_path")
    parser.add_option("--postures", help="Path to postures made available", default = "/vol/meka/nightly/share/meka_posture_execution/config/postures.yml",
        dest="posture_path")
    parser.add_option("--scope", help="Scope to listen to for remote events", default = "/meka/posture_execution",
        dest="scope")
    parser.add_option("--serverScope", help="Scope to listen to for remote procedure calls", default = "/meka/posture_execution/server",
        dest="serverscope")
    
    (opts, args_) = parser.parse_args()
        
    meka_posture_exec = MekaPostureExecution("whatever")
    
    meka_posture_exec.load_postures(opts.posture_path)
    
    time.sleep(1)
    #meka_posture_exec.execute("head", "nodding_twice");
    
    try:
        rsbif = rsbServer.RSBInterface(scope=opts.scope,
                                        serverscope=opts.serverscope,
                                        handler=meka_posture_exec.handle, 
                                        assumePoseRPC=meka_posture_exec.execute_rpc,
                                        getPosesRPC=meka_posture_exec.get_postures
                                        )
    except Exception, e:
        logging.error("Interface not brought up! Error was: \"%s\"", e)
        
    print "Postures:"
    print meka_posture_exec.get_postures("bla")
    
    rospy.spin()
    #while True:
    #    time.sleep(1)
    
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
