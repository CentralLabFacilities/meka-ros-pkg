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
from meka_posture_execution.posture_execution import MekaPostureExecution

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"

        
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
    
    rospy.logdebug("current ver")
    
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
