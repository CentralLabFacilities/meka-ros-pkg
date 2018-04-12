#!/usr/bin/env python

import rospy
import time

from optparse import OptionParser
from meka_posture_execution.posture_execution import MekaPostureExecution
from meka_posture_execution_msgs.srv import *

if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("--postures", help="Path to postures made available", default = "/vol/meka/nightly/share/meka_posture_execution/config/postures.yml",
        dest="posture_path")
    
    (opts, args_) = parser.parse_args()
        
    # init posture execution
    meka_posture_exec = MekaPostureExecution()
    
    # load postures
    meka_posture_exec.load_postures(opts.posture_path)
    
    time.sleep(1)

    # list available postures after successfull load
    rospy.loginfo("Postures:")
    rospy.loginfo(meka_posture_exec.get_postures())
    
    posture_execution_client = rospy.Service('execute_posture', ExecutePosture, meka_posture_exec.handle_ros)
    posture_get_execution_client = rospy.Service('get_postures', GetPostures, meka_posture_exec.get_postures_ros)
    posture_named_target_client = rospy.Service('execute_named_target', ExecuteNamedTarget, meka_posture_exec.moveit_pose)
    
    rospy.loginfo('All clients successfully initialized! Waiting for requests.')
    
    rospy.spin()
