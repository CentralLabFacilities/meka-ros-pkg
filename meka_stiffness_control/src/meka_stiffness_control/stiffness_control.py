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

from collections import OrderedDict
from std_msgs.msg import Float64MultiArray

from functools import partial

STIFFNESS_CONTROLLER = "stiffness_controller/command"


class MekaStiffnessControl(object):

    def __init__(self, name):

        rospy.init_node('meka_stiffness_control', anonymous=True, log_level=rospy.DEBUG)

        self._name = name
        self._prefix = "meka_roscontrol"
        self._joint_names = ["right_arm_j0",
            "right_arm_j1",
            "right_arm_j2",
            "right_arm_j3",
            "right_arm_j4",
            "right_arm_j5",
            "right_arm_j6",
            "left_arm_j0",
            "left_arm_j1",
            "left_arm_j2",
            "left_arm_j3",
            "left_arm_j4",
            "left_arm_j5",
            "left_arm_j6",
            "right_hand_j0",
            "right_hand_j1",
            "right_hand_j2",
            "right_hand_j3",
            "right_hand_j4",
            "left_hand_j0",
            "left_hand_j1",
            "left_hand_j2",
            "left_hand_j3",
            "left_hand_j4",
            "head_j0",
            "head_j1",
            "torso_j0",
            "torso_j1",
            "zlift_j0"]
        self._stiffness_dict = OrderedDict( (name, 1.0) for name in self._joint_names)
        self.init_pub()
        threading.Thread(None, rospy.spin)

    def init_pub(self):
        self._pub = rospy.Publisher("/" + self._prefix + "/"+ STIFFNESS_CONTROLLER, Float64MultiArray, queue_size=1)

    def apply_stiffness(self):
        msg = Float64MultiArray()
        for name in self._stiffness_dict:
            msg.data.append(self._stiffness_dict[name])
        rospy.logdebug(msg)
        self._pub.publish(msg)

    def set_all_stiffness(self, value):
        for name in self._stiffness_dict:
            self._stiffness_dict[name] = min(1., max(value, 0.))
        self.apply_stiffness()

    def change_stiffness(self, names, values):
        """
        change stiffness of a selection of joints with given values
        @param names: vector of string with joint names to change
        @param values: vector of the values to set for given joint names
        """
        if len(names)==len(values):
            for name, val in zip(names, values):
                if name in self._stiffness_dict:
                    self._stiffness_dict[name] = min(1., max(val, 0.))
                else:
                    rospy.logwarn("name ", name, " does not exist in the stiffness controller")
            self.apply_stiffness()
        else:
            rospy.logwarn("names and values must be the same lenght to change stiffness")

    def get_joints(self):
        return self._joint_names


def main():
    FORMAT = "%(levelname)s %(asctime)-15s %(name)s %(module)s - %(message)s"
    logging.basicConfig(format=FORMAT)

    stiffness_control_exec = MekaStiffnessControl("whatever")

    time.sleep(1)

    rospy.logdebug("current ver")

    stiffness_control_exec.apply_stiffness()
    stiffness_control_exec.change_stiffness(["right_arm_j3", "left_arm_j3"],[0.5, 0.2])
    stiffness_control_exec.set_all_stiffness(0.3)

    rospy.spin()
    #while True:
    #    time.sleep(1)

if __name__ == "__main__":
    main()
