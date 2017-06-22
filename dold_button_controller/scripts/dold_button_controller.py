#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# author semeyerz

import rospy

from dold_msgs.msg import DoldStates, DoldState
import os
import signal
import subprocess
import actionlib
from hand_over.msg import HandOverAction, HandOverGoal, HandOverFeedback, HandOverResult
import time
from enum import Enum

class Conditions(Enum):
    SINGLE=1
    LOW=2
    HIGH=3



class DoldButtonController(object):

    def __init__(self):
        self.CONDITION = Conditions.LOW
        self.PARTICIPANT = 555555


        self.MAX = 9


        self.counti = 0


        rospy.loginfo("Starting for "+str(self.CONDITION)+"_"+str(self.PARTICIPANT)+"_"+str(self.counti)+"...")

        self._last_message_time = 0

        self._button_sub = rospy.Subscriber("/dold_driver/state", DoldStates,
                                        self.button_callback,
                                        queue_size=2)
        self._client = actionlib.SimpleActionClient('hand_over', HandOverAction)
        self._client.wait_for_server()
        rospy.loginfo('... Ready!')

        #self.do_sth()


    def button_callback(self, msg):
        """
        callback to process button messages
        :param msg:
        :type msg: DoldStates
        """
        cmd = None
        tmp_cmd = 0
        for event in msg.inputs:
            #print "dold event: ", event
            # check whether button zero was pressed as all other buttons are already in use.
            if event.type == DoldState.BUTTON and event.state == DoldState.PRESSED and event.name == 'B0':
                print "button ", event.name, " was pressed"
                if (self.MAX == self.counti):
                    return
                #receive
                self.do_sth("receive")
                #give
                self.do_sth("give")
                self.counti = self.counti + 1


    def do_sth(self,suffix):

        bashCommand = "rosbag record --duration=5m " \
                      "--output-prefix=/vol/hand_over_data/"+str(self.CONDITION)+"_"+str(self.PARTICIPANT)+"_"+str(self.counti)+"_"+suffix+" " \
                                                                                                            "--regex \"/hand_over/(.*)\" " \
                                                                                                            "--regex \"/meka_roscontrol/(.*)/follow_joint_trajectory/(.*)\" " \
                                                                                                            "--regex \"(.*)/camera_info\" " \
                                                                                                            "/rosout " \
                                                                                                            "/tf " \
                                                                                                            "/tf_static " \
                                                                                                            "/usb_cam/image_raw/compressed " \
                                                                                                            "/xtion/rgb/image_raw/compressed " \
                                                                                                            "/meka_ros_pub/m3loadx6_ma29_l0/wrench " \
                                                                                                            "/meka_ros_pub/m3loadx6_ma30_l0/wrench " \
                                                                                                            "/joint_states"

        rospy.loginfo('Calling %s' % bashCommand)

        process = subprocess.Popen(bashCommand, shell=True, preexec_fn=os.setsid)

        # Creates a goal to send to the action server.
        goal = HandOverGoal()
        goal.group_name = 'right_arm'


        if(self.CONDITION==Conditions.SINGLE):
            goal.type = goal.TYPE_SINGLE_HANDED
        elif (self.CONDITION == Conditions.HIGH):
            if (self.counti % 2 == 0):
                goal.type = goal.TYPE_NONVERBAL_HIGH
            else:
                goal.type = goal.TYPE_SINGLE_HANDED
        elif(self.CONDITION==Conditions.LOW):
            if( self.counti % 2 == 0):
                goal.type = goal.TYPE_NONVERBAL_LOW
            else:
                goal.type = goal.TYPE_SINGLE_HANDED

        rospy.loginfo('Calling hand_over')
        self._client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self._client.wait_for_result()
        rospy.loginfo('received hand_over result')

        os.killpg(os.getpgid(process.pid), signal.SIGINT)

        # Prints out the result of executing the action
        return self._client.get_result()


def main():
    rospy.init_node('dold_button_controller')
    dold_meka_if = DoldButtonController()

    rospy.spin()

if __name__ == "__main__":
    main()
