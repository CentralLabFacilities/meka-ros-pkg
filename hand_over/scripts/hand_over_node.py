#! /usr/bin/env python

import rospy
import numpy
from functools import partial

from os import sys, path
from optparse import OptionParser

import time

import actionlib
from actionlib import SimpleActionClient
from hand_over.msg import HandOverAction, HandOverGoal, HandOverFeedback, HandOverResult

from meka_posture.meka_posture import MekaPosture
from meka_stiffness_control.stiffness_control import MekaStiffnessControl

from geometry_msgs.msg import Wrench, Point

from people_msgs.msg import PositionMeasurementArray

from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"


class HandOver(object):
    # create messages that are used to publish feedback/result
    _feedback = HandOverFeedback()
    _result = HandOverResult()
    _carrying = {'left_arm': False, 'right_arm' : False}

    def __init__(self, name):
        self._action_name = name
        self._prefix = "meka_roscontrol"

        self._as = actionlib.SimpleActionServer(self._action_name, HandOverAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._meka_posture = MekaPosture("posture_exec")
        self._stiffness_control = MekaStiffnessControl("stiffness_control")

        self._client = {}
        self._movement_finished = {}
        self.force_variation = {}
        self.previous_force = {}
        self._r = rospy.Rate(10)

        self.force_variation['left_arm'] = numpy.array([0, 0, 0])
        self.force_variation['right_arm'] = numpy.array([0, 0, 0])
        self.previous_force['left_arm'] = numpy.array([0, 0, 0])
        self.previous_force['right_arm'] = numpy.array([0, 0, 0])

        self.sub_left = rospy.Subscriber("/meka_ros_pub/m3loadx6_ma30_l0/wrench", Wrench, self.handle_left)
        self.sub_right = rospy.Subscriber("/meka_ros_pub/m3loadx6_ma29_l0/wrench", Wrench, self.handle_right)

        self.sub_face = rospy.Subscriber("/face_detector/people_tracker_measurements_array", PositionMeasurementArray, self.face_callback)

        self._as.start()

    def face_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I received %s", data.people[0].name)
        rospy.loginfo(rospy.get_caller_id() + "Frame: %s", data.people[0].header.frame_id)

    def load_postures(self, path):
        self._meka_posture.load_postures(path)

    def _set_up_action_client(self, group_name):
        """
        Sets up an action client to communicate with the trajectory controller
        """

        self._client[group_name] = SimpleActionClient("/" +
                                                      self._prefix + "/" + group_name + JNT_TRAJ_SRV_SUFFIX,
                                                      FollowJointTrajectoryAction
                                                      )
        if self._client[group_name].wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to %s action server in 4 sec", group_name)
            del self._client[group_name]
            raise
        else:
            self._movement_finished[group_name] = True

    def handle_left(self, msg):
        current_force = numpy.array([msg.force.x, msg.force.y, msg.force.z])
        self.force_variation['left_arm'] = (current_force - self.previous_force['left_arm'])
        self.previous_force['left_arm'] = current_force

    def handle_right(self, msg):
        current_force = numpy.array([msg.force.x, msg.force.y, msg.force.z])
        self.force_variation['right_arm'] = (current_force - self.previous_force['right_arm'])
        self.previous_force['right_arm'] = current_force

    def on_motion_done(self, group_name, *cbargs):
        msg = cbargs[1]
        #if msg.error_code == 0:
        self._movement_finished[group_name] = True

    def wait_for_force(self, threshold, group_name, timeout=None):
        success = True
        # wait for condition
        if timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration(timeout)
        while numpy.linalg.norm(self.force_variation[group_name]) < threshold:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._result.success = False
                self._as.set_preempted(self._result)
                success = False
                break

            if timeout is not None:
                if rospy.Time.now() > timeout_time:
                    rospy.loginfo("timeout waiting for force")
                    print numpy.linalg.norm(self.force_variation[group_name]), "not larger than ", threshold
                    success = False
                    break

            self._as.publish_feedback(self._feedback)
            self._r.sleep()

        return success

    def start_motion(self, group_name, posture):
        if posture in self._meka_posture.list_postures(group_name):
            goal = self._meka_posture.get_trajectory_goal(group_name, posture)
            print goal
            if group_name not in self._client:
                rospy.logwarn("Action client for %s not initialized. Trying to initialize it...", group_name)
                try:
                    self._set_up_action_client(group_name)
                except:
                    rospy.logerr("Could not set up action client for %s.", group_name)
                    return False

            self._client[group_name].send_goal(goal, done_cb=partial(self.on_motion_done, group_name))
            self._movement_finished[group_name] = False
            return True
        else:
            rospy.logerr("No goal found for posture %s in group  %s.", posture, group_name)
            return False

    def wait_for_motion(self, group_name):
        success = True
        # wait for result of the motion here
        while self._movement_finished[group_name] is False:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)

                self._result.success = False
                self._as.set_preempted(self._result)
                success = False
                break
            self._r.sleep()
        return success

    def approach(self, group_name):
        success = True
        if self.start_motion(group_name, "shake_approach"):
            self._feedback.phase = HandOverFeedback.PHASE_APPROACH
            self._as.publish_feedback(self._feedback)
            success = self.wait_for_motion(group_name)
        else:
            success = False
        return success

    def retreat(self, group_name):
        success = True
        if "left" in group_name:
            joint_name = "left_arm_j3"
        else:
            joint_name = "right_arm_j3"
        # recover stiffness slowly
        self._stiffness_control.change_stiffness([joint_name], [0.1])
        time.sleep(1)
        self._stiffness_control.change_stiffness([joint_name], [0.5])
        time.sleep(0.5)
        self._stiffness_control.change_stiffness([joint_name], [1.0])

        if self.start_motion(group_name, "shake_retreat"):
            self._feedback.phase = HandOverFeedback.PHASE_RETREAT
            self._as.publish_feedback(self._feedback)
            success = self.wait_for_motion(group_name)
        else:
            success = False
        return success

    def shake(self, group_name):
        if "left" in group_name:
            joint_name = "left_arm_j3"
        else:
            joint_name = "right_arm_j3"

        # reduce stiffness
        self._stiffness_control.change_stiffness([joint_name], [0.35])

        success = True
        if self.start_motion(group_name, "shake_movement"):
            self._feedback.phase = HandOverFeedback.PHASE_EXECUTING
            self._as.publish_feedback(self._feedback)
            success = self.wait_for_motion(group_name)
        else:
            success = False

        return success

    def close_for_shaking(self, group_name):
        success = True
        if "left" in group_name:
            hand_name = "left_hand"
            j0 = "left_hand_j0"
            j1 = "left_hand_j1"
            j2 = "left_hand_j2"
            j3 = "left_hand_j3"
            j4 = "left_hand_j4"
        else:
            hand_name = "right_hand"
            j0 = "right_hand_j0"
            j1 = "right_hand_j1"
            j2 = "right_hand_j2"
            j3 = "right_hand_j3"
            j4 = "right_hand_j4"

        # reduce stiffness
        self._stiffness_control.change_stiffness([j0, j1, j2, j3, j4], [0.35, 0.35, 0.35, 0.35, 0.35])

        if self.start_motion(hand_name, "close"):
            self._feedback.phase = HandOverFeedback.PHASE_EXECUTING
            self._as.publish_feedback(self._feedback)
            # wait for result of the motion here
            success = self.wait_for_motion(hand_name)
        else:
            success = False
        return success

    def open_hand(self, group_name):
        success = True

        if "left" in group_name:
            hand_name = "left_hand"
            j0 = "left_hand_j0"
            j1 = "left_hand_j1"
            j2 = "left_hand_j2"
            j3 = "left_hand_j3"
            j4 = "left_hand_j4"
        else:
            hand_name = "right_hand"
            j0 = "right_hand_j0"
            j1 = "right_hand_j1"
            j2 = "right_hand_j2"
            j3 = "right_hand_j3"
            j4 = "right_hand_j4"

        if self.start_motion(hand_name, "shake_open"): #open
            self._feedback.phase = HandOverFeedback.PHASE_EXECUTING
            self._as.publish_feedback(self._feedback)
            # wait for result of the motion here
            success = self.wait_for_motion(hand_name)
        else:
            success = False

        # recovert stiffness
        self._stiffness_control.change_stiffness([j0, j1, j2, j3, j4], [1, 1, 1, 1, 1])

        return success

    def execute_cb(self, goal):
        success = True

        # prepare the feedback
        self._feedback.phase = 0

        group_name = goal.group_name
        # check goal validity
        if group_name == "right_arm" or group_name == "left_arm":

            # approach
            if self.approach(group_name):
                # wait for touch
                self._feedback.phase = HandOverFeedback.PHASE_WAITING_FOR_CONTACT
                self._as.publish_feedback(self._feedback)
                if self.wait_for_force(threshold=1000.0, group_name=group_name, timeout=20.0):

                    #check if something in hand
                    if self._carrying[group_name] == False:
                    # close hand
                        if self.close_for_shaking(group_name):
                            self._carrying[group_name] = True
                    else:
                        # open hand
                        if self.open_hand(group_name):
                            self._carrying[group_name] = False
                    # retreat
                    if not self.retreat(group_name):
                            success = False
                    else:
                        success = False
                else:
                    success = False
            else:
                success = False
        else:
            success = False

        self._result.success = success

        if not success:
            #check if it was pre-empted
            if not self._as.is_preempt_requested():
                rospy.loginfo('%s: preempted' % self._action_name)
                self._as.set_aborted(self._result)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--postures", help="Path to postures made available", default = "asdfsadf/vol/meka/nightly/share/meka_posture_execution/config/postures.yml",
                      dest="posture_path")

    (opts, args_) = parser.parse_args()

    rospy.init_node('hand_over')
    hs = HandOver(rospy.get_name())

    rospy.loginfo('Loading postures from: %s' % opts.posture_path)

    hs.load_postures(opts.posture_path)
    rospy.spin()
