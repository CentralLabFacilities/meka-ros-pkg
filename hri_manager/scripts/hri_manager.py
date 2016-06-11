#! /usr/bin/env python

import rospy
import numpy
from functools import partial

import actionlib
from actionlib import SimpleActionClient
from hri_msgs.msg import HriAction, HriGoal, HriFeedback, HriResult
from hri_msgs.msg import HriEvent, HriInteraction, HriTask, HriTaskResult, HriTaskPhase

from meka_posture.meka_posture import MekaPosture

from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"


class HriManager(object):
    # create messages that are used to publish feedback/result
    _feedback = HriFeedback()
    _result = HriResult()

    def __init__(self, name):
        self._action_name = name
        self._prefix = "meka_roscontrol"

        self._as = actionlib.SimpleActionServer(self._action_name, HriAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._meka_posture = MekaPosture("mypostures")
        self._client = {}
        self._movement_finished = {}
        self._cond = {}
        self._valid = {}
        self._timeout = 0
        self._r = rospy.Rate(1)
        self._last_event_id = 0
        self._as.start()

    def _is_task_valid(self, task):
        if task.group_name == "":
            return False
        if task.type == 0:
            return False
        if task.type == HriTask.MOTION:
            if len(task.joint_motion.points) == 0 and len(task.grasp.grasp_posture.points) == 0:
                return False
            if task.interaction_expected:
                if task.interaction.type == 0:
                    # interaction type should be non null for a motion
                    return False
                if not self._is_interaction_valid(task.interaction):
                    return False

        if task.type == HriTask.WAIT:
            if not task.interaction_expected:
                # a time must be provided at least in the interaction
                return False
            if task.interaction.type == 0:
                if task.interaction.timeout.to_sec() == 0:
                    # timeout cannot be null for a WAIT only interaction
                    return False
            if not self._is_interaction_valid(task.interaction):
                return False
        return True

    def _is_interaction_valid(self, interaction):

        if (interaction.type & HriInteraction.ID) == HriInteraction.ID:
            if interaction.event_id == 0:
                # event id cannot be null for ID matching
                return False
        if (interaction.type & HriInteraction.CONTACT) == HriInteraction.CONTACT:
            if interaction.contact_threshold_value == 0:
                # contact threshold cannot be null for CONTACT testing
                return False
        if (interaction.type & HriInteraction.FORCE) == HriInteraction.FORCE:
            if interaction.force_threshold_value == 0:
                # force threshold cannot be null for FORCE testing
                return False
        if (interaction.type & HriInteraction.TORQUE) == HriInteraction.TORQUE:
            if interaction.torque_threshold_value == 0:
                # torque threshold cannot be null for TORQUE testing
                return False
        if (interaction.type & HriInteraction.VOICE) == HriInteraction.VOICE:
            if interaction.voice_trigger_text == "":
                # voice_trigger_text cannot be empty for VOICE matching
                return False
        if (interaction.type & HriInteraction.VISUAL) == HriInteraction.VISUAL:
            if interaction.visual_trigger_text == "":
                # visual_trigger_text cannot be empty for VISUAL matching
                return False
        return True

    def _init_interaction_dict(self, interaction):

        if (interaction.type & HriInteraction.ID) == HriInteraction.ID:
            self._cond['ID'] = interaction.event_id
            self._valid['ID'] = False
        if (interaction.type & HriInteraction.CONTACT) == HriInteraction.CONTACT:
            self._cond['CONTACT'] = interaction.contact_threshold_value
            self._valid['CONTACT'] = False
        if (interaction.type & HriInteraction.FORCE) == HriInteraction.FORCE:
            self._cond['FORCE'] = interaction.force_threshold_value
            self._valid['FORCE'] = False
        if (interaction.type & HriInteraction.TORQUE) == HriInteraction.TORQUE:
            self._cond['TORQUE'] = interaction.torque_threshold_value
            self._valid['TORQUE'] = False
        if (interaction.type & HriInteraction.VOICE) == HriInteraction.VOICE:
            self._cond['VOICE'] = interaction.voice_trigger_text
            self._valid['VOICE'] = False
        if (interaction.type & HriInteraction.VISUAL) == HriInteraction.VISUAL:
            self._cond['VISUAL'] = interaction.visual_trigger_text
            self._valid['VISUAL'] = False
        self._timeout = rospy.Time.now + interaction.timeout

    def _set_up_action_client(self, group_name):
        """
        Sets up an action client to communicate with the trajectory controller
        """

        self._client[group_name] = SimpleActionClient(
            self._prefix + "/" + group_name + JNT_TRAJ_SRV_SUFFIX,
            FollowJointTrajectoryAction
        )

        if self._client[group_name].wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to %s action server in 4 sec", group_name)
            del self._client[group_name]
            raise
        else:
            self._movement_finished[group_name] = True

    def get_motion_goal_from_task(self, task):
        goal = None
        group_name = task.group_name
        if task.description in self._meka_posture.list_postures(group_name):
            goal = self._meka_posture.get_trajectory_goal(group_name, task.description)
        return goal

    def event_handle(self, msg):
        self._last_event_id = msg.id
        if (msg.type & HriInteraction.ID) == HriInteraction.ID:
            if 'ID' in self._cond:
                if msg.id == self._cond['ID']:
                    self._valid['ID'] = True
                # first message of matching id is sufficient, will not reset to false
        if (msg.type & HriInteraction.VOICE) == HriInteraction.VOICE:
            if 'VOICE' in self._cond:
                if msg.voice_event_text == self._cond['VOICE']:
                    self._valid['VOICE'] = True
                # first message of matching id is sufficient, will not reset to false
        if (msg.type & HriInteraction.VISUAL) == HriInteraction.VISUAL:
            if 'VISUAL' in self._cond:
                if msg.visual_event_text == self._cond['VISUAL']:
                    self._valid['VISUAL'] = True
                # first message of matching id is sufficient, will not reset to false
        if (msg.type & HriInteraction.CONTACT) == HriInteraction.CONTACT:
            if 'CONTACT' in self._cond:
                if msg.contact_event_value > self._cond['CONTACT']:
                    self._valid['CONTACT'] = True
                else:
                    self._valid['CONTACT'] = False
        if (msg.type & HriInteraction.FORCE) == HriInteraction.FORCE:
            if 'FORCE' in self._cond:
                if numpy.linalg.norm(msg.physic_event_value.force) > self._cond['FORCE']:
                    self._valid['FORCE'] = True
                else:
                    self._valid['FORCE'] = False
        if (msg.type & HriInteraction.TORQUE) == HriInteraction.TORQUE:
            if 'TORQUE' in self._cond:
                if numpy.linalg.norm(msg.physic_event_value.torque) > self._cond['TORQUE']:
                    self._valid['TORQUE'] = True
                else:
                    self._valid['TORQUE'] = False

    def on_motion_done(self, group_name, *cbargs):
        msg = cbargs[1]
        #if msg.error_code == 0:
        self._movement_finished[group_name] = True

    def _check_all_cond(self, cond_type):
        if cond_type == HriInteraction.ANY:
            for key in self._valid:
                if self._valid[key]:
                    return True
        else:
            for key in self._valid:
                if not self._valid[key]:
                    return False
            return True

    def wait_for_condition(self, task, test_condition, motion_test=False, time_out_test=True):
        success = True
         # wait for condition
        while self._check_all_cond(task.interaction.condition) == test_condition:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._result.success = False
                self._as.set_preempted(self._result)
                success = False
                break

            # check conditions
            if time_out_test:
                if rospy.Time.now() > self._timeout:
                    self._result.error_code = HriTaskResult.TIMED_OUT

                    self._result.event_triggered = False
                    self._result.event_id = 0
                    success = False
                    break

            # check motion end
            if motion_test:
                if self._movement_finished[task.group_name]:
                    break

            self._as.publish_feedback(self._feedback)
            self._r.sleep()
        return success

    def start_motion(self, task):
        goal = self.get_motion_goal_from_task(task)
        if goal is not None:
            if task.group_name not in self._client:
                rospy.logerr("Action client for %s not initialized. Trying to initialize it...", task.group_name)
                try:
                    self._set_up_action_client(task.group_name)
                except:
                    rospy.logerr("Could not set up action client for %s.", task.group_name)
                    return False

            if self._client[task.group_name].send_goal(goal, done_cb=partial(self.on_motion_done, task.group_name)):
                self._movement_finished[task.group_name] = False
                return True
            else:
                return False
        else:
            rospy.logerr("No goal found for posture %s in group  %s.", task.description, task.group_name)
            return False

    def execute_motion_until(self, task):
        # execution the motion here and be ready to preempt it
        success = True
        if self.start_motion(task):
            self._feedback.phase.id = HriTaskPhase.STARTED_UNTIL
            self._as.publish_feedback(self._feedback)
            success = self.wait_for_condition(task, test_condition=True, motion_test=True, time_out_test=True)

            if success:
                self._result.event_triggered = True
                self._result.event_id = self._last_event_id

            # if motion still running
            if self._movement_finished[task.group_name] is False:
                # cancel motion
                self._client[task.group_name].cancel_goal()
                self._movement_finished[task.group_name] = True
        else:
            success = False
            self._result.error_code = HriTaskResult.MOTION_ERROR

        return success

    def execute_motion_if(self, task):
        self._feedback.phase.id = HriTaskPhase.STARTING_IF
        self._as.publish_feedback(self._feedback)
        success = self.wait_for_condition(task, test_condition=True, motion_test=False, time_out_test=True)
        if success:
            self._feedback.phase.id = HriTaskPhase.STARTED_IF
            self._as.publish_feedback(self._feedback)
            self._result.event_triggered = True
            self._result.event_id = self._last_event_id

            # run the motion now, until it ends or action is pre-empted
            if self.start_motion(task):
                # wait for result of the motion here
                while self._movement_finished[task.group_name] is False:
                    # check that preempt has not been requested by the client
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)

                        self._result.success = False
                        self._as.set_preempted(self._result)
                        success = False
                        break
                    self._r.sleep()

                # if motion still running
                if self._movement_finished[task.group_name] is False:
                    # cancel motion
                    self._client[task.group_name].cancel_goal()
                    self._movement_finished[task.group_name] = True
            else:
                success = False
                self._result.error_code = HriTaskResult.MOTION_ERROR

        return success

    def execute_motion_while(self, task):
        success = self.wait_for_condition(task, test_condition=True, motion_test=False, time_out_test=True)

        if success:
            self._result.event_triggered = True
            self._result.event_id = self._last_event_id

            # run the motion now,
            if self.start_motion(task):
                # until it ends or action is pre-empted of wait for condition is false again
                success = self.wait_for_condition(task, test_condition=False, motion_test=True, time_out_test=False)

                if self._movement_finished[task.group_name] is False:
                    # cancel motion
                    self._client[task.group_name].cancel_goal()
                    self._movement_finished[task.group_name] = True
            else:
                success = False
                self._result.error_code = HriTaskResult.MOTION_ERROR

        return success

    def execute_cb(self, goal):
        success = True

        # prepare the feedback
        self._feedback.seq = 0

        # check goal validity
        if len(goal.tasks) > 0:
            # check each task first
            for task in goal.tasks:
                if not self._is_task_valid(task):
                    success = False
                    self._result.error_code = HriTaskResult.INVALID_TASK
                    break

            if success:
                # loop on each task
                for task in goal.tasks:

                    self._result.event_triggered = False
                    self._result.event_id = 0

                    self._feedback.seq += 1
                    self._init_interaction_dict(task.interaction)

                    if task.type == HriTask.MOTION and task.interaction.do == HriInteraction.UNTIL:
                        success = self.execute_motion_until(task)
                    if task.type == HriTask.MOTION and task.interaction.do == HriInteraction.IF:
                        success = self.execute_motion_if(task)
                    if task.type == HriTask.MOTION and task.interaction.do == HriInteraction.WHILE:
                        success = self.execute_motion_if(task)
                    if task.type == HriTask.WAIT:
                        success = self.wait_for_condition(task, test_condition=True,
                                                          motion_test=False, time_out_test=True)

                    # test if task succeeded
                    if not success:
                        break

        else:
            success = False
            self._result.error_code = HriTaskResult.INVALID_GOAL

        self._result.success = success
        self._result.last_seq = self._feedback.seq
        self._result.last_phase = self._feedback.phase

        if not success:
            #check if it was pre-empted
            if self._as.is_active():
                rospy.loginfo('%s: Goal invalid' % self._action_name)
                self._as.set_aborted(self._result)

        if success:
            self._result.last_seq = 0
            self._result.error_code = HriTaskResult.NO_ERROR
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('hri_manager')
    HriManager(rospy.get_name())
    rospy.spin()
