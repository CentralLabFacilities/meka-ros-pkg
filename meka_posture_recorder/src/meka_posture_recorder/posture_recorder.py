#!/usr/bin/env python

# author: Guillaume WALCK (2015)

import threading
import functools

import rospy
from copy import deepcopy
from control_msgs.msg import JointTrajectoryControllerState

from controller_manager_msgs.srv import ListControllersRequest, \
    ListControllers

from trajectory_msgs.msg import JointTrajectory

from meka_posture_recorder.msg import PostureRecordErrorCodes,\
    PostureRecordPosture, PostureRecordWaypoint
from meka_posture_recorder.srv import PostureRecordStart, PostureRecordStop,\
    PostureRecordSave, PostureRecordAddWaypoint, PostureRecordStartResponse,\
    PostureRecordStopResponse, PostureRecordAddWaypointResponse,\
    PostureRecordSaveResponse, PostureRecordGetAllPosturesResponse,\
    PostureRecordGetAllPostures

from meka_posture.meka_posture import MekaPosture


class MekaPostureRecorder(object):
    """
    Class to record postures via services
    """
    def __init__(self, name):

        self._name = name
        rospy.init_node(name, anonymous=False, log_level=rospy.INFO)

        self._name = name
        self._prefix = "meka_roscontrol"
        self._controller_list = {}
        self._state = {}
        self._sub = {}
        self.init_controller_list()
        self._current_postures = {}
        self._mp = MekaPosture("mypostures")
        self._overall_time_from_start = 0.0

        self.init_services()

        threading.Thread(None, rospy.spin)

    def init_controller_list(self):
        """
        Get all the controllers running and store their name per group
        """
        service_name = self._prefix + \
            '/controller_manager/list_controllers'
        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(service_name, 5.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)
            return False
        cm_list_client = rospy.ServiceProxy(service_name,
                                            ListControllers)
        rospy.loginfo("Found %s", service_name)

        # get all the controllers
        try:
            resp = cm_list_client(ListControllersRequest())
        except rospy.ServiceException:
            rospy.logerr("Could not call list_controllers")
            return

        # loop on the controllers
        if resp:
            for controller in resp.controller:
                cname_split = controller.name.split("_")
                if len(cname_split) > 1:
                    if cname_split[0] in ["torso", "zlift", "head"]:
                        self._controller_list[cname_split[0]] = controller.name
                    else:
                        if len(cname_split) > 2:
                            if cname_split[1] in ["arm", "hand"]:
                                group_name = cname_split[0] + "_" +\
                                    cname_split[1]
                                self._controller_list[group_name] =\
                                    controller.name
        for group_name in self._controller_list:
            if group_name not in self._sub:
                try:
                    self.set_up_subscriber(group_name)
                    # give some time for the subscriber
                    # to receive its first data
                    rospy.sleep(0.5)
                except rospy.ROSException:
                    rospy.logerr("Could not set up subscriber \
                                 for group %s.", group_name)
                    resp.error_code.val =\
                        PostureRecordErrorCodes.NOCONTROLLER
                    

    def init_services(self):
        """
        Initialize the service servers
        """
        service_prefix = rospy.get_name() + "/"

        self._start_record_serv = rospy.Service(service_prefix +
                                                'start_record',
                                                PostureRecordStart,
                                                self.start_recording_cb)
        self._stop_record_serv = rospy.Service(service_prefix + 'stop_record',
                                               PostureRecordStop,
                                               self.stop_recording_cb)
        self._add_wp_serv = rospy.Service(service_prefix +
                                          'add_waypoint',
                                          PostureRecordAddWaypoint,
                                          self.add_waypoint_cb)
        self._save_serv = rospy.Service(service_prefix + 'save',
                                        PostureRecordSave, self.save_cb)
                                        
        self._getall_serv = rospy.Service(service_prefix + 'get_all_postures',
                                        PostureRecordGetAllPostures, self.getall_cb)

    def set_up_subscriber(self, group_name):
        """
        Sets up client to communicate with the trajectory controller
        """

        if group_name in self._controller_list:
            self._sub[group_name] = rospy.Subscriber(
                self._prefix + "/" + self._controller_list[group_name] +
                "/state", JointTrajectoryControllerState,
                functools.partial(self.state_cb, group_name=group_name))
        else:
            rospy.logerr("No controller for group %s", group_name)
            return

    def start_recording_cb(self, req):
        """
        Callback for the start recording service
        """
        resp = PostureRecordStartResponse()
        resp.error_code.val = PostureRecordErrorCodes.SUCCESS
        # check if there are ongoing recordings for the request groups
        ongoing = False
        for group in req.group_names:
            if group in self._current_postures:
                rospy.logwarn("Group %s is still in record mode,\
                              stop it first", group)
                ongoing = True

        if ongoing:
            resp.error_code.val = PostureRecordErrorCodes.NOTSTOPPED
        else:
            for group_name in req.group_names:
                if group_name not in self._sub:
                    try:
                        self.set_up_subscriber(group_name)
                        # give some time for the subscriber
                        # to receive its first data
                        rospy.sleep(0.5)
                    except rospy.ROSException:
                        rospy.logerr("Could not set up subscriber \
                                     for group %s.", group_name)
                        resp.error_code.val =\
                            PostureRecordErrorCodes.NOCONTROLLER
                        break
                # create a new traj for this group
                self._current_postures[group_name] = JointTrajectory()
                # set joint_names
                joint_names = self.get_joint_names(group_name)
                if joint_names is not None:
                    self._current_postures[group_name].joint_names = \
                        joint_names
                    continue
                else:
                    rospy.logerr("No joint_names for group %s.", group_name)
                    resp.error_code.val = PostureRecordErrorCodes.NOSTATE
                    break
                # clear the overall time from start
                self._overall_time_from_start = 0.0

            if resp.error_code.val != PostureRecordErrorCodes.SUCCESS:
                # clear current_postures as we failed
                # to start recording for some groups
                self._current_postures = {}

        return resp

    def stop_recording_cb(self, req):
        """
        Callback for the stop recording service
        """
        posture_name = req.posture_name
        resp = PostureRecordStopResponse()
        resp.error_code.val = PostureRecordErrorCodes.SUCCESS
        group_to_clear = []
        for group in self._current_postures:
            if not self._mp.add_posture(group, posture_name,
                                        self._current_postures[group]):
                rospy.logerr("Could not record current posture\
                             for group %s with name %s", group, posture_name)
                resp.error_code.val = PostureRecordErrorCodes.FAILURE
            else:
                group_to_clear.append(group)
                self._overall_time_from_start = 0.0
        for group in group_to_clear:
            self._current_postures.pop(group)
        return resp

    def add_waypoint_cb(self, req):
        """
        Callback for the add waypoint service
        """
        resp = PostureRecordAddWaypointResponse()
        resp.error_code.val = PostureRecordErrorCodes.SUCCESS
        resp.waypoint_count = []
        resp.overall_time_from_start = 0.0
        wp_nb = []
        max_time_from_start = 0.0

        if len(req.waypoint.group_names) != len(req.waypoint.time_from_start):
            resp.error_code.val = PostureRecordErrorCodes.INVALID
            return resp

        # for requested groups, check existance and way point first
        # so if it fails, we don't end up in an intermediate state
        wp = {}
        for group_name in req.waypoint.group_names:
            # if exists
            if group_name not in self._current_postures:
                rospy.logerr("Group %s was not started, start it first",
                             group_name)
                resp.error_code.val = PostureRecordErrorCodes.NOTSTARTED
                break
            # get current state as waypoint
            wp[group_name] = self.get_current_point(group_name)
            if wp[group_name] is None:
                rospy.logerr("Could not get current state for group %s, \
                              not adding any waypoint", group_name)
                resp.error_code.val = PostureRecordErrorCodes.NOSTATE
                break

        # if got all the way points
        if resp.error_code.val == PostureRecordErrorCodes.SUCCESS:
            for i, group_name in enumerate(req.waypoint.group_names):

                if max_time_from_start < req.waypoint.time_from_start[i]:
                    max_time_from_start = req.waypoint.time_from_start[i]

                # set time_from_start
                wp[group_name].time_from_start = rospy.Duration.from_sec(self._overall_time_from_start) + rospy.Duration.from_sec(req.waypoint.time_from_start[i])
                # store it
                self._current_postures[group_name].points.append(deepcopy(wp[group_name]))
                wp_nb.append(len(self._current_postures[group_name].points))

            #print self._current_postures
            # add the maximum time to overall time
            self._overall_time_from_start += max_time_from_start
            resp.overall_time_from_start = self._overall_time_from_start
            resp.waypoint_count = wp_nb

        return resp

    def save_cb(self, req):
        """
        Callback for the save recording service
        """
        resp = PostureRecordSaveResponse()
        resp.error_code.val = PostureRecordErrorCodes.SUCCESS
        if len(self._current_postures.keys()) > 0:
            rospy.logwarn("There are currently unstopped recordings, \
                          did you stop all of them ?")
            resp.error_code.val = PostureRecordErrorCodes.NOTSTOPPED
        else:
            self._mp.save_postures(req.filepath, req.strategy)
            self._mp.clear_postures()
        return resp

    def getall_cb(self, req):
        """
        Callback for the get all postures service (included the non stored ones)
        """
        resp = PostureRecordGetAllPosturesResponse()
        resp.error_code.val = PostureRecordErrorCodes.SUCCESS
        posture_dict = self._mp.list_postures()
        selected_groups_dict = {}
        # invert the dictionnary
        for group in posture_dict:
            for posture_name in posture_dict[group]:
                if posture_name not in selected_groups_dict:
                    selected_groups_dict[posture_name] = []
                selected_groups_dict[posture_name].append(group)
                
        # process stored postures
        for posture_name in selected_groups_dict:
            overall_time_from_start = 0.0
            posture = PostureRecordPosture()
            posture.posture_name = str(posture_name)
            posture.selected_groups = selected_groups_dict[posture_name]
            
            no_more_waypoint = False
            waypoint_idx = 0
            while(not no_more_waypoint):
                no_more_waypoint = True
                wp = PostureRecordWaypoint()
                for group_name in posture.selected_groups:
                    joint_traj = self._mp.get_posture(group_name, posture_name)
                    # is there a waypoint at this index ?
                    if waypoint_idx < len(joint_traj.points):
                        no_more_waypoint = False
                        wp.group_names.append(group_name)
                        wp.time_from_start.append(joint_traj.points[waypoint_idx].time_from_start.to_sec())
                
                # only the last max will be used
                
                if(not no_more_waypoint):
                    #print "ovt:", overall_time_from_start, " max wp t:",max(wp.time_from_start)
                    if overall_time_from_start < max(wp.time_from_start):
                        overall_time_from_start = max(wp.time_from_start)
                    posture.waypoints.append(deepcopy(wp))
                waypoint_idx += 1
            
            resp.postures.append(posture)
            resp.overall_time_from_start.append(overall_time_from_start)
            
        # process non stored postures
        if self._current_postures:
            overall_time_from_start = 0.0
            posture = PostureRecordPosture()
            posture.posture_name = str("unstored")
            posture.selected_groups = list(self._current_postures.keys())
            no_more_waypoint = False
            waypoint_idx = 0
            while(not no_more_waypoint):
                no_more_waypoint = True
                wp = PostureRecordWaypoint()
                for group_name in self._current_postures:
                    joint_traj = self._current_postures[group_name]
                    # is there a waypoint at this index ?
                    if waypoint_idx < len(joint_traj.points):
                        no_more_waypoint = False
                        wp.group_names.append(group_name)
                        wp.time_from_start.append(joint_traj.points[waypoint_idx].time_from_start.to_sec())
                # only the last max will be used
                if(not no_more_waypoint):
                    if overall_time_from_start < max(wp.time_from_start):
                        overall_time_from_start = max(wp.time_from_start)
                    posture.waypoints.append(deepcopy(wp))
                waypoint_idx += 1
            resp.postures.append(posture)
            resp.overall_time_from_start.append(overall_time_from_start)
        return resp

    def state_cb(self, msg, group_name):
        """
        Callback to handle new state from the running controllers
        """
        self._state[group_name] = msg

    def get_joint_names(self, group_name):
        """
        Retrieve the joint_names for the given group
        requires a running controller
        @param group_name: name of the group to retrieve the joints for
        """
        if group_name in self._state:
            return self._state[group_name].joint_names
        else:
            return None

    def get_current_point(self, group_name):
        """
        Retrieve the current traj point for the given group
        requires a running controller
        @param group_name: name of the group to retrieve the point for
        """
        if group_name in self._state:
            point = self._state[group_name].actual
            # set velocity to empty
            point.velocities = []
            return point
        else:
            return None


def main():

    meka_posture_recorder = MekaPostureRecorder('meka_posture_recorder')
    rospy.spin()

if __name__ == "__main__":
    main()
