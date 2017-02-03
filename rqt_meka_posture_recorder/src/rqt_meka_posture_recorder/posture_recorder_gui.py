#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# derived from sr_gui_controller_tuner
#   original Authors Shadow Robot Team
#
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# author Guillaume WALCK (2015)

import rospy
import rospkg
import os
from copy import deepcopy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from PyQt5.QtWidgets import *

from functools import partial

from controller_manager_msgs.srv import ListControllersRequest, \
    ListControllers
    
from meka_posture_recorder.msg import PostureRecordErrorCodes, PostureRecordWaypoint
from meka_posture_recorder.srv import PostureRecordStart, PostureRecordStop,\
    PostureRecordSave, PostureRecordAddWaypoint, PostureRecordStartRequest,\
    PostureRecordStopRequest, PostureRecordAddWaypointRequest,\
    PostureRecordSaveRequest, PostureRecordGetAllPostures,\
    PostureRecordGetAllPosturesRequest


class MekaPostureRecorderGUI(Plugin):
    """
    a rqtgui plugin for the meka posture recorder
    """

    def __init__(self, context):
        Plugin.__init__(self, context)
        self.setObjectName('RqtMekaPostureRecorder')

        self._widget = QWidget()
        self._group_check_box = {}
        self._group_spinbox = {}

        self._file_path = None

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_meka_posture_recorder'), 'ui', 'RqtMekaPostureRecorder.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtMekaPostureRecorderUi')
        context.add_widget(self._widget)

        self._cm_prefix = "meka_roscontrol"
        self._recorder_name = "meka_posture_recorder"
        
        # find the existing groups (running controllers)
        self._group_list = []
        self.init_group_list()
        
        nb_group = len(self._group_list)
        if nb_group == 0:
            rospy.logerr("No controllers available.")
            return
        
        self._postureCount = 0
        self._verticalHeader = []
        
        # add a column and a checkbox per group
        
        hlayout = QHBoxLayout()
        vlayout = QVBoxLayout()
        all_group_layout = QHBoxLayout()
                
        self._tree_widget = QTreeWidget()
        self._tree_widget.setColumnCount(nb_group + 1)
       
        self._tree_widget.setHeaderLabels(["    "] + self._group_list)
        

        group_item = QTreeWidgetItem(["group"])
        self._tree_widget.addTopLevelItem(group_item)
        time_item = QTreeWidgetItem(["time_from_start"])
        self._tree_widget.addTopLevelItem(time_item)
        
        for index_item, group_name in enumerate(self._group_list):
            check_box = QCheckBox("enable")
            self._tree_widget.setItemWidget(group_item, index_item + 1, check_box)
            self._group_check_box[group_name] = check_box
            spinbox = QDoubleSpinBox()
            spinbox.setValue(2.0)
            spinbox.setSingleStep(0.5)
            spinbox.setMinimum(0.0)
            self._tree_widget.setItemWidget(time_item, index_item + 1, spinbox)
            self._group_spinbox[group_name] = spinbox

        self._postures_item = QTreeWidgetItem(["postures"])
        self._tree_widget.addTopLevelItem(self._postures_item)

        vlayout.addWidget(self._tree_widget)
        hlayout.addLayout(vlayout)
        self._widget.scrollarea.setLayout(hlayout)


        self._client = {}
        self.init_services("start_record", PostureRecordStart)
        self.init_services("stop_record", PostureRecordStop)
        self.init_services("save", PostureRecordSave)
        self.init_services("add_waypoint", PostureRecordAddWaypoint)
        self.init_services("get_all_postures", PostureRecordGetAllPostures)
        
        self._widget.btn_new_posture.clicked.connect(self.on_new_posture_clicked)
        self._widget.btn_save_postures.clicked.connect(self.on_save_postures_clicked)
        self._widget.btn_add_wp.clicked.connect(self.on_add_wp_clicked)
        self._widget.btn_store_posture.clicked.connect(self.on_store_posture_clicked)
        self._widget.btn_browse.clicked.connect(self.on_btn_file_path_clicked)
        
        self._widget.chk_select_all.stateChanged.connect(self.on_check_all_changed)
        self._widget.time_from_start.valueChanged.connect(self.on_time_from_start_changed)
        
        self.init_postures()

    def init_group_list(self):
        """
        Get all the groups from running controllers
        """
        service_name = self._cm_prefix + \
            '/controller_manager/list_controllers'
        rospy.loginfo("Waiting for %s", service_name)
        controller_service_found = False
        try:
            rospy.wait_for_service(service_name, 5.0)
            cm_list_client = rospy.ServiceProxy(service_name,
                                            ListControllers)
            rospy.loginfo("Found %s", service_name)
            controller_service_found = True
            
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)
            
        resp = False
        if controller_service_found:
            # get all the controllers
            try:
                resp = cm_list_client(ListControllersRequest())
            except rospy.ServiceException:
                rospy.logerr("Could not call list_controllers")

            if resp:
                for controller in resp.controller:
                    cname_split = controller.name.split("_")
                    if len(cname_split) > 1:
                        if cname_split[0] in ["torso", "zlift", "head"]:
                            self._group_list.append(cname_split[0])
                        else:
                            if len(cname_split) > 2:
                                if cname_split[1] in ["arm", "hand"]:
                                    group_name = cname_split[0] + "_" +\
                                        cname_split[1]
                                    self._group_list.append(group_name)

    def init_services(self, service_name, service_type):
        """
        Initialize the service client of name service_name
        """
        full_service_name = self._recorder_name + '/' + service_name
        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(full_service_name, 2.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", full_service_name)
            return False
        self._client[service_name] = rospy.ServiceProxy(full_service_name,
                                                service_type)

    def init_postures(self):
      
        is_unstored = False
        idx_unstored = 0
        # get postures from recorder
        if "get_all_postures" in self._client:
            req = PostureRecordGetAllPosturesRequest()
            resp = self._client["get_all_postures"](req)
            
            
            for i, posture in enumerate(resp.postures):
                #print "-------------------",posture.posture_name
                overall_time_from_start = 0.0
                self.add_new_posture(posture.selected_groups, posture.posture_name)
                # find unstored
                if posture.posture_name == "unstored":
                    is_unstored = True
                    unstored_selected_groups = posture.selected_groups
                # compute overall time
                postponed_group_names = []
                postponed_time_from_start = []
                more_waypoints = True
                idx_wp = 0
                while more_waypoints:
                    # get current waypoint if any left
                    if idx_wp < len(posture.waypoints):
                        process_group_names = list(posture.waypoints[idx_wp].group_names)
                        process_time_from_start = list(posture.waypoints[idx_wp].time_from_start)
                        idx_wp += 1
                    else:
                        process_group_names = []
                        process_time_from_start = []

                    
                    # get the postponed elements    
                    process_group_names += postponed_group_names
                    process_time_from_start += postponed_time_from_start
                    
                    if len(process_group_names) == 0:
                        more_waypoints = False
                        continue
                        
                    postponed_group_names = []
                    postponed_time_from_start = []
                    #print idx_wp, " ", process_group_names, process_time_from_start
                    
                    # process the elements
                    while (len(process_group_names)>0):
                        # find min and max
                        max_time = max(process_time_from_start)
                        idx_max = process_time_from_start.index(max_time)
                        min_time = min(process_time_from_start)
                        idx_min = process_time_from_start.index(min_time)
                        count_min = process_time_from_start.count(min_time)
                        
                        #print "idx_min",idx_min, count_min
                        if (min_time == max_time):
                            #print max_time, " overall", resp.overall_time_from_start[i]
                            #print idx_wp ," < ", len(posture.waypoints)-1 
                            # if max is the overall_time and there are other point to process, postpone max
                            if (max_time == resp.overall_time_from_start[i] and idx_wp < len(posture.waypoints)-1 ): 
                                postponed_group_names.append(process_group_names[idx_max])
                                postponed_time_from_start.append(process_time_from_start[idx_max])
                                process_group_names.pop(idx_max)
                                process_time_from_start.pop(idx_max)
                                #print "postpone"
                            else:
                                # identical time, add all in once
                                self.add_waypoint(process_group_names, process_time_from_start, overall_time_from_start)
                                next_overall_time_from_start = max(process_time_from_start)
                                #print "add all",  process_group_names, "ovt ", overall_time_from_start
                                break
                        else:
                            # separate in postponed and min list to add
                            min_group_names = []
                            min_time_from_start = []
                            for g,t in zip(process_group_names, process_time_from_start):
                                if t != min_time:
                                    postponed_group_names.append(g)
                                    postponed_time_from_start.append(t)
                                else:
                                    min_group_names.append(g)
                                    min_time_from_start.append(t)
                          
                            # add only smallest time and loop
                            self.add_waypoint(min_group_names, min_time_from_start, overall_time_from_start)
                            # do not set time as long
                            if overall_time_from_start < min_time:
                                next_overall_time_from_start = min_time
                            
                            #print "add min" ,min_group_names, "ovt ", overall_time_from_start
                            # clean the list
                            break

                    overall_time_from_start = next_overall_time_from_start
                    #overall_time_from_start = max(waypoint.time_from_start)
                    
                    
                if overall_time_from_start != resp.overall_time_from_start[i]:
                    rospy.logwarn("inconsistency in posture %s overalltime", posture.posture_name)
        if is_unstored:
            # select groups that were initially started for the unstored posture
            for group in self._group_list:
                if group in unstored_selected_groups:
                    self._group_check_box[group].setChecked(True)
                else:
                    self._group_check_box[group].setChecked(False)
            
            self._widget.btn_new_posture.setEnabled(False)
            self._widget.btn_store_posture.setEnabled(True)
            self._widget.btn_add_wp.setEnabled(True)
        else:
            self._widget.btn_new_posture.setEnabled(True)
            self._widget.btn_store_posture.setEnabled(False)
            self._widget.btn_add_wp.setEnabled(False)

    def reset_file_path(self):
        """
        Clear the chosen file path and disable the save button until user selects another path
        """
        self._widget.edit_file_path.setText("")
        self._widget.btn_save_postures.setEnabled(False)

    def on_check_all_changed(self):
        for group in self._group_list:
            self._group_check_box[group].setCheckState(self._widget.chk_select_all.checkState())
    
    def on_time_from_start_changed(self):
        for group in self._group_spinbox:
            self._group_spinbox[group].setValue(self._widget.time_from_start.value()) 

    def on_btn_file_path_clicked(self):
        """
        File browser
        """
        if self._file_path:
            path_to_config = self._file_path
        else:
            path_to_config = "~"
        
        filter_files = "*.yaml"

        filename, _ = QFileDialog.getSaveFileName(self._widget.btn_browse, self._widget.tr('Save Controller Settings'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self._file_path = filename

        self._widget.edit_file_path.setText(filename)

        self._widget.btn_save_postures.setEnabled(True)


    def on_save_postures_clicked(self):
        """
        Save 
        """
        file_name = self._widget.edit_file_path.text()
        if not file_name:
            QMessageBox.warning(self._widget.btn_store_posture, "Required field", "No file name given")
            return
            
        req = PostureRecordSaveRequest()
        req.filepath = file_name
        # TODO read a checkbox for strategy overwrite
        req.strategy = "append"
        try:
            resp = self._client["save"](req)
            if resp.error_code.val == PostureRecordErrorCodes.NOTSTOPPED:
                warn_message =  "Stop the recording first"
                QMessageBox.warning(self._widget.btn_save_postures, "Failure", warn_message)
                return
            else:
                ok_message =  "Saved"
                QMessageBox.information(self._widget.btn_save_postures, "Success", ok_message)
        except rospy.ServiceException:
            rospy.logerr("Cannot call start_record")
            warn_message =  "Cannot save"
            QMessageBox.warning(self._widget.btn_save_postures, "Failure", warn_message)


    def on_new_posture_clicked(self):
        """
        start
        """
        start_group = []
        for group in self._group_list:
            if self._group_check_box[group].checkState():
                start_group.append(group)
        if len(start_group) == 0:
            QMessageBox.warning(self._widget.btn_new_posture, "Failure", "No group selected")
            return
        req = PostureRecordStartRequest()
        req.group_names = start_group

        try:
            resp = self._client["start_record"](req)
            if resp.error_code.val == PostureRecordErrorCodes.NOSTATE:
                warn_message =  "Missing state"
                QMessageBox.warning(self._widget.btn_new_posture, "Failure", warn_message)
                return
            if resp.error_code.val == PostureRecordErrorCodes.NOTSTOPPED:
                warn_message =  "Cannot start, store current recording first"
                QMessageBox.warning(self._widget.btn_new_posture, "Failure", warn_message)
                self._widget.btn_new_posture.setEnabled(False)
                self._widget.btn_add_wp.setEnabled(True)
                self._widget.btn_store_posture.setEnabled(True)
                return
            else:
                self.add_new_posture(start_group)

                self._widget.btn_new_posture.setEnabled(False)
                self._widget.btn_store_posture.setEnabled(True)
                self._widget.btn_add_wp.setEnabled(True)
        except rospy.ServiceException:
            rospy.logerr("Cannot call start_record")

    def add_new_posture(self, start_group, posture_name=None):
        if posture_name is None:
            posture_header = ["unknown" + str(self._postures_item.childCount())]
        else:
            posture_header = [posture_name]

        for group in self._group_list:
            if group in start_group:
                header = ["selected"]
            else:
                header = ["-----"]
            posture_header += header
        self._current_posture_item = QTreeWidgetItem(self._postures_item, posture_header)
        self._tree_widget.addTopLevelItem(self._current_posture_item)
        self._postures_item.setExpanded(True)

    def on_add_wp_clicked(self):
        """
        add
        """
        add_group = []
        time_from_previous = []
        for group in self._group_list:
            if self._group_check_box[group].checkState():
                add_group.append(group)
                time_from_previous.append(self._group_spinbox[group].value())
        
        req = PostureRecordAddWaypointRequest()
        req.waypoint.group_names = add_group
        req.waypoint.time_from_start = time_from_previous
        try:
            resp = self._client["add_waypoint"](req)
            if resp.error_code.val == PostureRecordErrorCodes.NOTSTARTED:
                warn_message =  "One group was not started"
                QMessageBox.warning(self._widget.btn_add_wp, "Failure", warn_message)
                return
            if resp.error_code.val == PostureRecordErrorCodes.NOSTATE:
                warn_message =  "Missing state"
                QMessageBox.warning(self._widget.btn_add_wp, "Failure", warn_message)
                return
            self.add_waypoint(add_group, time_from_previous, 0.0)

        except rospy.ServiceException:
            rospy.logerr("Cannot call add waypoint")

    def add_waypoint(self, selected_groups, time_from_start, previous_overall_time_from_start):
        header = []
        header += ["wp " + str(self._current_posture_item.childCount() + 1)]
        wp_idx = 0
        for i, group in enumerate(self._group_list):
            if group in selected_groups:
                header += [str(time_from_start[wp_idx]-previous_overall_time_from_start)]
                wp_idx += 1
            else:
                header += ["--"]


        waypoint_item = QTreeWidgetItem(self._current_posture_item, header)
        self._tree_widget.addTopLevelItem(waypoint_item)
        self._current_posture_item.setExpanded(True)
      
    def on_store_posture_clicked(self):
        """
        stop record
        """

        req = PostureRecordStopRequest()
        posture_name = self._widget.edit_posture_name.text()
        if not posture_name:
            QMessageBox.warning(self._widget.btn_store_posture, "Required field", "required posture name is empty")
            return
        
        req.posture_name = posture_name
        try:
            resp = self._client["stop_record"](req)
            if resp.error_code.val == PostureRecordErrorCodes.SUCCESS:
                #row_count = self._filemodel.rowCount()
                item = self._postures_item.child(self._postures_item.childCount() - 1)
                if item:
                    item.setText(0, posture_name)
                
                self._widget.btn_new_posture.setEnabled(True)
                self._widget.btn_store_posture.setEnabled(False)
                self._widget.btn_add_wp.setEnabled(False)
            else:
                warn_message = "Could not record, probably posture name " + posture_name + " already exists"
                QMessageBox.warning(self._widget.btn_store_posture, "Failure", warn_message)
            
        except rospy.ServiceException:
            rospy.logerr("Cannot call stop record")


    #########
    # Default methods for the rqtgui plugins

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
