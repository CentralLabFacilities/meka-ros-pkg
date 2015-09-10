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

from python_qt_binding.QtCore import QSize

from QtCore import Qt, QThread, SIGNAL, QObject
from QtGui import QWidget, QDoubleSpinBox, QStandardItemModel, QStandardItem, QTableView, QTreeWidget, QTreeWidgetItem, QCheckBox, QFileDialog, QMessageBox, QPushButton, QFrame, QHBoxLayout, QVBoxLayout
from functools import partial

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllersRequest, \
    ListControllers

from functools import partial

from meka_posture.meka_posture import MekaPosture

from meka_posture_recorder.msg import PostureRecordErrorCodes, PostureRecordWaypoint
from meka_posture_recorder.srv import PostureRecordStart, PostureRecordStop,\
    PostureRecordSave, PostureRecordAddWaypoint, PostureRecordStartRequest,\
    PostureRecordStopRequest, PostureRecordAddWaypointRequest,\
    PostureRecordSaveRequest, PostureRecordGetAllPostures,\
    PostureRecordGetAllPosturesRequest

JNT_TRAJ_SRV_SUFFIX = "_position_trajectory_controller/follow_joint_trajectory"

class MekaPostureEditorGUI(Plugin):
    """
    a rqtgui plugin for the meka posture editor
    """

    def __init__(self, context):
        Plugin.__init__(self, context)
        self.setObjectName('RqtMekaPostureEditor')

        self._widget = QWidget()
        self._filemodel = {}
        self._table_view = {}
        self._vh = {}
        self._file_path = None

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_meka_posture_editor'), 'ui', 'RqtMekaPostureEditor.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtMekaPostureEditorUi')
        context.add_widget(self._widget)

        self._cm_prefix = "meka_roscontrol"
        self._client = {}
        
        # find the existing groups (running controllers)
        self._group_list = []
        self.init_group_list()
        self._current_posture_dict = {}
        self._posture_unstored = {}
        self._posture_modified = {}
        self._table_initialized = {}
        self._current_posture_name = None
        
        nb_group = len(self._group_list)
        if nb_group == 0:
            rospy.logerr("No controllers available.")
            return
        
        self._postureCount = 0
        self._verticalHeader = []
        
        # add a one column table per group
        
        all_group_layout = QHBoxLayout()
        
        for group_name in self._group_list:
            self._filemodel[group_name] = QStandardItemModel(0, 1)
            self._filemodel[group_name].setHorizontalHeaderLabels([group_name])
            self._table_view[group_name] = QTableView()
            self._table_view[group_name].setModel(self._filemodel[group_name])
            self._table_view[group_name].resizeColumnsToContents()
            self._vh[group_name] = self._table_view[group_name].verticalHeader()
            self._table_view[group_name].connect(self._vh[group_name], SIGNAL("sectionResized(int,int,int)"), partial(self.on_row_resized, group_name))
            all_group_layout.addWidget(self._table_view[group_name])
                
        self._widget.scrollarea.setLayout(all_group_layout)
        
        #temporary set a value there
        self._widget.edit_file_path.setText("/home/meka/workspace/mekabot/meka-ros-pkg/meka_posture_execution/cfg/postures.yml")
        
        self._widget.btn_load_postures.clicked.connect(self.on_load_postures_clicked)
        self._widget.btn_save_postures.clicked.connect(self.on_save_postures_clicked)
        self._widget.btn_execute.clicked.connect(self.on_execute_clicked)
        self._widget.btn_stop.clicked.connect(self.on_stop_clicked)
        self._widget.btn_undo_mod.clicked.connect(self.on_undo_modification_clicked)
        self._widget.btn_browse.clicked.connect(self.on_btn_file_path_clicked)
        
        self._widget.treeWidget.itemClicked.connect(self.on_posture_clicked)
        
        
        self._mp = MekaPosture("myposture")

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

    def init_action_client(self, group_name):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        rospy.loginfo("connecting to %s", (self._cm_prefix + "/"+ group_name + JNT_TRAJ_SRV_SUFFIX))
        self._client[group_name] = SimpleActionClient(
            self._cm_prefix + "/"+ group_name + JNT_TRAJ_SRV_SUFFIX,
            FollowJointTrajectoryAction
        )

        if self._client[group_name].wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to %s action server in 4 sec",group_name)
            raise

    def populate_posture_list(self):
        self._widget.treeWidget.clear()
        postures_item = QTreeWidgetItem(["posture"])
        self._widget.treeWidget.addTopLevelItem(postures_item)
        
        group_dict = self._mp.list_postures()
        # invert the dictionnary
        for group in group_dict:
            for posture_name in group_dict[group]:
                if posture_name not in self._current_posture_dict:
                    self._current_posture_dict[posture_name] = []
                self._current_posture_dict[posture_name].append(group)
        
        
        for posture in self._current_posture_dict:
            posture_item = QTreeWidgetItem(postures_item, [posture])
            font = posture_item.font(0)
            self._posture_modified[posture] = False
            self._posture_unstored[posture] = False

        self._widget.treeWidget.addTopLevelItem(posture_item)
        postures_item.setExpanded(True)

    def update_posture_list(self):
        postures_modified = False
        posture_items = self._widget.treeWidget.topLevelItem(0)
        for i in range(posture_items.childCount()):
            posture_item = posture_items.child(i)
            font = posture_item.font(0)
            if self._posture_modified[posture_item.text(0)]:
                postures_modified = True
                font.setBold(True)
            else:
                font.setBold(False)
            posture_item.setFont(0,font)
        
        font = posture_items.font(0)
        if postures_modified:
            posture_items.setText(0,"postures*")
            font.setBold(True)
        else:
            posture_items.setText(0,"postures")
            font.setBold(False)
        posture_items.setFont(0,font)

    def on_row_resized(self, group_name, *args):
        if self._table_initialized[group_name]:
            self.update_table(group_name, args[0], self.time_from_row_height(args[2]))
    
    def on_posture_clicked(self, item):
        if item.childCount() == 0:
            posture_name = item.text(0)
            if self._current_posture_name == None:
                self._current_posture_name = posture_name
               
            if (self._posture_modified[self._current_posture_name] and self._posture_unstored[self._current_posture_name]):
                # save the modified data first
                if self.store_posture_mods(self._current_posture_name):
                    # store current edited posture
                    self._current_posture_name = posture_name
                else:
                    #TODO reselect previous posture and warn user
                    QMessageBox.warning(self._widget.treeWidget, "Posture storage problem", "Unable to store the changes")
                    return
            else:
                self._current_posture_name = posture_name
                    
            # get posture data
            if posture_name in self._current_posture_dict:
                # clear all tables first
                for group_name in self._filemodel:
                    fm = self._filemodel[group_name]
                    fm.clear()
                    fm.setHorizontalHeaderLabels([group_name])
                    if group_name in self._current_posture_dict[posture_name]:
                        jnt_traj = self._mp.get_posture(group_name, posture_name)
                        self.populate_table(group_name, jnt_traj)     

    def store_posture_mods(self, current_posture_name):
        """
        update the internal storage to new values
        """
        for group_name in self._filemodel:
            fm = self._filemodel[group_name]
            if group_name in self._current_posture_dict[current_posture_name]:
                jnt_traj = self._mp.get_posture(group_name, current_posture_name)
                self.update_time_from_table(group_name, jnt_traj)
                if self._mp.add_posture(group_name, current_posture_name, jnt_traj, strategy="overwrite"):
                    self._posture_unstored[self._current_posture_name] = False
                    continue
                else:
                    self._posture_unstored[self._current_posture_name] = True
                    return False
        return True
            
                

    def populate_table(self, group_name, jnt_traj):
        """
        update table
        """
        if group_name in self._filemodel:
            fm = self._filemodel[group_name]
            overall_time_from_start = 0.0
            self._table_initialized[group_name] = False
            for i, waypoint in enumerate(jnt_traj.points):
                new_item = QStandardItem(str(round(waypoint.time_from_start.to_sec()-overall_time_from_start,1)))
                new_item.setEditable(False)
                fm.setItem(i, 0, new_item)
                self._table_view[group_name].setRowHeight(i, self.compute_row_height(waypoint.time_from_start.to_sec()-overall_time_from_start))
                overall_time_from_start = waypoint.time_from_start.to_sec()
            self._table_initialized[group_name] = True
                
    def update_table(self, group_name, idx, time_from_start):
        if group_name in self._filemodel:
            fm = self._filemodel[group_name]
            item_idx = fm.index(idx, 0)
            fm.setData(item_idx, str(time_from_start))
            # data modified and not stored in the internal model yet
            self._posture_unstored[self._current_posture_name] = True
            # data modified
            if not self._posture_modified[self._current_posture_name]:
                # update the tree view to mark the current posture as modified
                self._posture_modified[self._current_posture_name] = True
                self.update_posture_list()
                
    
    def update_time_from_table(self, group_name, traj):
        """
        update joint trajectory points from table
        """
        overall_time_from_start = 0.0
        fm = self._filemodel[group_name]
        for i, waypoint in enumerate(traj.points):
            item_idx = fm.index(i, 0)
            data = fm.data(item_idx)
            new_time_from_start = overall_time_from_start + round(float(data),1)
            waypoint.time_from_start = rospy.Duration.from_sec(new_time_from_start)
            overall_time_from_start = new_time_from_start
        
      
    def time_from_row_height(self, row_height):
        return round((row_height-20)/10.0,1)
      
    def compute_row_height(self, time_from_start):
        height = 20.0 # default
        height += round(time_from_start*10.0,0)
        return height
        
      
    def reset_file_path(self):
        """
        Clear the chosen file path and disable the save button until user selects another path
        """
        self._widget.edit_file_path.setText("")
        self._widget.btn_save_postures.setEnabled(False)

    def on_btn_file_path_clicked(self):
        """
        File browser
        """
        if self._file_path:
            path_to_config = self._file_path
        elif self._widget.edit_file_path.text(): 
            path_to_config = self._widget.edit_file_path.text()
        else:
            path_to_config = "/home/meka/workspace/mekabot/meka-ros-pkg/meka_posture_execution/cfg/postures.yml"
        
        filter_files = "YAML (*.yaml *.yml)"

        filename, _ = QFileDialog.getOpenFileName(self._widget.btn_browse, self._widget.tr('Save Controller Settings'),
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
            QMessageBox.warning(self._widget.btn_save_postures, "Required field", "No file name given")
            return
        
        self.save_all(file_name)
        # reset modified postures
        for posture in self._current_posture_dict:
            self._posture_modified[posture] = False
        self.update_posture_list()
        ok_message =  "Saved"
        QMessageBox.information(self._widget.btn_save_postures, "Success", ok_message)
        
    def save_all(self, file_name):
        
        # store the current posture
        if not self.store_posture_mods(self._current_posture_name):
            QMessageBox.warning(self._widget.btn_save_postures, "Internal storage problem", "Cannot update current posture")
            return 

        self._mp.save_postures(file_name, strategy="overwrite")
        

    def on_load_postures_clicked(self):
        """
        load
        """
        
        # check for modified postures
        for posture in self._current_posture_dict:
            if self._posture_modified[posture]:
                QMessageBox.warning(self._widget.btn_load_postures, "Load stopped", "Posture were modified, save them first")
                return
        
        file_name = self._widget.edit_file_path.text()
        if not file_name:
            QMessageBox.warning(self._widget.btn_load_postures, "Required field", "No file name given")
            return
            

        self._mp.append_postures(file_name, strategy="keep")
        self.populate_posture_list()
        
        
    def on_undo_modification_clicked(self):
        """
        undo
        """
        if self._current_posture_name is not None:
            # get posture data
            posture_name = self._current_posture_name
            # clear all tables first
            for group_name in self._filemodel:
                fm = self._filemodel[group_name]
                fm.clear()
                fm.setHorizontalHeaderLabels([group_name])
                if group_name in self._current_posture_dict[posture_name]:
                    jnt_traj = self._mp.get_posture(group_name, posture_name)
                    self.populate_table(group_name, jnt_traj)
                    self._posture_unstored[self._current_posture_name] = False
            self.update_posture_list()
    
    def on_execute_clicked(self):
        """
        execute selected movements
        """
        if not self._posture_unstored[self._current_posture_name]:
            for group_name in self._current_posture_dict[self._current_posture_name]:
                #TODO change header of group to bold until movement is finished (use done_cb())
                self.execute(group_name, self._current_posture_name)
        else:
            rospy.logwarn("Unstored values in current posture, valide the modification first before executing")
            
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
            goal = self._mp.get_trajectory_goal(group_name, posture_name)
            if goal is not None:
                if timescale != 1.0:
                    # rescale the time_from_start for each point
                    self.rescale_time_from_start(goal, timescale)

                if group_name not in self._client:
                    rospy.logerr("Action client for %s not initialized. Trying to initialize it...", group_name)
                    try:
                        self.init_action_client(group_name)
                    except:
                        rospy.logerr("Could not set up action client for %s.", group_name)
                        return
                self._client[group_name].send_goal(goal)
            else:
                rospy.logerr("No goal found for posture %s in group  %s.", posture_name, group_name)
        
    def on_stop_clicked(self):
        """
        stop all movements
        """
        for group_name in self._current_posture_dict[self._current_posture_name]:
            self.stop_movement(group_name)
        
    def stop_movement(self, group_name):
        if group_name in self._client:
            self._client[group_name].cancel_goal()
        
    def shutdown_plugin(self):
        for posture in self._current_posture_dict:
            if self._posture_modified[posture]:
                ret = QMessageBox.question(self._widget, "Quit", "Posture were modified, do you want to save them before leaving ?", "Save", "Do Not Save")
                if ret == 0:
                    file_name = self._widget.edit_file_path.text()
                    if not file_name:
                        file_name = "postures.bak"
                    self.save_all(file_name)
                
        
        
    #########
    # Default methods for the rqtgui plugins

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
