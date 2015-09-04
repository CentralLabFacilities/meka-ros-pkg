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

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from python_qt_binding.QtCore import QSize

from QtCore import Qt, QThread
from QtGui import QWidget, QDoubleSpinBox, QStandardItemModel, QStandardItem, QTableView, QCheckBox, QFileDialog, QMessageBox, QPushButton, QFrame, QHBoxLayout, QVBoxLayout
from functools import partial

from controller_manager_msgs.srv import ListControllersRequest, \
    ListControllers
    
from meka_posture_recorder.msg import PostureRecordErrorCodes
from meka_posture_recorder.srv import PostureRecordStart, PostureRecordStop,\
    PostureRecordSave, PostureRecordAddWaypoint, PostureRecordStartRequest,\
    PostureRecordStopRequest, PostureRecordAddWaypointRequest,\
    PostureRecordSaveRequest


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
        
        # add a column and a checkbox per group
        
        hlayout = QHBoxLayout()
        vlayout = QVBoxLayout()
        all_group_layout = QHBoxLayout()
        
        self._filemodel = QStandardItemModel(0, nb_group)
        self._filemodel.setHorizontalHeaderLabels(self._group_list)
        self._table_view = QTableView()
        self._table_view.setModel(self._filemodel)
        self._table_view.resizeColumnsToContents()
        
        group_layout = {}
        for group_name in self._group_list:
            group_layout[group_name] = QVBoxLayout()
            self._group_check_box[group_name] = QCheckBox(group_name)
            group_layout[group_name].addWidget(self._group_check_box[group_name])
            self._group_spinbox[group_name] = QDoubleSpinBox()
            self._group_spinbox[group_name].setValue(2.0)
            self._group_spinbox[group_name].setSingleStep(0.5)
            self._group_spinbox[group_name].setMinimum(0.0)
            group_layout[group_name].addWidget(self._group_spinbox[group_name])
            all_group_layout.addLayout(group_layout[group_name])

        vlayout.addLayout(all_group_layout)
        vlayout.addWidget(self._table_view)
        hlayout.addLayout(vlayout)
        self._widget.scrollarea.setLayout(hlayout)


        self._client = {}
        self.init_services("start_record", PostureRecordStart)
        self.init_services("stop_record", PostureRecordStop)
        self.init_services("save", PostureRecordSave)
        self.init_services("add_waypoint", PostureRecordAddWaypoint)
        
        self._widget.btn_new_posture.clicked.connect(self.on_new_posture_clicked)
        self._widget.btn_save_postures.clicked.connect(self.on_save_postures_clicked)
        self._widget.btn_add_wp.clicked.connect(self.on_add_wp_clicked)
        self._widget.btn_store_posture.clicked.connect(self.on_store_posture_clicked)
        self._widget.btn_browse.clicked.connect(self.on_btn_file_path_clicked)
        
        self._widget.chk_select_all.stateChanged.connect(self.on_check_all_changed)
        self._widget.time_from_start.valueChanged.connect(self.on_time_from_start_changed)
        
        self._widget.btn_new_posture.setEnabled(True)
        self._widget.btn_store_posture.setEnabled(False)
        self._widget.btn_add_wp.setEnabled(False)


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
            rospy.wait_for_service(full_service_name, 5.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", full_service_name)
            return False
        self._client[service_name] = rospy.ServiceProxy(full_service_name,
                                                service_type)

    def reset_file_path(self):
        """
        Clear the chosen file path and disable the save button until user selects another path
        """
        self._widget.edit_file_path.setText("")
        self._widget.btn_save_postures.setEnabled(False)

    def on_check_all_changed(self):
        for group in self._group_list:
            self._group_check_box[group].setCheckState(self._widget.chk_select_all.checkState()) \
    
    def on_time_from_start_changed(self):
        for group in self._group_spinbox:
            self._group_spinbox[group].setValue(self._widget.time_from_start.value()) 

    def on_btn_file_path_clicked(self):
        """
        File browser
        """
        if self._file_path:
            path_to_config = self.file_path
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
                row_count = self._filemodel.rowCount()
                for i, group in enumerate(self._group_list):
                    if self._group_check_box[group].checkState():
                        new_item = QStandardItem("???:0")
                        new_item.setEditable(False)
                    else:
                        new_item = QStandardItem("----")
                        new_item.setEditable(False)
                    self._filemodel.setItem(row_count, i, new_item)
                    #self._filemodel.appendRow(new_item)
              
                self._widget.btn_new_posture.setEnabled(False)
                self._widget.btn_store_posture.setEnabled(True)
                self._widget.btn_add_wp.setEnabled(True)
        except rospy.ServiceException:
            rospy.logerr("Cannot call start_record")

    def on_add_wp_clicked(self):
        """
        add
        """
        add_group = []
        time_from_start = []
        for group in self._group_list:
            if self._group_check_box[group].checkState():
                add_group.append(group)
                time_from_start.append(self._group_spinbox[group].value())
        
        req = PostureRecordAddWaypointRequest()
        req.group_names = add_group
        req.time_from_start = time_from_start
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
            row_count = self._filemodel.rowCount()
            wp_idx = 0
            for i, group in enumerate(self._group_list):
                if self._group_check_box[group].checkState():
                    idx = self._filemodel.index(row_count-1, i)
                    self._filemodel.setData(idx, "???:"+str(resp.waypoints[wp_idx]))
                    wp_idx += 1
            
        except rospy.ServiceException:
            rospy.logerr("Cannot call add waypoint")

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
                row_count = self._filemodel.rowCount()
                wp_idx = 0
                for i, group in enumerate(self._group_list):
                    idx = self._filemodel.index(row_count-1, i)
                    data = self._filemodel.data(idx)
                    if str(data) != "----":
                        new_str = str(data)
                        new_str = new_str.replace("???", posture_name)
                        self._filemodel.setData(idx, new_str)  
                self._table_view.resizeColumnsToContents()
              
                self._widget.btn_new_posture.setEnabled(True)
                self._widget.btn_store_posture.setEnabled(False)
                self._widget.btn_add_wp.setEnabled(False)
            else:
                warn_message = "Could not record, probably posture name " + posture_name + " already exists"
                QMessageBox.warning(self._widget.btn_store_posture, "Failure", warn_message)
            
        except rospy.ServiceException:
            rospy.logerr("Cannot call stop record")


    def refresh_controller_tree_(self, controller_type="Motor Force"):
        """
        Get the controller settings and their ranges and display them in the tree.
        Buttons and plots will be added unless in edit_only mode.
        Move button will be added if controller is position type
        Buttons "set all" "set selected" and "stop movements" are disabled in edit_only_mode
        Controller settings must exist for every motor of every finger in the yaml file.
        """

        if self.sr_controller_tuner_app_.edit_only_mode:
            self._widget.btn_set_selected.setEnabled(False)
            self._widget.btn_set_all.setEnabled(False)
            self._widget.btn_stop_mvts.setEnabled(False)
        else:
            self._widget.btn_set_selected.setEnabled(True)
            self._widget.btn_set_all.setEnabled(True)
            self._widget.btn_stop_mvts.setEnabled(True)

        self.controller_type = controller_type
        ctrl_settings = self.sr_controller_tuner_app_.get_controller_settings(controller_type)

        self._widget.tree_ctrl_settings.clear()
        self._widget.tree_ctrl_settings.setColumnCount(ctrl_settings.nb_columns)
        # clear the ctrl_widgets as the motor name might change also now
        self.ctrl_widgets = {}

        tmp_headers = []
        for header in ctrl_settings.headers:
            tmp_headers.append(header["name"])
        self._widget.tree_ctrl_settings.setHeaderLabels(tmp_headers)

        hand_item = QTreeWidgetItem(ctrl_settings.hand_item)
        self._widget.tree_ctrl_settings.addTopLevelItem(hand_item)
        for index_finger, finger_settings in enumerate(ctrl_settings.fingers):
            finger_item = QTreeWidgetItem(hand_item, finger_settings)
            self._widget.tree_ctrl_settings.addTopLevelItem(finger_item)
            for motor_settings in ctrl_settings.motors[index_finger]:
                motor_name = motor_settings[1]

                motor_item = QTreeWidgetItem(finger_item, motor_settings)
                self._widget.tree_ctrl_settings.addTopLevelItem(motor_item)

                parameter_values = self.sr_controller_tuner_app_.load_parameters(controller_type, motor_name)
                if parameter_values != -1:
                    # the parameters have been found
                    self.ctrl_widgets[motor_name] = {}

                    # buttons for plot/move are not added in edit_only_mode
                    if not self.sr_controller_tuner_app_.edit_only_mode:
                        # add buttons for the automatic procedures (plot / move / ...)
                        frame_buttons = QFrame()
                        layout_buttons = QHBoxLayout()
                        btn_plot = QPushButton("Plot")
                        self.ctrl_widgets[motor_name]["btn_plot"] = btn_plot
                        self.ctrl_widgets[motor_name]["btn_plot"].clicked.connect(partial(self.on_btn_plot_pressed_,
                                                                                          motor_name, self.ctrl_widgets[motor_name]["btn_plot"]))
                        layout_buttons.addWidget(btn_plot)

                        if self.controller_type in ["Position", "Muscle Position", "Mixed Position/Velocity"]:
                            # only adding Move button for position controllers
                            btn_move = QPushButton("Move")
                            self.ctrl_widgets[motor_name]["btn_move"] = btn_move
                            self.ctrl_widgets[motor_name]["btn_move"].clicked.connect(partial(self.on_btn_move_pressed_, motor_name,
                                                                                              self.ctrl_widgets[motor_name]["btn_move"]))
                            layout_buttons.addWidget(btn_move)
                            frame_buttons.setLayout(layout_buttons)

                        self._widget.tree_ctrl_settings.setItemWidget(motor_item, 0, frame_buttons)

                    for index_item, item in enumerate(ctrl_settings.headers):
                        if item["type"] == "Bool":
                            check_box = QCheckBox()

                            param_name = item["name"].lower()
                            param_val = parameter_values[param_name]
                            check_box.setChecked(param_val)
                            if param_name == "sign":
                                check_box.setToolTip("Check if you want a negative sign\n(if the motor is being driven\n the wrong way around).")

                            self._widget.tree_ctrl_settings.setItemWidget(motor_item, index_item, check_box)

                            self.ctrl_widgets[motor_name][param_name] = check_box

                        if item["type"] == "Int":
                            spin_box = QSpinBox()
                            spin_box.setRange(int(item["min"]), int(item["max"]))

                            param_name = item["name"].lower()
                            spin_box.setValue(int(parameter_values[param_name]))
                            self.ctrl_widgets[motor_name][param_name] = spin_box

                            self._widget.tree_ctrl_settings.setItemWidget(motor_item, index_item, spin_box)

                        if item["type"] == "Float":
                            spin_box = QDoubleSpinBox()
                            spin_box.setRange(-65535.0, 65535.0)
                            spin_box.setDecimals(3)

                            param_name = item["name"].lower()
                            spin_box.setValue(float(parameter_values[param_name]))
                            self.ctrl_widgets[motor_name][param_name] = spin_box

                            self._widget.tree_ctrl_settings.setItemWidget(motor_item, index_item, spin_box)

                        motor_item.setExpanded(True)
                else:
                    motor_item.setText(1, "parameters not found - controller tuning disabled")
            finger_item.setExpanded(True)
        hand_item.setExpanded(True)

        for col in range(0, self._widget.tree_ctrl_settings.columnCount()):
            self._widget.tree_ctrl_settings.resizeColumnToContents(col)

    def on_btn_stop_mvts_clicked_(self):
        for move_thread in self.move_threads:
            move_thread.__del__()
        self.move_threads = []

    #########
    # Default methods for the rqtgui plugins

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
