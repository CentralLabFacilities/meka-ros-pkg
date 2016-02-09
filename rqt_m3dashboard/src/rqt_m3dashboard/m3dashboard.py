#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# derived from rqt_emergency_buttons: emergency_buttons_dashboard.py
#   original Authors Sammy Pfeiffer
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#

# author Guillaume WALCK (2015)

import rospy

from rqt_robot_dashboard.dashboard import Dashboard
import actionlib

from python_qt_binding.QtCore import QSize
from QtGui import QPushButton, QVBoxLayout, QHBoxLayout, QWidget, \
    QCheckBox, QSpinBox, QLabel

from m3meka_msgs.msg import M3ControlState, M3ControlStates, \
    M3StateChangeGoal, M3StateChangeAction
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse

from std_msgs.msg import Bool, String
from .emergency_button import EmergencyButton
from .state_button import ControlStateButton
from PyQt4.Qt import QTextEdit

class M3Dashboard(Dashboard):
    """
    Dashboard for Mekabot

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Emergency Buttons Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._state_buttons = {}
        self._widget_initialized = False
        self._service_ready = True
        NAMESPACE = '/m3dashboard'

        # self._state_button = ControlStateButton("default", 9999)
        # TODO read this list on the parameters
        group_names = ["left_hand", "left_arm", "head", "right_arm", "right_hand", "zlift", "torso", "base"]
        # create as many buttons as groups received
        for group_name in group_names:
            self._state_buttons[group_name] = ControlStateButton(group_name, 0)

        self.bat_lbb = QLabel("Battery voltage")
        self.bat_txt = QLabel()
        self.bat_txt.setText("0.0")

        self._dashboard_mekaros_sub = rospy.Subscriber("/meka_ros_pub/generic", String, self.callback)

        self._dashboard_agg_sub = rospy.Subscriber("/meka_roscontrol_state_manager/state", M3ControlStates,
                                                                                    self.dashboard_callback,
                                                                                    queue_size=1)

        self._actionclient = actionlib.SimpleActionClient("/meka_state_manager", M3StateChangeAction)
        rospy.loginfo("Looking for state manager...")
        if self._actionclient.wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to state_manager action server in 4 sec")
            self._service_ready = False
        else:
            rospy.loginfo("Found the state manager")

        self._main_widget = QWidget()
        vlayout = QVBoxLayout()
        hlayout = QHBoxLayout()
        hlayout2 = QHBoxLayout()

        hlayout2.addWidget(self.bat_lbb)
        hlayout2.addWidget(self.bat_txt)

        self.chk_all = QCheckBox("enable_all")
        self.chk_all.setChecked(True)

        self.spin_retries = QSpinBox()
        # infinite number of times is -1
        self.spin_retries.setMinimum(-1)
        self.spin_retries.setMaximum(10)
        self.spin_retries.setValue(2)
        label_retries = QLabel("trial times")

        self.btn_start = QPushButton("start")
        self.btn_stop = QPushButton("stop")
        self.btn_freeze = QPushButton("freeze")
        if not self._service_ready:
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(False)
            self.btn_freeze.setEnabled(False)

        hlayout.addWidget(self.chk_all)
        hlayout.addWidget(label_retries)
        hlayout.addWidget(self.spin_retries)

        vlayout.addLayout(hlayout2)
        vlayout.addLayout(hlayout)
        vlayout.addWidget(self.btn_start)
        vlayout.addWidget(self.btn_freeze)
        vlayout.addWidget(self.btn_stop)

        self.btn_start.clicked.connect(self.on_btn_start_clicked)
        self.btn_stop.clicked.connect(self.on_btn_stop_clicked)
        self.btn_freeze.clicked.connect(self.on_btn_freeze_clicked)
        self.chk_all.stateChanged.connect(self.on_enable_all_clicked)

        self._main_widget.setLayout(vlayout)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

    def change_state(self, cmd):
        goal = M3StateChangeGoal()
        goal.retries = 0
        if self.spin_retries.value() == -1:
            goal.strategy = M3StateChangeGoal.KEEP_TRYING
        if self.spin_retries.value() == 0:
            goal.strategy = M3StateChangeGoal.HALT_ON_FAILURE
        if self.spin_retries.value() == 1:
            goal.strategy = M3StateChangeGoal.BEST_POSSIBLE

        if self.spin_retries.value() > 1:
            goal.strategy = M3StateChangeGoal.RETRY_N_TIMES
            goal.retries = self.spin_retries.value()


        # find enabled groups
        for group_name  in self._state_buttons:
            if self._state_buttons[group_name]._enable_menu.isChecked():
                goal.command.group_name.append(group_name)
                goal.command.state.append(cmd)
        try:
            if len(goal.command.group_name) > 0:
                self._actionclient.send_goal(goal)
        except rospy.ROSException:
            rospy.logerr("Failed to call change state")

    def on_enable_all_clicked(self):
        """
        start
        """
        for group_name  in self._state_buttons:
            self._state_buttons[group_name]._enable_menu.setChecked(self.chk_all.isChecked())
            self._state_buttons[group_name]._enable_menu.triggered.emit(self.chk_all.isChecked())
            # stateChanged.emit( _set_enabled_signal.emit(self.chk_all.isChecked())

    def on_btn_start_clicked(self):
        """
        start
        """
        self.change_state(M3ControlStates.START)

    def on_btn_stop_clicked(self):
        """
        stop
        """
        self.change_state(M3ControlStates.STOP)


    def on_btn_freeze_clicked(self):
        """
        freeze
        """
        self.change_state(M3ControlStates.FREEZE)

    def get_widgets(self):
        widgets_list = []
        for group_name in self._state_buttons:
            widgets_list.append([self._state_buttons[group_name]])

        return widgets_list

    def callback(self, msg):
        """
        callback to process meka ros str messages
        :param msg:
        :type msg: String
        """
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

        val = str(msg.data)[:5]
        self.bat_txt.setText(val)


    def dashboard_callback(self, msg):
        """
        callback to process state messages
        :param msg:
        :type msg: M3ControlStates
        """

        single_msg = M3ControlStates()
        for group_name, state in zip(msg.group_name, msg.state):
            if group_name in self._state_buttons:
                self._state_buttons[group_name].set_state(state)

        if not self._service_ready:
            # test reconnection only each 5 seconds
            if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(5.0):
                return
            self._last_dashboard_message_time = rospy.Time.now()
            if self._actionclient.wait_for_server(timeout=rospy.Duration(1)) is False:
                rospy.logfatal("Failed to connect to state_manager action server, trying again in 5 seconds")
            else:
                self._service_ready = True
                self.btn_start.setEnabled(True)
                self.btn_stop.setEnabled(True)
                self.btn_freeze.setEnabled(True)


    def shutdown_dashboard(self):
        self._dashboard_mekaros_sub.unregister()
        self._dashboard_agg_sub.unregister()

