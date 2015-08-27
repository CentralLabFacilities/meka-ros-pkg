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
from QtGui import QPushButton, QVBoxLayout, QWidget

from m3meka_msgs.msg import M3ControlState, M3ControlStates,\
    M3StateChangeGoal, M3StateChangeAction
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse




from std_msgs.msg import Bool
from .emergency_button import EmergencyButton
from .state_button import ControlStateButton

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
        self._widget_initialized = False

        NAMESPACE = '/m3dashboard'
           
        self._state_button = ControlStateButton('All', 0)
        self._dashboard_agg_sub = rospy.Subscriber("/meka_roscontrol_state_manager/state",M3ControlStates,
                                                                                    self.dashboard_callback,
                                                                                    callback_args={'state': "All"},
                                                                                    queue_size=1)
                                                                                    
        self._actionclient = actionlib.SimpleActionClient("/meka_state_manager", M3StateChangeAction)                                                        
        if self._actionclient.wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to state_manager action server in 4 sec")
            
        
        self._main_widget = QWidget()
        vlayout = QVBoxLayout()
    
        self.btn_start = QPushButton("start")
        self.btn_stop = QPushButton("stop")
        self.btn_freeze = QPushButton("freeze")
        
        vlayout.addWidget(self.btn_start)
        vlayout.addWidget(self.btn_freeze)
        vlayout.addWidget(self.btn_stop)
        
        self.btn_start.clicked.connect(self.on_btn_start_clicked)
        self.btn_stop.clicked.connect(self.on_btn_stop_clicked)
        self.btn_freeze.clicked.connect(self.on_btn_freeze_clicked)
        
        self._main_widget.setLayout(vlayout)
        self.context.add_widget(self._main_widget)
        #self._main_widget.addLayout(hlayout)

        self._widget_initialized = True

    def change_state(self, name, cmd):
        goal = M3StateChangeGoal()
        goal.retries = 2
        goal.strategy = M3StateChangeGoal.RETRY_N_TIMES
        goal.command.group_name.append(name)
        goal.command.state.append(cmd)
        try:
            self._actionclient.send_goal(goal)
        except rospy.ROSException:
            rospy.logerr("Failed to call change state")

    def on_btn_start_clicked(self):
        """
        start
        """
        self.change_state("all", M3ControlStates.START)
        
    def on_btn_stop_clicked(self):
        """
        stop
        """
        self.change_state("all", M3ControlStates.STOP)

    
    def on_btn_freeze_clicked(self):
        """
        freeze
        """
        self.change_state("all", M3ControlStates.FREEZE)

    def get_widgets(self):
        widgets_list = []
        widgets_list.append([self._state_button])

        return widgets_list

    def dashboard_callback(self, msg, cb_args):
        """
        callback to process messages

        :param msg:
        :type msg: Float32 or Bool
        :param cb_args:
        :type cb_args: dictionary
        """
        if not self._widget_initialized:
            return

        
        if cb_args.has_key('state'):
            state_name = cb_args['state']
            self._state_button.set_state_msg(msg)

        # Throttling to 1Hz the update of the widget whatever the rate of the topics is
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0):
            return
        self._last_dashboard_message_time = rospy.Time.now()

    

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()
        
