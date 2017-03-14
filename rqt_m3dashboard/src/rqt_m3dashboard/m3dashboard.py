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
# author pluecking (2016)

import rospy
import sip
import rospkg
import os
import inspect
import actionlib

from functools import partial

from rqt_robot_dashboard.dashboard import Dashboard

#messages
from m3meka_msgs.msg import M3ControlState, M3ControlStates, \
    M3StateChangeGoal, M3StateChangeAction
from rospy_tutorials.msg import Floats
from std_msgs.msg import Bool, String

#services
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse, \
    M3ControlStateChangeRequest
from meka_ros_publisher.srv import ListComponents, ListFields, RequestValues

#widgets
from widgets import EmergencyButton, ControlStateButton, WrappedBattery
from dialogs import PlotDialog

#qt stuff
from PyQt5.Qt import QMenu, QGridLayout, QLineEdit, QPixmap, QIcon, QSize

from QtWidgets import QPushButton, QCheckBox, QSpinBox, QLabel, QVBoxLayout, QHBoxLayout, QWidget
    
from QtCore import QObject

from QtCore import pyqtSignal as SIGNAL

MIN_V = 22.0
MAX_V = 25.5
CHARGE_V_THRES = 26.0

STATE_CMD_DISABLE = 1
STATE_CMD_ENABLE = 2

class M3Dashboard(Dashboard):
    """
    Dashboard for Mekabot

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Meka Dashboard'
        self.max_icon_size = QSize(50, 30)
        
        self._widget_initialized = False
        self._service_ready = True
        self._last_dashboard_message_time = rospy.Time.now()
        self._path = os.path.join(rospkg.RosPack().get_path('rqt_m3dashboard'), 'images')
        self._battery_icons = {}
        self._state_buttons = {}
        
        self._dashboard_mekaros_subs = {}
        self._m3field_values = {}
        self._m3field_plots = {}
        
        NAMESPACE = '' 

        rospy.loginfo("Starting up...")
        
        # self._state_button = ControlStateButton("default", 9999)
        # TODO read this list on the parameters
        battery_names = ["m3pwr_pwr038", "m3pwr_pwr042"]
        group_names = {"left_hand":"mh26", "left_arm":"ma30", "head":"ms8", "right_arm":"ma29", "right_hand":"mh24", "zlift":"mz7", "torso":"mt6", "base":"mb7"}
        
        # create as many buttons as groups received
        for group_name in group_names.keys():
            self._state_buttons[group_name] = ControlStateButton(group_name, 0)

        battery_widget = self.init_batteries(battery_names)

        self._dashboard_agg_sub = rospy.Subscriber("/meka_roscontrol_state_manager/state", 
                                                       M3ControlStates,                               
                                                       self.state_callback,
                                                       queue_size=1)
        self._actionclient = actionlib.SimpleActionClient("/meka_state_manager", M3StateChangeAction)
        rospy.loginfo("Looking for state manager...")
        if self._actionclient.wait_for_server(timeout=rospy.Duration(0.1)) is False:
            rospy.logfatal("Failed to connect to state_manager action server in 4 sec")
            self._service_ready = False
        else:
            rospy.loginfo("Found the state manager")
            
        self._state_control = rospy.ServiceProxy('/meka_roscontrol_state_manager/change_state', M3ControlStateChange)

        self._main_widget = QWidget()
        vlayout = QVBoxLayout()
        hlayout = QHBoxLayout()
        hlayout2 = QHBoxLayout()

        self.chk_all = QCheckBox("enable_all")
        self.chk_all.setChecked(False)

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
            
        inspection_button = self.init_inspection()
        self.hz_rate = QSpinBox()
        # infinite number of times is -1
        self.hz_rate.setMinimum(1)
        self.hz_rate.setMaximum(100)
        self.hz_rate.setValue(1)
        label_hz = QLabel("hz")

        hlayout2.addWidget(inspection_button)
        hlayout2.addWidget(self.hz_rate)
        hlayout2.addWidget(label_hz)
        
        hlayout.addWidget(self.chk_all)
        hlayout.addWidget(label_retries)
        hlayout.addWidget(self.spin_retries)

        self.inspection_layout = QGridLayout()

        #vlayout.addWidget()
        vlayout.addWidget(battery_widget)
        vlayout.addLayout(hlayout)
        vlayout.addWidget(self.btn_start)
        vlayout.addWidget(self.btn_freeze)
        vlayout.addWidget(self.btn_stop)
        vlayout.addLayout(hlayout2)
        vlayout.addLayout(self.inspection_layout)

        self.btn_start.clicked.connect(self.on_btn_start_clicked)
        self.btn_stop.clicked.connect(self.on_btn_stop_clicked)
        self.btn_freeze.clicked.connect(self.on_btn_freeze_clicked)
        self.chk_all.stateChanged.connect(self.on_enable_all_clicked)

        self._main_widget.setLayout(vlayout)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True
        
    def init_batteries(self, battery_names):
        
        widget = QWidget()
        layout = QHBoxLayout(widget)        
        fields = ["motor_enabled", "bus_voltage"]        
        dt = Floats
                 
        for battery_name in battery_names:
            self._battery_icons[battery_name] = WrappedBattery(self.context, battery_name)
            layout.addWidget(self._battery_icons[battery_name])
            self._dashboard_mekaros_subs[(battery_name, fields[0])] = rospy.Subscriber("/meka_ros_pub/"+battery_name+"/"+fields[0], dt, self.battery_pwrd_cb, battery_name)                            
            self._dashboard_mekaros_subs[(battery_name, fields[1])] = rospy.Subscriber("/meka_ros_pub/"+battery_name+"/"+fields[1], dt, self.battery_voltage_cb, battery_name)
            
        layout.addStretch(1)
        widget.setFixedHeight(75)

        #self.bat_lbb = QLabel("Battery voltage")
        #self.bat_txt = QLabel()
        #self.bat_txt.setText("0.0")

        return widget

    def init_inspection(self):

        button = QPushButton("Robot inspection (beta)")

        service_list_comps = '/meka_ros_publisher/list_components'
        service_list_fields = '/meka_ros_publisher/list_fields'
        service_req_values = '/meka_ros_publisher/request_values'
        rospy.loginfo("Waiting for %s, %s and %s", service_list_comps, service_list_fields, service_req_values)
        try:
            rospy.wait_for_service(service_list_comps, 4.0)
            rospy.wait_for_service(service_list_fields, 4.0)
            rospy.wait_for_service(service_req_values, 4.0)
        except rospy.ROSException:
            rospy.logerr("%s and/or %s did not show up. Giving up", service_list_comps, service_list_fields)
            return button
            
        self.list_comps_client = rospy.ServiceProxy(service_list_comps, ListComponents)
        self.list_fields_client = rospy.ServiceProxy(service_list_fields, ListFields)
        self.req_vals_client = rospy.ServiceProxy(service_req_values, RequestValues)
        rospy.loginfo("Found %s, %s and %s", service_list_comps, service_list_fields, service_req_values)

        # get all the components
        try:
            resp = self.list_comps_client("")
        except rospy.ServiceException:
            rospy.logerr("Could not call list_components")
            return button
            
        menu = QMenu("Menu")
        menu.setStyleSheet("QMenu { menu-scrollable: 1; }");
        submenus = []

        self.req_action = {}
        if(resp):
            for component in resp.components:
                s = menu.addMenu(component)
                self.req_action[component] = s.addAction("request fields", partial(self.request_fields, component, s))
                

        button.setMenu(menu)

        return button

    def init_inspection_test(self):

        button = QPushButton("Robot inspection (beta)")

        menu = QMenu("Menu")
        menu.setStyleSheet("QMenu { menu-scrollable: 1; }");
        submenus = []


        for i in range(5):
            # a component has fields which are either strings or arrays.
            s = menu.addMenu(str(i))
            for j in range(5):
                s.addAction(str(j), partial(self.subscribe_to_field_test, "blubber", j))

        button.setMenu(menu)

        return button

    def testbaloon(self, name):

        name_label = QLabel(name)
        lineedit = QLineEdit("random")
        name_label.setBuddy(lineedit);

        close_btn = QPushButton()

        pixmap = QPixmap(self._path + "/close.png")
        icon = QIcon(pixmap);
        close_btn.setIcon(icon)
        close_btn.setIconSize(pixmap.rect().size())

        idx = self.inspection_layout.rowCount()

        close_btn.clicked.connect(partial(self.remove_row, self.inspection_layout, idx, False))
        self.inspection_layout.addWidget(name_label, idx, 0);
        self.inspection_layout.addWidget(lineedit, idx, 1);
        self.inspection_layout.addWidget(close_btn, idx, 2);

    def subscribe_to_field_test(self, component, bla):

        field = str(bla)

        if (component, field) in self._m3field_values:
            rospy.logwarn("Already subscribed to field. Exiting.")
            return


        dt = Floats
        topic = str("/meka_ros_pub/" + component + "/" + field)
        self._dashboard_mekaros_subs[(component, field)] = rospy.Subscriber(topic, dt, self.field_callback, (component, field))
        self._m3field_values[(component, field)] = []

        name_label = QLabel(component + "->" + field + ":")
        for val in range(bla):
            label = QLabel(str(val)[:5])
            label.setStyleSheet("border: 2px solid grey");
            self._m3field_values[(component, field)].append(label)

        idx = self.inspection_layout.rowCount()

        plot_pixmap = QPixmap(self._path + "/plot.png")
        plot_icon = QIcon(plot_pixmap);
        plot_btn = QPushButton()
        plot_btn.setIcon(plot_icon)
        plot_btn.setIconSize(plot_pixmap.rect().size())
        plot_btn.setFixedWidth(30)
        plot_btn.clicked.connect(partial(self.plot_values, component, field))

        close_pixmap = QPixmap(self._path + "/close.png")
        close_icon = QIcon(close_pixmap);
        close_btn = QPushButton()
        close_btn.setIcon(close_icon)
        close_btn.setIconSize(close_pixmap.rect().size())
        close_btn.setFixedWidth(30)
        close_btn.clicked.connect(partial(self.remove_row, self.inspection_layout, idx, False, component, field))

        self.inspection_layout.addWidget(name_label, idx, 0)
        val_layout = QHBoxLayout()
        for label in self._m3field_values[(component, field)]:
            print label
            val_layout.addWidget(label);
        self.inspection_layout.addLayout(val_layout, idx, 1)
        self.inspection_layout.addWidget(plot_btn, idx, 2)
        self.inspection_layout.addWidget(close_btn, idx, 3)


    def request_fields(self, component, top_menu):
        
        # a component has fields which are either strings or arrays.
        try:
            resp2 = self.list_fields_client(component)
        except rospy.ServiceException:
            rospy.logerr("Could not call list_fields")
            return
        if(resp2):
            for field in resp2.fields:
                top_menu.addAction(field, partial(self.subscribe_to_field, component, field))
                
        self.req_action[component].setEnabled(False)


    def subscribe_to_field(self, component, field):
        
        if (component, field) in self._m3field_values:
            rospy.logwarn("Already subscribed to field. Exiting.")
            return
                
        try:
            resp = self.req_vals_client(component, field, "", self.hz_rate.value())
        except rospy.ServiceException:
            rospy.logerr("Could not call request_values")
            return
        
        print resp.values
        
        dt = Floats
        topic = str("/meka_ros_pub/"+component+"/"+field)
        self._dashboard_mekaros_subs[(component, field)] = rospy.Subscriber(topic, dt, self.field_callback, (component, field))
        self._m3field_values[(component, field)] = []
        
        name_label = QLabel(component+"->"+field+":")
        for val in resp.values:
            label = QLabel(str(val)[:5])
            label.setStyleSheet("border: 2px solid grey");
            self._m3field_values[(component, field)].append(label)
        
        idx = self.inspection_layout.rowCount()
        
        plot_pixmap = QPixmap(self._path + "/plot.png")
        plot_icon = QIcon(plot_pixmap);
        #plot_btn = QPushButton()
        #plot_btn.setIcon(plot_icon)
        #plot_btn.setIconSize(plot_pixmap.rect().size())
        #plot_btn.setFixedWidth(30)
        #plot_btn.clicked.connect(partial(self.plot_values, component, field))
        
        close_pixmap = QPixmap(self._path + "/close.png")
        close_icon = QIcon(close_pixmap);
        close_btn = QPushButton()
        close_btn.setIcon(close_icon)
        close_btn.setIconSize(close_pixmap.rect().size())
        close_btn.setFixedWidth(30)
        close_btn.clicked.connect(partial(self.remove_row, self.inspection_layout, idx, False, component, field))
        
        self.inspection_layout.addWidget(name_label, idx, 0)
        val_layout = QHBoxLayout()
        for label in self._m3field_values[(component, field)]:
            val_layout.addWidget(label);
        self.inspection_layout.addLayout(val_layout, idx, 1)
        #self.inspection_layout.addWidget(plot_btn, idx, 2)
        self.inspection_layout.addWidget(close_btn, idx, 3)
        
    def plot_values(self, component, field):
        """
        plot_values create a new plot for the values of the given component and field
        :params component, field:
        :type component: Identifier of the component, field: Identifier of the field
        """
        dialog = PlotDialog()
        QObject.connect(dialog, SIGNAL('rejected()'), partial(self.on_plot_close,  component, field))
        dialog.init_plot(component, field)
        dialog.show()
        
        self._m3field_plots[(component, field)] = dialog
        
        #dialog.exec_()

    
    def change_state(self, cmd):
        """
        change_state calls the m3rosctrl robot interface to change the 
        state of one or multiple joint groups (e.g. "head" to "freeze")
        :params cmd:
        :type cmd: Identifier of the new state
        """
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
        for group_name in self._state_buttons:
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
        enable all
        """
        state_cmd = M3ControlStateChangeRequest()
                    
        for group_name in self._state_buttons:
            state_cmd.command.group_name.append(group_name)
            if self.chk_all.isChecked():
                self._state_buttons[group_name]._enable_menu.setChecked(True)
                self._state_buttons[group_name]._disable_menu.setChecked(False)
                self._state_buttons[group_name]._enable_menu.setEnabled(False)
                self._state_buttons[group_name]._disable_menu.setEnabled(True)
                state_cmd.command.state.append(STATE_CMD_ENABLE)
            else:
                self._state_buttons[group_name]._enable_menu.setChecked(False)
                self._state_buttons[group_name]._disable_menu.setChecked(True)
                self._state_buttons[group_name]._enable_menu.setEnabled(True)
                self._state_buttons[group_name]._disable_menu.setEnabled(False)
                state_cmd.command.state.append(STATE_CMD_DISABLE)
        try:
            self._state_control(state_cmd)
        except rospy.ServiceException, e:
            QMessageBox.critical(self, "Error", "Service call failed with error: %s" % (e), "Error")

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

    def on_plot_close(self, component, field):
        """
        close plot
        """
        rospy.loginfo("Deleting plot data...")
        
        del self._m3field_plots[(component, field)]

    def get_widgets(self):
        widgets_list = []
        for group_name in self._state_buttons:
            widgets_list.append([self._state_buttons[group_name]])

        return widgets_list

    def field_callback(self, msg, args):
        """
        field_callback to process meka ros float messages
        :param msg:
        :type msg: Floats
        """
        component = args[0]
        field = args[1]
        
        for idx, val in enumerate(self._m3field_values[(component, field)]):
            val.setText(str(msg.data[idx])[:5])

        if (component, field) in self._m3field_plots:
            self._m3field_plots[(component, field)].get_plot().setData(msg.data)
    
    
    def battery_pwrd_cb(self, msg, battery_name):
        """
        battery_pwrd_cb to process meka ros str messages
        :param msg:
        :type msg: Floats
        """
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        val_bool = bool(msg.data[0])
          
        self._battery_icons[battery_name].set_motor_enabled(val_bool)
    
    def battery_voltage_cb(self, msg, battery_name):
        """
        battery_voltage_cb to process meka ros str messages
        :param msg:
        :type msg: Floats
        """
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        val = str(msg.data[0])[:5]
        val_float = float(val)
        
        charging = False
        perc = 100.0
        if (val_float >= CHARGE_V_THRES):
            charging = True
        else:
            perc = ((float(val_float) - MIN_V) / (MAX_V-MIN_V)) * 100.0

        self._battery_icons[battery_name].set_power_state_perc(float(perc), charging)
    
            

    def state_callback(self, msg):
        """
        state_callback to process state messages
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
        for x in self._dashboard_mekaros_subs.keys():
            self._dashboard_mekaros_subs[x].unregister()

        self._dashboard_agg_sub.unregister()
    
    def remove_row(self, layout, row, deleteWidgets, component, field):
        """
        remove_row QGridLayout specific function to delete contents of a 
        given row extended to unsubscribe and cleanup everything related to the row
        :params layout, row, deleteWidgets:
        :type layout: QGridLayout, row: int, deleteWidgets: bool
        """
        
        self._dashboard_mekaros_subs[(component, field)].unregister()
        if (component, field) in self._m3field_plots:
            self._m3field_plots[(component, field)].close()
        
        self.remove_widget(layout, row, -1, deleteWidgets);
        layout.setRowMinimumHeight(row, 0);
        layout.setRowStretch(row, 0);     
        
        del self._dashboard_mekaros_subs[(component, field)]
        del self._m3field_values[(component, field)]
            
    def remove_widget(self, layout, row, column, deleteWidgets):
        """
        remove_widget remove a widget from the layout, either by
        specificying a certain row or column (if QGridLayout) or all widgets
        :params layout, row, column, deleteWidgets:
        :type layout: QLayout, row: int, column: int, deleteWidgets: bool
        """
        if isinstance(layout, QGridLayout): # delete row
            for i in reversed(range(layout.count())): 
                (r, c, rs, cs) = layout.getItemPosition(i)
                if ((r <= row and r + rs - 1 >= row) or (c <= column and c + cs - 1 >= column)):
                    item = layout.takeAt(i);
                    if isinstance(item, QHBoxLayout): #another layout
                        self.remove_widget(item, -1, -1, False)
                    else:
                        layout.removeWidget(item.widget())     
                        item.widget().setParent(None)                        
                    #item = None
        else: # delete all
            for i in reversed(range(layout.count())): 
                layout.itemAt(i).widget().setParent(None)

    
