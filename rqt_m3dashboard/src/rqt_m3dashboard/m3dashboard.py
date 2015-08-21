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

from python_qt_binding.QtCore import QSize

from m3meka_msgs.msg import M3ControlState, M3ControlStates
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
        if rospy.has_param(NAMESPACE):
            # rosparam set /emergency_buttons_dashboard/emergency_button "{'emergency1': {'pressed_topic': '/emergency1', 'tooltip_name': 'EMERGENCY1'}}"
            if rospy.has_param(NAMESPACE + '/emergency_buttons'):
                self._emergency_buttons_list = rospy.get_param(NAMESPACE + '/emergency_buttons')
            else:
                rospy.logwarn("No emergency buttons to monitor found in param server under " + NAMESPACE)
                
            if rospy.has_param(NAMESPACE + '/state_buttons'):
                self._state_buttons_list = rospy.get_param(NAMESPACE + '/state_buttons')
            else:
                rospy.logwarn("No state buttons to monitor found in param server under " + NAMESPACE)
        else:
            rospy.logerr("You must set " + NAMESPACE +" parameters to use this plugin. e.g.:\n" +
                         "rosparam set "+ NAMESPACE + "/emergency_buttons \"{'emergency1': {'pressed_topic': '/emergency1', 'tooltip_name': 'EMERGENCY1'}}\""
                         "rosparam set "+ NAMESPACE + "/state_buttons \"{'state1': {'state_topic': '/state1', 'tooltip_name': 'STATE1'}}\"")
            exit(-1)

        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                pressed_topic = emergency_button_elem[emer_name].get('pressed_topic', None)
                tooltip_name = emergency_button_elem[emer_name].get('tooltip_name', None)
                rospy.loginfo("Emergency button: " + str(emer_name) + " has pressed topic: " +
                    str(pressed_topic)  + " and has tooltip name: " + str(tooltip_name))

                emergency_button_elem[emer_name].update({'pressed_status': False})
                emergency_button_elem[emer_name].update({'pressed_sub': rospy.Subscriber(pressed_topic,
                                                                                    Bool,
                                                                                    self.dashboard_callback,
                                                                                    callback_args={'emergency': emer_name},
                                                                                    queue_size=1)})

                emergency_button_elem[emer_name].update({'emergency_widget': EmergencyButton(self.context, name=tooltip_name)})
        
        self._state_button = ControlStateButton('Main', 0)
        self._dashboard_agg_sub = rospy.Subscriber("/meka_roscontrol_state_manager/state",M3ControlStates,
                                                                                    self.dashboard_callback,
                                                                                    callback_args={'state': "Main"},
                                                                                    queue_size=1)
        #for state_button_elem in self._state_buttons_list:
            #for state_name in state_button_elem.keys():
                #state_topic = state_button_elem[state_name].get('state_topic', None)
                #tooltip_name = state_button_elem[state_name].get('tooltip_name', None)
                #rospy.loginfo("State button: " + str(state_name) + " has pressed topic: " +
                    #str(state_topic)  + " and has tooltip name: " + str(tooltip_name))

                #state_button_elem[state_name].update({'state': 0})
                #state_button_elem[state_name].update({'state_sub': rospy.Subscriber(state_topic,
                                                                                    #Int,
                                                                                    #self.dashboard_callback,
                                                                                    #callback_args={'state': state_name},
                                                                                    #queue_size=1)})

                #state_button_elem[state_name].update({'state_widget': StateButton(self.context, name=tooltip_name)})

        self._widget_initialized = True


    def get_widgets(self):
        widgets_list = []
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                widgets_list.append([emergency_button_elem[emer_name]['emergency_widget']])
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

        if cb_args.has_key('emergency'):
            emer_name = cb_args['emergency']
            for emergency_button_elem in self._emergency_buttons_list:
                if emergency_button_elem.has_key(emer_name):
                    emergency_button_elem[emer_name].update({'pressed_status': msg.data})
        
        if cb_args.has_key('state'):
            state_name = cb_args['state']
            self._state_button.set_state_msg(msg)

        # Throttling to 1Hz the update of the widget whatever the rate of the topics is
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0):
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update all widgets
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                emergency_button_elem[emer_name]['emergency_widget'].set_pressed_status(emergency_button_elem[emer_name]['pressed_status'])
                rospy.logdebug("Updated " + str(emer_name) + " with "
                              + ("pressed." if emergency_button_elem[emer_name].get('pressed_status') else "not pressed."))


    def shutdown_dashboard(self):
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                if emergency_button_elem[emer_name]['pressed_sub']:
                    emergency_button_elem[emer_name]['pressed_sub'].unregister()
        self._dashboard_agg_sub.unregister()
        
