#!/usr/bin/env python
#
# author Guillaume Walck (2015)
#
# derived from pr2_breaker.py
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import rospy

from python_qt_binding.QtCore import QSize, pyqtSignal

from m3meka_msgs.msg import M3ControlState, M3ControlStates
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeRequest

from rqt_robot_dashboard.widgets import MenuDashWidget

from python_qt_binding.QtGui import QMessageBox


class ControlStateButton(MenuDashWidget):
    """
    Dashboard widget to display and interact with the Control state.
    """
    # create a signal for external triggering
    _set_enabled_signal = pyqtSignal(int)
        
    def __init__(self, group_name, group_index):
        """
        :param group_name: Name of the group
        :type group_name: str
        :param group_index: Index of the group
        :type group_index: int
        """

        if group_name == 'left_arm':
            state_icon = 'ic-larm.svg'
        elif group_name == 'left_hand':
            state_icon = 'ic-lhand.svg'
        elif group_name == 'head':
            state_icon = 'ic-head.svg'
        elif group_name == 'right_arm':
            state_icon = 'ic-rarm.svg'
        elif group_name == 'right_hand':
            state_icon = 'ic-rhand.svg'
        elif group_name == 'torso':
            state_icon = 'ic-torso.svg'
        elif group_name == 'zlift':
            state_icon = 'ic-zlift.svg'
        elif group_name == 'base':
            state_icon = 'ic-base.svg'
        else:
            state_icon = 'ic-breaker.svg'

        running_icon = ['bg-green.svg', state_icon]
        ready_icon = ['bg-yellow.svg', state_icon]
        estop_icon = ['bg-red.svg', state_icon, 'ol-err-badge.svg']
        standby_icon = ['bg-grey.svg', state_icon]
        disabled_icon = ['bg-light_grey.svg', state_icon]

        icons = [disabled_icon, estop_icon, standby_icon, ready_icon, running_icon]

        super(ControlStateButton, self).__init__('State:' + group_name, icons=icons, icon_paths=[['rqt_m3dashboard', 'images']])
        
        # init the button in disabled
        self.update_state(0)

        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

        self.add_action('Run', self.on_run)
        self.add_action('Freeze', self.on_freeze)
        self.add_action('Standby', self.on_standby)
        self._enable_menu = self.add_action('Enable/Disable', self.on_enable_disable)
        self._enable_menu.setCheckable(True)
        self._enable_menu.setChecked(False)
        self.set_group_enabled(False)
        
        self._set_enabled_signal.connect(self.on_enable_disable)
                
        #self.add_separator()
        #self.add_action('Run All Groups', self.on_run_all)
        #self.add_action('Freeze All Groups', self.on_freeze_all)
        #self.add_action('Standby All Groups', self.on_standby_all)

        self._state_control = rospy.ServiceProxy('/meka_roscontrol_state_manager/change_state', M3ControlStateChange)
        self._serial = 0
        self._index = group_index
        self._name = group_name
        self._pending_msg =  None
        self._state = None
        self._last_status_msg = None
        self.setToolTip(group_name)

    def control(self, group, cmd):
        """
        Sends a change state to the m3ros_control

        :param group: group iname to send command to
        :type group: str
        :param cmd: command to be sent to the pr2 breaker
        :type cmd: int
        """
        if (self._state==None):
            QMessageBox.critical(self, "Error", self.tr("Cannot control until we have received a state message"))
            return False

        if (self._state == 0):
            if (cmd == 3):
                QMessageBox.critical(self, "Error", self.tr("Group will not enable because one of the estops is pressed"))
                return False

        try:
            state_cmd = M3ControlStateChangeRequest()
            state_cmd.command.group_name.append(group)
            state_cmd.command.state.append(cmd)
            self._state_control(state_cmd)

            return True
        except rospy.ServiceException, e:
            QMessageBox.critical(self, "Error", "Service call failed with error: %s" % (e), "Error")
            return False

        return False

    def control3(self, cmd):
        if (not self.control(0, cmd)):
            return False
        if (not self.control(1, cmd)):
            return False
        if (not self.control(2, cmd)):
            return False
        return True

    def on_run(self):
        self.set_run()

    def on_freeze(self):
        self.set_freeze()

    def on_standby(self):
        self.set_instandby()
  
    def on_enable_disable(self):
        if self._enable_menu.isChecked():
            if self._state is not None:
                self.set_group_enabled(True)
                self.set_state(self._pending_msg)
            else:
                self._enable_menu.setChecked(False)
        else:
            self.set_group_enabled(False)
            

    def on_run_all(self):
        self.set_run_all()

    def on_freeze_all(self):
        self.set_freeze_all()

    def on_standby_all(self):
        self.set_standby_all()

    def set_run(self):
        if (not self.control(self._name, 2)):
            return

        self.control(self._name, 3)

    def set_freeze(self):
        self.control(self._name, 2)

    def set_instandby(self):
        self.control(self._name, 1)

    def set_run_all(self):
        if (not self.control3(2)):
            return
        self.control3(3)

    def set_freeze_all(self):
        self.control3(2)

    def set_standby_all(self):
        self.control3(1)

    def set_state(self, state):
        """
        Sets state of button based on msg

        :param msg: message containing the M3 control state
        :type msg: m3meka_msgs.msg.M3ControlStates
        """

        if (self._enable_menu.isChecked() or self._state is None):
            status_msg = "Running"
            # if first message received, enable the group
            if self._state is None:
                self.set_group_enabled(True)
                self._enable_menu.setChecked(True)
            self._state = state
            if (state == M3ControlStates.ESTOP):
                self.set_state_estop()
                status_msg = "E-Stop"
            elif (state == M3ControlStates.STOP):
                self.set_state_standby()
                status_msg = "Stop"
            elif (state == M3ControlStates.FREEZE):
                self.set_state_ready()
                status_msg = "Freeze"
            else:
                self.set_state_running()

            if (status_msg != self._last_status_msg):
                self.setToolTip("Group: %s \nState: %s" % (self._name, status_msg))
                self._last_status_msg = status_msg
        else:
            self._pending_msg = state
            
    def set_state_running(self):
        self.update_state(4)

    def set_state_ready(self):
        self.update_state(3)

    def set_state_standby(self):
        self.update_state(2)

    def set_state_estop(self):
        self.update_state(1)
        
    def set_group_enabled(self, val):
        if not val:
            self.update_state(0)
        for action in self._menu.actions():
            if not action.isCheckable():
                action.setEnabled(val)
                
