#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_batteries: wrap_battery.py
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
# Authors:
#   * Sammy Pfeiffer

from rqt_robot_dashboard.widgets import BatteryDashWidget


class WrappedBattery(BatteryDashWidget):
    """
    Dashboard widget to display batteries state.
    """
    #TODO When nonbutton Dashboard objects are available rebase this widget
    def __init__(self, context, name):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        icon_names = ['ic-battery-0.svg', 'ic-battery-20.svg', 'ic-battery-40.svg',
                 'ic-battery-60-green.svg', 'ic-battery-80-green.svg', 'ic-battery-100-green.svg']
        icons = []
        for icon in icon_names:
            icons.append([icon])

        charge_icon_names = ['ic-battery-charge-0.svg', 'ic-battery-charge-20.svg', 'ic-battery-charge-40.svg',
                        'ic-battery-charge-60-green.svg', 'ic-battery-charge-80-green.svg', 'ic-battery-charge-100-green.svg']
        charge_icons = []
        for charge_icon in charge_icon_names:
            charge_icons.append([charge_icon])
            
        self._wrapped_battery_name = name
        self.motor_enabled = False
        
        super(WrappedBattery, self).__init__(name=name,
                                             icons=icons, charge_icons=charge_icons,
                                             icon_paths=[['rqt_m3dashboard', 'images']])

        self.unset_stale()
        self.update_perc(0)
        

    def set_power_state_perc(self, percentage, charging):
        """
        """
        self.update_perc(percentage)
        self.update_time(percentage) # + remaining
        self.set_charging(charging)
        
    def set_motor_enabled(self, enabled):
        """
        """ 
        self.motor_enabled = enabled
        
    def update_time(self, value):
        try:
            fval = float(value)
            self.setToolTip("%s: %.2f%% remaining. enabled: %r" % (self._name, fval, self.motor_enabled))
        except ValueError:
            self.setToolTip("%s: %s%% remaining. enabled: %r" % (self._name, value, self.motor_enabled))

