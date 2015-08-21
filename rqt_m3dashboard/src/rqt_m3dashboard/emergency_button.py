#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_emergency_buttons: emergency_button.py
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

from .icon_widget import IconWidget


class EmergencyButton(IconWidget):
    """
    Dashboard widget to display emergency button state.
    """
    def __init__(self, context, name):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        icon_names = ['emergency_button_gray.png','emergency_button_color.png']
        icons = []
        for icon in icon_names:
            icons.append([icon])
        icon_paths = []
        icon_paths.append(['rqt_m3dashboard', 'images'])
        self._name = name
        super(EmergencyButton, self).__init__(name=name,
                                                  icons=icons,
                                                  icon_paths=icon_paths,
                                                  suppress_overlays=True)
        self.setToolTip(name)
        self.set_non_pressed()

    def set_non_pressed(self):
        self.update_state(0)
        self.setToolTip(self._name + " is NOT pressed.")

    def set_pressed(self):
        self.update_state(1)
        self.setToolTip(self._name + " IS pressed.")

    def set_pressed_status(self, pressed):
        if pressed:
            self.set_pressed()
        else:
            self.set_non_pressed()
