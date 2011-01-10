#! /usr/bin/python

import m3.toolbox as m3t
import m3.gui as m3g
import math
import sys
from time import sleep
import roslib; roslib.load_manifest('mic_array')
import rospy
from mic_array.msg import MicArray
from mic_array.srv import MicArrayParam


class mic_array_param:
    def __init__(self):
        self.gui = m3g.M3Gui(stride_ms=125)
        self.num_chan = 6
        self.gains = [1.0] * self.num_chan
        
    def start(self):
        self.gui.add('M3GuiSliders','Gains', (self,'gains'),range(len(self.gains)),[0.0,2.0],m3g.M3GuiWrite,column=1)
        self.gui.start(self.step)
        
    def step(self):
        sleep(0.1)



t=mic_array_param()
try:
    t.start()
except (KeyboardInterrupt):
    pass

    

rospy.wait_for_service('mic_array_param')


try:
    mic_array_param = rospy.ServiceProxy('mic_array_param', MicArrayParam)
    resp = mic_array_param([2.2,0.8,1.0,1.8,1.0,0.5],100000,0.1,1.0)
    print resp.response
    
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

