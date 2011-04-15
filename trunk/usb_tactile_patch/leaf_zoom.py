#!/usr/bin/env python
import roslib; roslib.load_manifest('usb_tactile_patch')
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('/leaf_zoom', Int32)
    rospy.init_node('leaf_zoom_node')
    if not rospy.is_shutdown():              
        pub.publish(Int32(9))
        
    while not rospy.is_shutdown():
        num = raw_input('Input from 1, 2, 3, ... 9 for Zoom and press enter:  ')
        #print(type(num))
        try:
            num_int = int(num)
        except:
            pass
        else:
            if num_int > 0 and num_int < 10:
                pub.publish(Int32(num_int))
        #pub.publish(Int32(3))
        #rospy.sleep(1.0)
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass