'''
Created on Jan 14, 2016

@author: plueckin
'''
#! /usr/bin/python
import time
import logging
import sys
from optparse import OptionParser

import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf

import rospy
from std_msgs.msg import String
from rospy.exceptions import ROSException

DEFAULT_NODE_NAME = "meka_ros_publisher"
DEFAULT_PUBLISHER_SCOPE = "/meka_ros_pub/generic"
DEFAULT_HZ = 1

class MekaRosPublisher(object):
    
    def __init__(self, component=None, field=None):
        self.logger = logging.getLogger(self.__class__.__name__)    
        self.logger.setLevel(logging.INFO)
        
        self.logger.debug("Initializing m3rt proxy.")
        self.proxy = m3p.M3RtProxy()
        self.proxy.start()
        
        if(component == None):
            self.cid = self.get_components()
        else:
            self.cid = component
            
        comps = self.proxy.get_available_components()
        name = comps[self.cid]
        
        self.logger.info("Subscribing to component " + str(name))
        
        self.comp = mcf.create_component(name)
        self.proxy.subscribe_status(self.comp)
            
        self.repeated = False
        if(field == None):
            self.field = self.get_field(self.comp)
        else:
            self.field = field
            
        self.logger.info("Subscribing to field " + str(self.field))
        
#       print 'Select Y-Range? [n]'
        yrange = None
#       if m3t.get_yes_no('n'):
#           yrange = []
#           print 'Min?'
#           yrange.append(m3t.get_int())
#           print 'Max?'
#           yrange.append(m3t.get_int())
            
        
        # proxy.publish_param(comp)
   
    def run(self, scope):
         
        #scope = m3t.M3Scope(xwidth=100, yrange=yrange)
        try:
            self.logger.info("Setting up ROS publisher on scope: " + str(scope))
            pub = rospy.Publisher(scope, String, queue_size=5)
            self.logger.info("Setting up ROS node..")
            rospy.init_node("meka_ros_publisher", anonymous=True)
            rate = rospy.Rate(DEFAULT_HZ)
            #ts = time.time()
            self.logger.info("Entering publish loop with HZ: " + str(DEFAULT_HZ))
            while True and not rospy.is_shutdown():
                self.proxy.step()
                if self.repeated:
                    v = m3t.get_msg_field_value(self.comp.status, self.field)[self.idx]
                else:
                    v = m3t.get_msg_field_value(self.comp.status, self.field)
                rospy.loginfo(v)
                pub.publish(v)
                rate.sleep()
                #scope.plot(v)
#                 if False:
#                     if time.time() - ts > 60.0:
#                         print 'Continue [y]?'
#                         if m3t.get_yes_no('y'):
#                             ts = time.time()
#                         else:
#                             break
        except ROSException as e:
            self.logger.error("ROS exception caught! " + str(e))
        except:
            e = sys.exc_info()[0]
            self.logger.error("Exception caught! " + str(e))
            
        
        self.proxy.stop(force_safeop=False)  # allow other clients to continue running
        
    def get_component(self):
        comps = self.proxy.get_available_components()
        print '------- Components ------'
        for i in range(len(comps)):
            print i, ' : ', comps[i]
        print '-------------------------'
        print 'Enter component id'
        
        return m3t.get_int()
    
    def get_field(self, comp):
        field = m3t.user_select_msg_field(comp.status)  
        self.idx = 0
        if hasattr(m3t.get_msg_field_value(comp.status, field), '__len__'):
            self.repeated = True
            print 'Select index of repeated field to monitor: [0]'
            self.idx = m3t.get_int(0)            
        
        return field
   
    
if __name__ == '__main__':
    try:
        # Set up logging.
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')
        #logging.getLogger("rsb").setLevel(logging.WARN) 
    
        # Parse the command line options for the robot ip etc.
        parser = OptionParser()
        parser.add_option("--scope", help="ROS scope of the publisher (default %default)", default=DEFAULT_PUBLISHER_SCOPE,
                          metavar="SCOPE", dest="scope")
        parser.add_option("--component", help="M3Component index",
                          metavar="INDEX", dest="component")
        parser.add_option("--field", help="Field of selected m3component to publish", 
                          metavar="FIELD", dest="field")
        #parser.add_option("--yrange", help="Field of selected m3component to publish", 
        #                  metavar="FIELD", dest="field")
        
        (options, _) = parser.parse_args()
        
        mekarospub = MekaRosPublisher(options.component, options.field)
        mekarospub.run(options.scope)
    except rospy.ROSInterruptException as e:
        logging.log(logging.ERROR,"Exception caught. " + str(e))
        pass

