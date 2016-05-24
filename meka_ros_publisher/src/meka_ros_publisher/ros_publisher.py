#!/usr/bin/env python

import time
import logging
import sys
import argparse
from optparse import OptionParser

import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf

import rospy
from std_msgs.msg import String
from rospy.exceptions import ROSException

DEFAULT_NODE_NAME = "meka_ros_publisher"
DEFAULT_PUBLISHER_SCOPE = "/meka_ros_pub"
DEFAULT_HZ = 1

class MekaRosPublisher(object):

    def __init__(self, components=[], fields=[]):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO)
        self.initialized = False
        
        if(len(components) != len(fields)):
            rospy.logerr("Inequal amount of arguments for components and fields! Exiting..")
            
            return

        rospy.loginfo("Initializing m3rt proxy.")
        self.proxy = m3p.M3RtProxy()
        self.proxy.start()

        names = []
        comps = self.proxy.get_available_components()
        if(components == []):
            cid = self.get_component()
            names.append(comps[int(cid)])
        else:
            for c in components:
                if c in comps:
                    names.append(c)

        rospy.loginfo("Subscribing to components " + str(names))
        
        self.comps = []
        for x in names:
            self.comps.append(mcf.create_component(str(x)))
            self.proxy.subscribe_status(self.comps[-1])
        
        self.fields = []
        self.repeated = False
        if(fields == []):
            for x in self.comps:
                self.fields.append(self.get_field(x))
        else:
            i = 0
            for x in self.comps:
                temp = x.status.DESCRIPTOR.fields_by_name.keys()
                if fields[i] in temp:
                    self.fields.append(fields[i])
                i += 1

        rospy.loginfo("Subscribing to fields " + str(self.fields))

#       print 'Select Y-Range? [n]'
        yrange = None
#       if m3t.get_yes_no('n'):
#           yrange = []
#           print 'Min?'
#           yrange.append(m3t.get_int())
#           print 'Max?'
#           yrange.append(m3t.get_int())

        self.initialized = True


        # proxy.publish_param(comps)

    def run(self, scope):
        # yrange = None
        # scopez = m3t.M3Scope(xwidth=100, yrange=yrange)
        try:
            rospy.loginfo("Setting up ROS publishers on parent scope: " + str(scope))
            publishers = []
            for c in self.comps:
                idx = self.comps.index(c)
                publishers.append(rospy.Publisher(str(scope)+"/"+str(c.name)+"/"+str(self.fields[idx]), String, queue_size=1))
            rospy.loginfo("Setting up ROS node..")
            rospy.init_node("meka_ros_publisher", anonymous=True)
            rospy.loginfo("ROS node started..")
            rate = rospy.Rate(DEFAULT_HZ)
            ts = time.time()
            rospy.loginfo("Entering publish loop with HZ: " + str(DEFAULT_HZ))
            while True and not rospy.is_shutdown():
                self.proxy.step()
                for c in self.comps:
                    idx = self.comps.index(c)
                    if self.repeated:
                        v = m3t.get_msg_field_value(self.comps[idx].status, self.fields[idx])[self.idx]
                    else:
                        v = m3t.get_msg_field_value(self.comps[idx].status, self.fields[idx])
                    rospy.loginfo(v)
                    publishers[idx].publish(str(v))
                rate.sleep()

                # scope.plot(v)
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
            print 'Select index of repeated fields to monitor: [0]'
            self.idx = m3t.get_int(0)

        return field

    def disconnect(self):
        print 'disconnect called'
        self.proxy.stop(force_safeop=False)  # allow other clients to continue running

def main():
    try:
        # Set up logging.
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')
        # logging.getLogger("rsb").setLevel(logging.WARN)

        # Parse the command line options for the robot ip etc.
        parser = argparse.ArgumentParser(description='M3 system introspection publisher for ROS.')
        parser.add_argument("--scope", help="ROS scope of the publisher", default=DEFAULT_PUBLISHER_SCOPE, type=str)
        parser.add_argument("--components", help="M3Component index", default=[],
                          nargs='+')
        parser.add_argument("--fields", help="Field of selected m3component to publish", default=[],
                          nargs='+')
        # parser.add_option("--yrange", help="Field of selected m3component to publish",
        #                  metavar="FIELD", dest="fields")
        
        args = parser.parse_args()

        mekarospub = MekaRosPublisher(args.components, args.fields)
        if mekarospub.initialized:
            mekarospub.run(args.scope)
    
    except rospy.ROSInterruptException as e:
        logging.log(logging.ERROR, "Exception caught. " + str(e))
        print e
        pass
    except:
        e = sys.exc_info()[0]
        print e
    logging.log(logging.ERROR, "Unknown error!")

    mekarospub.disconnect()

if __name__ == '__main__':
   main()
