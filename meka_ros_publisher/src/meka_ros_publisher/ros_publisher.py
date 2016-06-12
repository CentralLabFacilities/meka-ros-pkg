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
from geometry_msgs.msg import Wrench
from rospy.exceptions import ROSException

DEFAULT_NODE_NAME = "meka_ros_publisher"
DEFAULT_PUBLISHER_SCOPE = "/meka_ros_pub"
DEFAULT_HZ = 1

class MekaRosPublisher(object):

    def __init__(self, components=[], fields=[], dataTypes=[]):
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

	self.dataTypes = dataTypes

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

    def run(self, scope, verbose, rate):
        # yrange = None
        # scopez = m3t.M3Scope(xwidth=100, yrange=yrange)
        try:
            rospy.loginfo("Setting up ROS publishers on parent scope: " + str(scope))
            publishers = []
	    id=0;
            for c in self.comps:
                idx = self.comps.index(c)

		try:
			dt=self.dataTypes[id]
			if dt=="Wrench":
				dt=Wrench
			id = id+1
		except AttributeError:
			dt=String
		except IndexError:
			dt=String

		publishers.append(rospy.Publisher(str(scope)+"/"+str(c.name)+"/"+str(self.fields[idx]), dt, queue_size=1))
            rospy.loginfo("Setting up ROS node..")
            rospy.init_node("meka_ros_publisher", anonymous=True)
            rospy.loginfo("ROS node started..")
            ros_rate = rospy.Rate(rate)
            ts = time.time()
            rospy.loginfo("Entering publish loop with HZ: " + str(rate))
            while True and not rospy.is_shutdown():
		self.proxy.step()
                for c in self.comps:
                    idx = self.comps.index(c)
                    if self.repeated:
                        v = m3t.get_msg_field_value(self.comps[idx].status, self.fields[idx])[self.idx]
                    else:
                        v = m3t.get_msg_field_value(self.comps[idx].status, self.fields[idx])

                    if verbose:
			 rospy.loginfo(str(v))
		    if dt==Wrench:
				float_list = map(float, str(v).strip('[]').split(','))
				msg = Wrench()
			    	msg.force.x = float_list[0]
				msg.force.y = float_list[1]
		    		msg.force.z = float_list[2]

		    		msg.torque.x = float_list[3]
		    		msg.torque.y = float_list[4]
		    		msg.torque.z = float_list[5]
		    elif dt==String:
				    msg=str(v)
		    else:
				rospy.logerror("unknown Data type " + dt)


                    publishers[idx].publish(msg)
                ros_rate.sleep()

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


if __name__ == '__main__':
    try:
        # Set up logging.
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')
        # logging.getLogger("rsb").setLevel(logging.WARN)

        # Parse the command line options for the robot ip etc.
        parser = argparse.ArgumentParser(description='M3 system introspection publisher for ROS.')
        parser.add_argument("-s", "--scope-prefix", help="ROS scope prefix of the publisher", dest="scope", nargs=1, default=DEFAULT_PUBLISHER_SCOPE)
        parser.add_argument("-c", "--components", help="M3Component index", default=[], nargs='+')
        parser.add_argument("-f", "--fields", help="Field of selected m3component to publish", default=[], nargs='+')
        parser.add_argument("-t", "--data-types", help="Data Type for each field", default=[], nargs='+', dest='dataTypes')
        parser.add_argument("-r", "--rate", help="Rate to publish data at", default=DEFAULT_HZ, type=float)
        parser.add_argument("-v", "--verbose", help="Be verbose", action="store_true")
        # parser.add_option("--yrange", help="Field of selected m3component to publish",
        #                  metavar="FIELD", dest="fields")
        
        args = parser.parse_args()

        mekarospub = MekaRosPublisher(args.components, args.fields, args.dataTypes)
        if mekarospub.initialized:
            mekarospub.run(args.scope, args.verbose, args.rate)
	
	mekarospub.disconnect()
    
    except rospy.ROSInterruptException as e:
        logging.log(logging.ERROR, "Exception caught. " + str(e))
        print e
        pass
    except:
        e = sys.exc_info()[0]
        print e
    	logging.log(logging.ERROR, "Unknown error!")

