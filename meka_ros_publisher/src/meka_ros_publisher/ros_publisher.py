#!/usr/bin/env python
# -- coding: utf-8 --

import time
import logging
import sys
import argparse
from optparse import OptionParser

import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf

import rospy
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Wrench
from meka_ros_publisher.srv import ListComponents, ListComponentsResponse, ListFields, ListFieldsResponse, RequestValues, RequestValuesResponse

import threading

from rospy.exceptions import ROSException

DEFAULT_NODE_NAME = "meka_ros_publisher"
DEFAULT_PUBLISHER_SCOPE = "/meka_ros_pub"
DEFAULT_HZ = 1

class MekaRosPublisher(object):

    def __init__(self, scope, components=[], fields=[], dataTypes=[]):
        self.name = "meka_ros_publisher"

        self.lock = threading.Lock()

        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO)
        self.initialized = False
        self.scope = scope

        self.publishers = {}

        if(len(components) != len(fields)):
            rospy.logerr("Inequal amount of arguments for components and fields! Exiting..")
            return

        rospy.loginfo("Initializing m3rt rt_proxy.")
        self.rt_proxy = m3p.M3RtProxy()
        self.rt_proxy.start()

        rospy.loginfo("Setting up ROS node..")
        rospy.init_node(self.name, anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("ROS node started..")

        self.init_services()

        names = []
        comps = self.rt_proxy.get_available_components()
        if(components == []):
            cid = self.get_component()
            names.append(comps[int(cid)])
        else:
            for c in components:
                if c in comps:
                    names.append(c)

        rospy.loginfo("Subscribing to components " + str(names))

        self.comps = {}
        self.fields = {}
        
        for x in names:
            self.comps[x] = mcf.create_component(x)
            self.fields[x] = []
            self.rt_proxy.subscribe_status(self.comps[x])

        self.repeated = False
        if(fields == []):
            for k, v in self.comps.iteritems():
                self.fields[k].append(self.get_field(v))
        else:
            i = 0
            for k, v in self.comps.iteritems():
                temp = v.status.DESCRIPTOR.fields_by_name.keys()
                if fields[i] in temp:
                    self.fields[k].append(fields[i])
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


        # rt_proxy.publish_param(comps)

    def init_services(self):
        """
        Initialize the service servers
        """
        service_prefix = rospy.get_name() + "/"

        self._request_components_serv = rospy.Service(service_prefix +
                                                'list_components',
                                                ListComponents,
                                                self.get_components)
        self._request_fields_serv = rospy.Service(service_prefix +
                                                'list_fields',
                                                ListFields,
                                                self.get_fields)
        self._request_values_serv = rospy.Service(service_prefix +
                                                'request_values',
                                                RequestValues,
                                                self.get_values)


    def run(self, verbose, rate):
        # yrange = None
        # scopez = m3t.M3Scope(xwidth=100, yrange=yrange)
       
        try:
            rospy.loginfo("Setting up ROS publishers on parent scope: " + str(self.scope))

            id = 0;
            for k, v in self.comps.iteritems():
                try:
                    dt = self.dataTypes[id]
                    if dt == "Wrench":
                        dt = Wrench
                    id = id + 1
                except AttributeError:
                    dt = Floats
                except IndexError:
                    dt = Floats
                for field in self.fields[k]:
                    self.publishers[(k, field)] = rospy.Publisher(str(self.scope) + "/" + k + "/" + field, dt, queue_size=1)
                    rospy.loginfo("Added publisher for " + str((k, field)) + " with type " + str(dt))
            self.ros_rate = rospy.Rate(rate)
            ts = time.time()
            rospy.loginfo("Entering publish loop with HZ: " + str(rate))
            while True and not rospy.is_shutdown():
                self.rt_proxy.step()
                self.lock.acquire()
                for k, v in self.publishers.iteritems():
                    tmp = m3t.get_msg_field_value(self.comps[k[0]].status, k[1])
                    if dt == Wrench:
                        float_list = map(float, str(tmp).strip('[]').split(','))
                        msg = Wrench()
                        msg.force.x = float_list[0]
                        msg.force.y = float_list[1]
                        msg.force.z = float_list[2]

                        msg.torque.x = float_list[3]
                        msg.torque.y = float_list[4]
                        msg.torque.z = float_list[5]
                    elif dt == Floats:
                        msg = Floats()
                        if hasattr(tmp, '__len__'):
                            for val in tmp:
                                msg.data.append(val)
                        else:
                            msg.data.append(tmp)
                        
                    else:
                        rospy.logerr("unknown Data type " + dt)
                    v.publish(msg)
                self.lock.release()
                self.ros_rate.sleep()

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


        self.rt_proxy.stop(force_safeop=False)  # allow other clients to continue running

    def get_components(self, req):
        """
        Callback for the get_components service request
        """
        request_name = req.request

        names = []
        if(request_name == ""):
            comps = self.rt_proxy.get_available_components()  # get all
        else:
            comps = self.rt_proxy.get_available_components(request_name)

        for c in comps:
            names.append(str(c))

        resp = ListComponentsResponse(names)

        return resp


    def get_fields(self, req):
        """
        Callback for the get_fields service request
        """
        request_name = req.component

        if request_name not in self.comps.keys():
            try:
                comp = mcf.create_component(str(request_name))
                self.rt_proxy.subscribe_status(comp)
            except AttributeError, e:
                rospy.logwarn("No fields available for %s", request_name)
                return
            self.comps[request_name] = comp
            
        else:
            comp = self.comps[request_name]
            
        fields = comp.status.DESCRIPTOR.fields_by_name.keys()

        names = []
        for f in fields:
            names.append(str(f))

        resp = ListFieldsResponse(names)

        return resp

    def get_values(self, req):
        """
        Callback for the get_values service request
        """
        
        rospy.loginfo("Requesting values for " + str(req.component) +" " + str(req.field))
        
        request_component = req.component
        request_field = req.field
        self.ros_rate = rospy.Rate(req.hz) #adjust rate. careful: for all!
        rospy.loginfo("Changing HZ of publish loop: " + str(req.hz))
        
        values = []
        
        if request_component not in self.comps.keys():
            comp = mcf.create_component(str(request_component))
            self.comps[request_component] = comp 
            self.rt_proxy.subscribe_status(comp)
        
        rt_field_vals = m3t.get_msg_field_value(self.comps[request_component].status, request_field)
        
        if hasattr(rt_field_vals, '__len__'):
            for val in rt_field_vals:
                values.append(str(val))
        else:
            values.append(str(rt_field_vals))
        
        resp = RequestValuesResponse()
        resp.values = values
        
        dt = Floats
        if (str(req.component), str(req.field)) not in self.publishers.keys():
            rospy.loginfo("adding publisher for " + str((req.component, req.field)))
            with self.lock:    
                self.publishers[(req.component, req.field)] = rospy.Publisher(str(self.scope) + "/" + req.component + "/" + request_field, dt, queue_size=1)
            rospy.loginfo("done")
        return resp

    def get_component(self):
        comps = self.rt_proxy.get_available_components()
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
        self.rt_proxy.stop(force_safeop=False)  # allow other clients to continue running

def main():
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

        mekarospub = MekaRosPublisher(args.scope, args.components, args.fields, args.dataTypes)
        if mekarospub.initialized:
            mekarospub.run(args.verbose, args.rate)

        mekarospub.disconnect()

    except rospy.ROSInterruptException as e:
        logging.log(logging.ERROR, "Exception caught. " + str(e))
        print e
        pass
    except:
        e = sys.exc_info()[0]
        print e
        logging.log(logging.ERROR, "Unknown error!")

if __name__ == '__main__':
    main()

