#!/usr/bin/env python
# -- coding: utf-8 --

import time
import logging
import sys
import argparse
import signal
import numpy as np

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

class PublisherThread(threading.Thread): 
    #PublisherLock = threading.Lock() 
 
    def __init__(self, scope, component, field, dataType, rate): 
        threading.Thread.__init__(self) 
        
        self.scope = scope
        self.component = component
        self.field = field
        self.dataType = dataType
        self.rate = rate
        self.running = False
         
    def run(self):  
        self.running = True
        try:
            rospy.loginfo("Setting up ROS publishers on parent scope: " + str(self.scope))
            
            try:
                dt = self.dataType
                if dt == "Wrench":
                    dt = Wrench
                else:
                    dt = Floats
            except AttributeError:
                dt = Floats
            except IndexError:
                dt = Floats
            
            self.publisher = rospy.Publisher(str(self.scope) + "/" + str(self.component.name) + "/" + str(self.field), dt, queue_size=1)
            rospy.loginfo("Added publisher for " + str(self.component.name) + "/" + str(self.field) + " with type " + str(self.dataType) + " and rate " + str(self.rate))
            self.ros_rate = rospy.Rate(self.rate)
            while self.running and not rospy.is_shutdown():
                #PublisherThread.PublisherLock.acquire()
                tmp = m3t.get_msg_field_value(self.component.status, self.field)
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
                self.publisher.publish(msg)
                #PublisherThread.PublisherLock.release()
                self.ros_rate.sleep()
        except ROSException as e:
            rospy.logerr("ROS exception caught! " + str(e))
        except:
            e = sys.exc_info()[0]
            rospy.logerr("Exception caught! " + str(e))
 
 
    def stop(self):
        self.publisher.unregister()
        #PublisherThread.PublisherLock.release()
        self.running = False
    
    def set_hz(self, rate):
        rospy.loginfo("Changing publish frequency for " + str(self.component.name) + "/" + str(self.field) + " with type " + str(self.dataType) + " to rate " + str(rate))
        self.rate = rate
        self.ros_rate = rospy.Rate(rate)
        

class MekaRosPublisher(object):

    def __init__(self, scope, components=[], fields=[], dataTypes=[], rate=[], serveronly=True):
            
        self.components_idx = components
        self.fields_idx = fields
        self.dataTypes_idx = dataTypes
        self.rates_idx = []
        self.publishers = {}
        print "arara"
        self.name = "meka_ros_publisher"

        self.lock = threading.Lock()

        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO)
        self.initialized = False
        self.scope = scope

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
        if(components == [] and not serveronly):
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
            if x not in self.comps.keys():
                self.comps[x] = mcf.create_component(x)
                self.rt_proxy.subscribe_status(self.comps[x])
            self.fields[x] = []
            

        self.repeated = False

        if(fields == []):
            for k, v in self.comps.iteritems():
                self.fields[k].append(self.get_field(v))
        else:
            for idx, val in enumerate(fields):
                temp = self.comps[names[idx]].status.DESCRIPTOR.fields_by_name.keys()
                if val in temp:
                    self.fields[names[idx]].append(val)
                
        if(rate == []):
            self.rates_idx.append(1)
            
        for r in rate:
            self.rates_idx.append(int(r))
        
        if(dataTypes == []):
            self.dataTypes_idx.append(str("Float"))

        rospy.loginfo("Subscribing to fields " + str(self.fields))
        rospy.loginfo("Rates " + str(self.rates_idx))
        rospy.loginfo("Types " + str(self.dataTypes_idx))

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


    def run(self, verbose):
        # yrange = None
        # scopez = m3t.M3Scope(xwidth=100, yrange=yrange)
       
        ros_rate = rospy.Rate(np.amax(self.rates_idx))
       
        try:
            rospy.loginfo("Starting "+str(len(self.components_idx))+" publish threads!")
            for idx, val in enumerate(self.components_idx):
                t = PublisherThread(self.scope, self.comps[val], self.fields_idx[idx], self.dataTypes_idx[idx], self.rates_idx[idx]) 
                with self.lock: 
                    self.publishers[(self.components_idx[idx], self.fields_idx[idx], self.dataTypes_idx[idx])] = t                    
                t.start()
                
            while True and not rospy.is_shutdown():
                ros_rate = rospy.Rate(np.amax(self.rates_idx))
                self.rt_proxy.step()
                ros_rate.sleep()
            
        except:
            e = sys.exc_info()[0]
            rospy.logerror("Error e: " + str(e) + ". Stopping threads!")

        for t in self.publishers:
            t.stop()
            t.join()
                
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
        
        rospy.loginfo("Requesting values for " + str(req.component) +" " + str(req.field) + " with " + str(req.hz) + " Hz.")
                
        values = []
        
        if req.component not in self.comps.keys():
            comp = mcf.create_component(req.component)
            self.comps[req.component] = comp 
            self.rt_proxy.subscribe_status(comp)
        
        rt_field_vals = m3t.get_msg_field_value(self.comps[req.component].status, req.field)
        
        if hasattr(rt_field_vals, '__len__'):
            for val in rt_field_vals:
                values.append(str(val))
        else:
            values.append(str(rt_field_vals))
        
        resp = RequestValuesResponse()
        resp.values = values
        
        dt = "Float"
        if (req.component, req.field, dt) not in self.publishers.keys():
            rospy.loginfo("adding publisher thread for " + str((req.component, req.field)))
            t = PublisherThread(self.scope, req.component, req.field, dt, req.hz) 
            with self.lock: 
                self.publishers[req.component, req.field, dt] = t
            t.start()
            rospy.loginfo("done")
        else:
            rospy.loginfo("publisher already exists")
            if req.hz != self.publishers[req.component, req.field, dt].rate:
                rospy.loginfo("adjusting rate...")
                self.publishers[req.component, req.field, dt].set_hz(req.hz)
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
        
        for t in self.publishers.values():
            t.stop()
            t.join()
        
        self.rt_proxy.stop(force_safeop=False)  # allow other clients to continue running

        print 'shutdown complete'

def signal_handler(signal, frame):
        mekarospub.disconnect()
        sys.exit(0)

def main():
    try:
        global mekarospub
        
        signal.signal(signal.SIGINT, signal_handler)
        
        # Set up logging.
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')
        # logging.getLogger("rsb").setLevel(logging.WARN)

        # Parse the command line options for the robot ip etc.
        parser = argparse.ArgumentParser(description='M3 system introspection publisher for ROS.')
        parser.add_argument("-s", "--scope", help="ROS scope prefix of the publisher", dest="scope", nargs=1, default=DEFAULT_PUBLISHER_SCOPE)
        parser.add_argument("-c", "--components", help="M3Component index", default=[], nargs='+')
        parser.add_argument("-f", "--fields", help="Field of selected m3component to publish", default=[], nargs='+')
        parser.add_argument("-t", "--datatypes", help="Data Type for each field", default=[], nargs='+')
        parser.add_argument("-r", "--rate", help="Rate to publish data at", default=[], nargs='+')
        parser.add_argument("-v", "--verbose", help="Be verbose", action="store_true")
        parser.add_argument("--server", help="Run in server-only mode", action="store_true")
        # parser.add_option("--yrange", help="Field of selected m3component to publish",
        #                  metavar="FIELD", dest="fields")
        
        args = parser.parse_args()
        
        mekarospub = MekaRosPublisher(args.scope, args.components, args.fields, args.datatypes, args.rate, args.server)

        if mekarospub.initialized:
            mekarospub.run(args.verbose)
        
        mekarospub.disconnect()
        
    except rospy.ROSInterruptException as e:
        logging.log(logging.ERROR, "Exception caught. " + str(e))
        pass
    except SystemExit as e:
        logging.log(logging.ERROR, "You must specifiy at least one argument for the arguments COMPONENTS, FIELDS, TYPES and RATE.")
        pass
    except m3t.M3Exception as e:
        logging.log(logging.ERROR, str(e))
        pass
    except:
        e = sys.exc_info()[0]
        logging.log(logging.ERROR, "Unknown error of type " + str(e))

if __name__ == '__main__':
    main()

