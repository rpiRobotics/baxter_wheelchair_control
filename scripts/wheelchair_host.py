#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Empty

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import geometry_msgs.msg

wheelchair_servicedef="""
#Service to provide simple interface to Baxter
service Wheelchair_Interface

option version 0.4

object Wheelchair
function void setWVWheelchair(double[] command)
end object

"""
class Wheelchair_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('wheelchair')
        
        # data initializations
	self._wv_wheelchair = [0]*2;
	self._cmd_vel_msg = geometry_msgs.msg.Twist()
        
        # initial joint command is current pose
	self.setWVWheelchair(self._wv_wheelchair)
                
        # Start background threads
        self._running = True
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()

    def close(self):
        self._running = False
        self._t_command.join()

    def setWVWheelchair(self, command):
	self._wv_wheelchair = command
	self._cmd_vel_msg.linear.x = command[1]
	self._cmd_vel_msg.linear.y = 0.0
	self._cmd_vel_msg.linear.z = 0.0
	self._cmd_vel_msg.angular.x = 0.0
	self._cmd_vel_msg.angular.y = 0.0
	self._cmd_vel_msg.angular.z = command[0]
    
    # worker function to continuously issue commands to baxter
    # Try to maintain 100 Hz operation
    # TODO: INCLUDE CLOCK JITTER CORRECTION
    def command_worker(self):
	WVWheelchair_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
	
        while self._running:
            t1 = time.time()
	    WVWheelchair_pub.publish(self._cmd_vel_msg)
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Wheelchair Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="WheelchairServer"

    #Initialize object
    wheelchair_obj = Wheelchair_impl()

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()

    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(wheelchair_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Wheelchair",
                      "Wheelchair_Interface.Wheelchair",
                                          wheelchair_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/WheelchairServer/Wheelchair"
    raw_input("press enter to quit...\r\n")
    
    wheelchair_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
