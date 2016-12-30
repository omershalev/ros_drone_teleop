#!/usr/bin/env python

import rospy
import os
from drone_keyboard_teleop.keyboard_teleop_base import KeyboardTeleop
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv


class ArDroneKeyboardTeleop(KeyboardTeleop):
    def __init__(self, namespace):
        super(ArDroneKeyboardTeleop, self).__init__(namespace)

        rospy.Subscriber(self.namespace + 'navdata', Navdata, self.navdata_callback)
        self.takeoff_publisher = rospy.Publisher(self.namespace + 'takeoff', EmptyMsg, queue_size=10)
        self.flat_trim_handle = rospy.ServiceProxy(self.namespace + 'flattrim', EmptySrv)

        self.navdata_msg = None

        self.land_publisher = rospy.Publisher(self.namespace + 'land', EmptyMsg, queue_size=10)
        self.reset_publisher = rospy.Publisher(self.namespace + 'reset', EmptyMsg, queue_size=10)

    def navdata_callback(self, msg):
        self.navdata_msg = msg

    def print_model_specific_status(self):
        if self.navdata_msg != None:
            print 'Bettery         : ' + str(self.navdata_msg.batteryPercent) + '%'
            print 'Altitude        : ' + str(self.navdata_msg.altd/1000) + ' m'
            print 'Yaw             : ' + str(self.navdata_msg.rotZ) + ' degrees'
            print 'Active camera   : TODO'

    def print_model_specific_operations(self):
        print '=============== AR-Drone  Operations ==============='
        print '\'f\' - flat trim'
        print '\'t\' - takeoff'
        print '\'d\' - land'
        print '\'e\' - emergency shutdown'

    def check_model_specific_operations(self, ch):
        if ch == 't':
            twist = self.create_twist_message(0,0,0,0,0,0)
            self.send_twist(twist)
            msg = EmptyMsg()
            rospy.sleep(1)
            self.takeoff_publisher.publish(msg)
        elif ch == 'd':
            twist = self.create_twist_message(0,0,0,0,0,0)
            self.send_twist(twist)
            msg = EmptyMsg()
            rospy.sleep(1)
            self.land_publisher.publish(msg)
        elif ch == 'e':
            os.system('clear')
            print 'This will shutdown immediately the drone - are you sure? (y/n)'
            ch = self.get_key()
            while ch not in ['y', 'n']:
                ch = self.get_key()
            if ch == 'y':
                msg = EmptyMsg()
                rospy.sleep(1)
                self.reset_publisher.publish(msg)
        elif ch == 'f':
            self.flat_trim_handle()