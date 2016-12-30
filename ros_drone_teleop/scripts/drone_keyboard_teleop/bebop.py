#!/usr/bin/env python

import rospy
import os
from drone_keyboard_teleop.keyboard_teleop_base import KeyboardTeleop
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class BebopKeyboardTeleop(KeyboardTeleop):
    def __init__(self, namespace):
        super(BebopKeyboardTeleop, self).__init__(namespace)

        self.takeoff_publisher = rospy.Publisher(self.namespace + 'takeoff', EmptyMsg, queue_size=10)
        self.land_publisher = rospy.Publisher(self.namespace + 'land', EmptyMsg, queue_size=10)
        self.reset_publisher = rospy.Publisher(self.namespace + 'reset', EmptyMsg, queue_size=10)
        self.navigate_home_publisher = rospy.Publisher(self.namespace + 'autoflight/navigate_home', Bool, queue_size=10)
        self.flat_trim_publisher = rospy.Publisher(self.namespace + 'flattrim', EmptyMsg, queue_size=10)
        self.camera_control_publisher = rospy.Publisher(self.namespace + 'camera_control', Twist, queue_size=10)

    def print_model_specific_operations(self):
        print '================ Bebop  Operations ================='
        print '\'f\' - flat trim'
        print '\'t\' - takeoff'
        print '\'d\' - land'
        print '\'e\' - emergency shutdown'
        print '\'h\' - return to home'
        print '\'c\' - change camera orientation'


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
        elif ch == 'h':
            twist = self.create_twist_message(0,0,0,0,0,0)
            self.send_twist(twist)
            msg = Bool(True)
            rospy.sleep(1)
            self.navigate_home_publisher.publish(msg)
        elif ch == 'f':
            msg = EmptyMsg()
            rospy.sleep(1)
            self.flat_trim_publisher.publish(msg)
        elif ch == 'c':
            os.system('clear')
            print 'Enter the desired tilt in degrees'
            tilt = input()
            print 'Enter the desired pan in degrees'
            pan = input()
            if not isinstance(tilt, int) or not isinstance(pan, int):
                print 'Invalid input'
                return
            twist = self.create_twist_message(0,0,0,0,tilt,pan)
            self.send_twist(twist)
