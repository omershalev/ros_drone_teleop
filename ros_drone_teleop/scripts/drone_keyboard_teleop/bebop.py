#!/usr/bin/env python

import rospy
import os
from drone_keyboard_teleop.keyboard_teleop_base import KeyboardTeleop
from std_msgs.msg import Empty as EmptyMsg


class BebopKeyboardTeleop(KeyboardTeleop):
    def __init__(self, namespace):
        super(BebopKeyboardTeleop, self).__init__(namespace)

        self.takeoff_publisher = rospy.Publisher(self.namespace + 'takeoff', EmptyMsg, queue_size=10)
        self.land_publisher = rospy.Publisher(self.namespace + 'land', EmptyMsg, queue_size=10)
        self.reset_publisher = rospy.Publisher(self.namespace + 'reset', EmptyMsg, queue_size=10)
        self.flat_trim_publisher = rospy.Publisher(self.namespace + 'flattrim', EmptyMsg, queue_size=10)

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
            msg = EmptyMsg()
            rospy.sleep(1)
            self.flat_trim_publisher.pub(msg)
