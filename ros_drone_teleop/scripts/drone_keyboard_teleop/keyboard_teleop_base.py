#!/usr/bin/env python

import rospy
import sys
import os
import datetime
import tty
import select
import termios
from geometry_msgs.msg import Twist


class KeyboardTeleop(object):
    def __init__(self, namespace):
        self.namespace = namespace

        self.response_factor = 1
        self.last_keystroke_time = None
        self.is_last_twist_zero = False
        self.need_to_exit = False

        self.output_topic = self.namespace + 'cmd_vel'
        self.vel_publisher = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)

    def create_twist_message(self, l_x, l_y, l_z, a_x, a_y, a_z):
        twist = Twist()
        twist.linear.x = l_x
        twist.linear.y = l_y
        twist.linear.z = l_z
        twist.angular.x = a_x
        twist.angular.y = a_y
        twist.angular.z = a_z
        return twist

    def send_twist(self, twist):
        self.vel_publisher.publish(twist)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            self.last_keystroke_time = datetime.datetime.now()
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_model_specific_status(self):
        pass

    def check_model_specific_operations(self, ch):
        pass

    def print_model_specific_operations(self):
        pass

    def print_message(self):
        os.system('clear')
        print '===================== Status ======================='
        print 'Velocity factor : ' + str(self.response_factor)
        print 'Velocity topic  : ' + self.output_topic
        self.print_model_specific_status()
        print '================ Teleoperation keys ================'
        print '\'q\' - ascend'
        print '\'a\' - descend'
        print '\'i\' - forward'
        print '\'k\' - backward'
        print '\'l\' - left'
        print '\'j\' - right'
        print '\'o\' - rotate cw'
        print '\'u\' - rotate ccw'
        self.print_model_specific_operations()
        print '====================== Misc ========================'
        print '\'+\' - increase response'
        print '\'-\' - decrease response'
        print '\'x\' - exit'

    def run(self):
        while True:
            self.print_message()
            try:
                ch = self.get_key()
                self.check_model_specific_operations(ch)
                if ch == 'x':
                    self.need_to_exit = True
                    break
                elif ch == 'l':
                    twist = self.create_twist_message(0,-self.response_factor,0,0,0,0)
                    self.send_twist(twist)
                elif ch == 'j':
                    twist = self.create_twist_message(0,self.response_factor,0,0,0,0)
                    self.send_twist(twist)
                elif ch == 'i':
                    twist = self.create_twist_message(self.response_factor,0,0,0,0,0)
                    self.send_twist(twist)
                elif ch == 'k':
                    twist = self.create_twist_message(-self.response_factor,0,0,0,0,0)
                    self.send_twist(twist)
                elif ch == 'o':
                    twist = self.create_twist_message(0,0,0,0,0,-0.8*self.response_factor)
                    self.send_twist(twist)
                elif ch == 'u':
                    twist = self.create_twist_message(0,0,0,0,0,0.8*self.response_factor)
                    self.send_twist(twist)
                elif ch == 'q':
                    twist = self.create_twist_message(0,0,1.3*self.response_factor,0,0,0)
                    self.send_twist(twist)
                elif ch == 'a':
                    twist = self.create_twist_message(0,0,-1.3*self.response_factor,0,0,0)
                    self.send_twist(twist)
                elif ch == '+':
                    self.response_factor = self.response_factor + 0.2
                elif ch == '-':
                    if self.response_factor - 0.2 > 0.1:
                        self.response_factor = self.response_factor - 0.2
                elif ch == '':
                    twist = self.create_twist_message(0,0,0,0,0,0)
                    self.send_twist(twist)

            except Exception, e:
                print 'Exception was thrown - ignoring'
                print e
