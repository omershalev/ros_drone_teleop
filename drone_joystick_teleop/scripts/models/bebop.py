#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class BebopJoystickTeleop(object):
    def __init__(self, namespace):
        self.namespace = namespace
        self.takeoff_publisher = rospy.Publisher(self.namespace + 'takeoff', EmptyMsg, queue_size=10)
        self.land_publisher = rospy.Publisher(self.namespace + 'land', EmptyMsg, queue_size=10)
        self.reset_publisher = rospy.Publisher(self.namespace + 'reset', EmptyMsg, queue_size=10)
        self.navigate_home_publisher = rospy.Publisher(self.namespace + 'autoflight/navigate_home', Bool, queue_size=10)
        self.flat_trim_publisher = rospy.Publisher(self.namespace + 'flattrim', EmptyMsg, queue_size=10)
        self.camera_control_publisher = rospy.Publisher(self.namespace + 'camera_control', Twist, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.empty_msg = EmptyMsg()

    def create_twist_message(self, l_x, l_y, l_z, a_x, a_y, a_z):
        twist = Twist()
        twist.linear.x = l_x
        twist.linear.y = l_y
        twist.linear.z = l_z
        twist.angular.x = a_x
        twist.angular.y = a_y
        twist.angular.z = a_z
        return twist

    def joy_callback(self, msg):
        if msg.buttons[13] == 1 and msg.buttons[11] == 1 and msg.buttons[9] == 1:
            self.reset_publisher.publish(self.empty_msg)
        elif msg.buttons[14] == 1 and msg.buttons[11] == 1:
            self.land_publisher.publish(self.empty_msg)
        elif msg.buttons[12] == 1 and msg.buttons[11] == 1:
            self.takeoff_publisher.publish(self.empty_msg)
        elif msg.buttons[15] == 1 and msg.buttons[11] == 1:
            bool_msg = Bool(True)
            self.navigate_home_publisher.publish(bool_msg)
        elif msg.buttons[3] == 1:
            self.flat_trim_publisher.publish(self.empty_msg)
        elif msg.axes[8] != 0:
            twist = self.create_twist_message(0,0,0,0,0,0)
            self.camera_control_publisher.publish(twist)
        elif msg.axes[8] != 1.0:
            twist = self.create_twist_message(0,0,0,0,0,0)
            self.camera_control_publisher.publish(twist)
        elif msg.axes[10] != 1.0:
            twist = self.create_twist_message(0,0,0,0,-90,0)
            self.camera_control_publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('drone_teleop', anonymous=True)
    namespace = rospy.get_namespace()
    BebopJoystickTeleop(namespace)
    rospy.spin()