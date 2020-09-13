#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from enum import IntEnum

LINEAR_AXIS = 4
ANGULAR_AXIS = 0
    
LINEAR_SCALE = 1
ANGULAR_SCALE = 1

GATE_THRESH = 0.05

class Modes(IntEnum):
    USER_MODE = 1
    CRUISE_MODE = 2
    AUTOPILOT_MODE = 3
    DATA_COLLECTION_MODE = 4

class TeleopJoyNode:
    
    twist_pub = None
    mode_pub = None
    blocked_pub = None
    right_pub = None
    save_pub = None
    joy_sub = None
    last_twist = Twist()
    last_joy = None

    reverse_selected = False

    def __init__(self):
        self.twist_pub = rospy.Publisher('joystick/drive_cmd', Twist, queue_size=10)
        self.mode_pub = rospy.Publisher('joystick/mode', Int32, queue_size=10)
        self.blocked_pub = rospy.Publisher('joystick/blocked', Bool, queue_size=10)
        self.right_pub = rospy.Publisher('joystick/right', Int32, queue_size=10)
        self.save_pub = rospy.Publisher('joystick/save', Empty, queue_size=10)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)
            rate.sleep()

    def joy_callback(self, joy_msg):
        twist = Twist()
        mode = Int32()
        blocked = Bool()
        right = Int32()

        if joy_msg.buttons[3] == 1: # A button
            mode.data = int(Modes.USER_MODE)
            self.mode_pub.publish(mode)
        if joy_msg.buttons[1] == 1: # Y button
            mode.data = int(Modes.CRUISE_MODE)
            self.mode_pub.publish(mode)

        if self.last_joy != None and self.last_joy.buttons[6] == 0 and joy_msg.buttons[6] == 1: # LB button
            blocked.data = False
            self.blocked_pub.publish(blocked)
        if self.last_joy != None and self.last_joy.buttons[7] == 0 and joy_msg.buttons[7] == 1: # Back button
            blocked.data = True
            self.blocked_pub.publish(blocked)
        if self.last_joy != None and self.last_joy.buttons[2] == 0 and joy_msg.buttons[2] == 1: # Power button
            right.data = 0
            self.right_pub.publish(right)
        if self.last_joy != None and self.last_joy.buttons[0] == 0 and joy_msg.buttons[0] == 1: # Start button
            right.data = 1
            self.right_pub.publish(right)
        if self.last_joy != None and self.last_joy.buttons[9] == 0 and joy_msg.buttons[9] == 1: # Right Stick button
            right.data = -1
            self.right_pub.publish(right)
        if self.last_joy != None and self.last_joy.axes[7] == 0 and joy_msg.axes[7] == -1: # D-Down button
            self.save_pub.publish(Empty())

        linear_x = -(joy_msg.axes[LINEAR_AXIS] - 1) / 2
        angular_z = -ANGULAR_SCALE * joy_msg.axes[ANGULAR_AXIS]

        if self.last_joy != None and self.last_joy.buttons[5] == 0 and joy_msg.buttons[5] == 1 and linear_x == 0: # RB button
            self.reverse_selected = not self.reverse_selected

        if linear_x > GATE_THRESH:
            if self.reverse_selected:
                twist.linear.x = -linear_x
            else:
                twist.linear.x = linear_x
        
        if abs(angular_z) > GATE_THRESH:
            twist.angular.z = angular_z
        
        self.last_twist = twist
        self.last_joy = joy_msg
        self.twist_pub.publish(self.last_twist)

if __name__ == '__main__':
    rospy.init_node('teleop_joystick', log_level=rospy.DEBUG)

    node = TeleopJoyNode()
    node.publish()
    
    rospy.spin()