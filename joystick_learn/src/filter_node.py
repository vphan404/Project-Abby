#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class FilterNode:

    twist_pub = None
    twist_sub = None

    last_twist = Twist()

    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_sub = rospy.Subscriber('autonomous_driver/drive_cmd', Twist, self.autonomous_command_callback)

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)
            rate.sleep()

    def autonomous_command_callback(self, twist):
        new_twist = Twist()

        last_weight = 0.0

        new_twist.linear.x = self.last_twist.linear.x * last_weight + twist.linear.x * (1 - last_weight)

        new_twist.angular.z = self.last_twist.angular.z * last_weight + twist.angular.z * (1 - last_weight)

        self.last_twist = new_twist

if __name__ == '__main__':
    rospy.init_node('filter', log_level=rospy.DEBUG)

    node = FilterNode()
    node.publish()
    
    rospy.spin()