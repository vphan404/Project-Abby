#!/usr/bin/env python3

import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from enum import IntEnum
#from keras.models import model_from_json
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
#from skimage.measure import block_reduce
import cv2
import numpy as np
import os

ULTRASONIC_THRESH = 150

class Modes(IntEnum):
    USER_MODE = 1
    CRUISE_MODE = 2
    AUTOPILOT_MODE = 3
    DATA_COLLECTION_MODE = 4

class AutononmousDriverNode:
    
    selected_mode = Modes.USER_MODE

    twist_pub = None
    twist_sub = None
    ultrasonic_sub = None
    depth_camera_sub = None
    mode_sub = None
    mode_pub = None
    gps_pub = None
    gps_sub = None

    blocked_model = None
    orient_model = None

    last_twist = Twist()
    last_gps = NavSatFix()
    last_image = []
    last_was_blocked = False
    last_orient_direction = False

    blocked_sub = None
    right_sub = 0
    save_sub = None
    data_collection_is_blocked = False
    data_collection_is_right = False

    bridge = CvBridge()

    def __init__(self):
        self.set_pubs_and_subs()
        self.load_models()
        self.last_gps.latitude = 28.603882
        self.last_gps.longitude = -81.199260

    def set_pubs_and_subs(self):
        self.twist_pub = rospy.Publisher('autonomous_driver/drive_cmd', Twist, queue_size=10)
        self.twist_sub = rospy.Subscriber('joystick/drive_cmd', Twist, self.joystick_command_callback)
        self.ultrasonic_sub = rospy.Subscriber('arduino/ultrasonic_ranges', Int32MultiArray, self.perimeter_check)
        self.depth_camera_sub = rospy.Subscriber('/mynteye/depth/image_raw', Image, self.depth_callback)
        self.mode_sub = rospy.Subscriber('joystick/mode', Int32, self.change_mode)
        self.mode_pub = rospy.Publisher('joystick/mode', Int32, queue_size=10)
        self.blocked_sub = rospy.Subscriber('joystick/blocked', Bool, self.change_blocked)
        self.right_sub = rospy.Subscriber('joystick/right', Int32, self.change_right)
        self.save_sub = rospy.Subscriber('joystick/save', Empty, self.save_image)
        self.gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=10)

    def change_blocked(self, blocked):
        self.data_collection_is_blocked = blocked.data

    def change_right(self, right):
        self.data_collection_is_right = right.data

    def depth_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        
        scale_factor = 20
        dim = (scale_factor, scale_factor) 

        resized = image # block_reduce(image, block_size=dim, func=np.max)

        self.last_image = resized

        if self.selected_mode == Modes.CRUISE_MODE:
            self.cruise()

    def save_image(self, empty):
        if self.last_image == []:
            return
        
        path = '/home/abe/Pictures/'

        if self.data_collection_is_blocked:
            path = path + 'blocked/'

            if self.data_collection_is_right == 1:
                path = path + 'right/'
            elif self.data_collection_is_right == 0:
                path = path + 'left/'
            else:
                path = path + 'either/'
        else:
            path = path + 'unblocked/'

        path = path + 'depth-' + datetime.now().strftime('%m%d%Y-%H%M%S') + '.png'
        
        result = cv2.imwrite(path, self.last_image)
        rospy.loginfo("Image saved: " + path + " " + str(result))

    def cruise(self):
        rospy.loginfo('cruising')
        blocked_prediction = self.blocked_model.predict(self.last_image.reshape(1,24,38,1))[0]

        twist = Twist()

        twist.linear.x = 0.75

        if blocked_prediction <= 0.5:
            self.last_was_blocked = False

            twist.angular.z = 0

            self.last_twist = twist

            return

        if self.last_was_blocked:
            return

        self.last_was_blocked = True

        orient_direction = self.orient_model.predict(self.last_image.reshape(1,24,38,1))[0]

        if orient_direction <= 0.5:
            twist.angular.z = -1

            self.last_twist = twist
        else:
            twist.angular.z = 1

            self.last_twist = twist

        self.last_orient_direction = orient_direction


    def autopilot(self):
        return

    def change_mode(self, mode):
        rospy.loginfo("Mode changed: " + str(mode.data))
        self.selected_mode = mode.data

    def load_models(self):
        return
        # Load blocked model
        # json_file = open('models1/blocked_model.json', 'r')
        # loaded_model_json = json_file.read()
        # self.blocked_model = model_from_json(loaded_model_json)
        # self.blocked_model.load_weights('models1/blocked_model.h5')

        # # Load orient model
        # json_file = open('models1/orient_model.json', 'r')
        # loaded_model_json = json_file.read()
        # self.orient_model = model_from_json(loaded_model_json)
        # self.orient_model.load_weights('models1/orient_model.h5')

        # json_file.close()

    def perimeter_check(self, ultrasonic_ranges):
        if not self.perimeter_clear(ultrasonic_ranges) and self.selected_mode == Modes.CRUISE_MODE:
            self.selected_mode = Modes.USER_MODE
            mode = Int32()
            mode.data = int(Modes.USER_MODE)
            self.mode_pub.publish(mode)

    def perimeter_clear(self, ultrasonic_ranges):
        for r in ultrasonic_ranges.data:
            if r < ULTRASONIC_THRESH:
                return False
        return True

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)
            rate.sleep()

    def joystick_command_callback(self, twist):
        # Pass through drive commands if in USER_MODE
        if self.selected_mode == Modes.USER_MODE:
            self.last_was_blocked = False
            self.last_twist = twist

if __name__ == '__main__':
    rospy.init_node('autonomous_driver', log_level=rospy.DEBUG)

    node = AutononmousDriverNode()
    node.publish()
    
    rospy.spin()