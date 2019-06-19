#!/usr/bin/env python
import threading
import time
from collections import deque

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg
import sensor_msgs.msg
import sensor_msgs.srv
import detector
import yaml
import Queue
import tf
import time
from bronkhorst.msg import LfeCoordinate

class LfeDetector:

    def __init__(self):
            # assume any non-default service names have been set.  Wait for the service to become ready
        cal_params = self.parse_calibration_file('/home/wybren/catkin_ws/src/bronkhorst/calibration/manta.yaml')

        self.cm, self.coeffs, w, h = cal_params
        self.ncm, _ = cv2.getOptimalNewCameraMatrix(self.cm, self.coeffs,
                                                    (w, h), 0, (w, h))
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('lfe_coordinate', LfeCoordinate, queue_size=1)
        self.tf_br = tf.TransformBroadcaster()
        rospy.init_node('lfe_detector')
        rospy.sleep(0.1)
        img_sub = message_filters.Subscriber('camera/image_raw', sensor_msgs.msg.Image)
        img_sub.registerCallback(self.handle_img_msg)


    def msg_to_img(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        return cv2.undistort(image, self.cm, self.coeffs, None, self.ncm)

    def handle_img_msg(self, msg):
        lfe_property = detector.get_lfe_property(self.msg_to_img(msg))
        if lfe_property:
            msg = LfeCoordinate()
            msg.x_axe = float(lfe_property.x_axe)
            msg.y_axe = float(lfe_property.y_axe)
            msg.upside = bool(lfe_property.upside)

            self.pub.publish(msg)

    def parse_calibration_file(self, path):
        with open(path, 'r') as fobj:
            text = fobj.read()

        cal_pars = yaml.load(text)

        camera_matrix = np.array(
            cal_pars['camera_matrix']['data']).reshape((3, 3))
        distortion_coefficients = np.array(
            cal_pars['distortion_coefficients']['data'])
        width = cal_pars['image_width']
        height = cal_pars['image_height']

        return camera_matrix, distortion_coefficients, width, height


def main():
    LfeDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
