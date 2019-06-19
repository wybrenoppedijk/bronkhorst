#!/usr/bin/env python
import threading
import time
from collections import deque

import cv2
import yaml
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg
import sensor_msgs.msg
import sensor_msgs.srv
import Queue
import detector

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)


class LfeDetector:

    def __init__(self):
            # assume any non-default service names have been set.  Wait for the service to become ready
        cal_params = self.parse_calibration_file('/home/wybren/catkin_ws/src/bronkhorst/calibration/manta.yaml')

        self.cm, self.coeffs, w, h = cal_params
        self.ncm, _ = cv2.getOptimalNewCameraMatrix(self.cm, self.coeffs,
                                                    (w, h), 0, (w, h))

        self.q_mono = Queue.Queue()

        self.bridge = CvBridge()
        msub = message_filters.Subscriber('camera/image_raw', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)

        mth = ConsumerThread(self.q_mono, self.handle_img_msg)
        mth.setDaemon(True)
        mth.start()

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def msg_to_img(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        return cv2.undistort(image, self.cm, self.coeffs, None, self.ncm)

    def handle_img_msg(self, msg):
        undistorted_img = self.msg_to_img(msg)
        # detector.draw_circles(undistorted_img, detector.detect_circles(undistorted_img))
        cv2.imshow('u', undistorted_img)
        cv2.waitKey(4)

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
    rospy.init_node('lfe_detector')
    LfeDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
