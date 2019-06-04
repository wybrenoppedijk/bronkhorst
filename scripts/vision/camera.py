import threading
import time
from collections import deque

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg
import sensor_msgs.msg
import sensor_msgs.srv


class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            # wait for an image (could happen at the very beginning when the queue is still empty)
            while len(self.queue) == 0:
                time.sleep(0.1)
            self.function(self.queue[0])


class CameraNode:
    def __init__(self):
            # assume any non-default service names have been set.  Wait for the service to become ready
        svcname = "camera"
        remapped = rospy.remap_name(svcname)
        if remapped != svcname:
            fullservicename = "%s/set_camera_info" % remapped
            print("Waiting for service", fullservicename, "...")
            try:
                rospy.wait_for_service(fullservicename, 5)
                print("OK")
            except rospy.ROSException:
                print("Service not found")
                rospy.signal_shutdown('Quit')

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)

        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"),
                                                          sensor_msgs.srv.SetCameraInfo)

        self.q_mono = deque([], 1)
        self.q_stereo = deque([], 1)
            self.br = cv_bridge.CvBridge()

        self.c = None
        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

    def handle_monocular(self, msg):
        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)

    def queue_monocular(self, msg):
        self.q_mono.append(msg)

    def mkgray(self, msg):
        """
        Convert a message into a 8-bit 1 channel monochrome OpenCV image
        """
        # as cv_bridge automatically scales, we need to remove that behavior
        if self.br.encoding_to_dtype_with_channels(msg.encoding)[0] in ['uint16', 'int16']:
            mono16 = self.br.imgmsg_to_cv2(msg, '16UC1')
            mono8 = numpy.array(mono16 / 256, dtype=numpy.uint8)
            return mono8
        elif 'FC1' in msg.encoding:
            # floating point image handling
            img = self.br.imgmsg_to_cv2(msg, "passthrough")
            _, max_val, _, _ = cv2.minMaxLoc(img)
            if max_val > 0:
                scale = 255.0 / max_val
                mono_img = (img * scale).astype(np.uint8)
            else:
                mono_img = img.astype(np.uint8)
            return mono_img
        else:
            return self.br.imgmsg_to_cv2(msg, "mono8")