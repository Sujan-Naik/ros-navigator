#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov4 import Detector
from second_coursework.srv import yolo_detect, yolo_detectResponse
from second_coursework.msg import YOLODetection

class YOLO:
    def __init__(self):
        self.cv_image = None
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.yolo_srv = rospy.Service('/detect_frame', yolo_detect, self.yolo_service)


        self.detector = Detector(gpu_id=1, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path=f'/home/sujan/Documents/CS/Y2/ITR/cw2/ros_ws/src/second_coursework/config/coco.data')

    def image_callback(self, msg):
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def yolo_service(self, request):
        res = yolo_detectResponse()

        if self.cv_image is not None:
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                d = YOLODetection(detection.class_name)
                res.detections.append(d)
            return res

def start():
    try:
        rospy.init_node('yolo_robot')
    except rospy.exceptions.ROSException:
        print("Node has already been initialized")

    YOLO()


if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass