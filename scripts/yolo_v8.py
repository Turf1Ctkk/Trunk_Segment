#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
import time
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from sensor_msgs.msg import Image
#from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes, Contour, ContourArray
from cv_bridge import CvBridge, CvBridgeError
#import message_filters

fx = 598.7568359375
fy = 598.7568969726562
cx = 326.3443298339844
cy = 250.24488830566406
BUFF_SIZE = 52428800
CONF = 0.7
HEIGHT = 500

class YOLO_Segment:
    def __init__(self):
        # Check device
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"Using device: {self.device}")
        # YOLO interface
        self.model = YOLO(rospy.get_param('~weight_path', ''))
        self.model.fuse()

        self.bridge = CvBridge()
        self.getImageStatus = False

        # Subscribe from depth camera messages
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback, queue_size=1, buff_size=BUFF_SIZE)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1, buff_size=BUFF_SIZE)

        # If no image messages
        while not self.getImageStatus:
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)    

    def depth_callback(self, data):
        # Convert raw depth data to numpy arrays
        try:
        #   self.depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width)
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

    def find_ymax_point(self, contour):
        ymax_value = float('-inf')
        ymax_point = None

        for points in contour:
            for point in points:
                x, y = int(point[0]), int(point[1])
                y_real = (y - cy) * 0.001 * self.depth_image[y, x]
                if y_real > ymax_value:
                    ymax_value = y_real
                    ymax_point = point

        return ymax_point

    def height_filter(self, contour):
        if self.depth_image is None or len(contour) == 0:
            rospy.logerr("Depth image is empty or contour is empty.")
            return []

        max_y_point = self.find_ymax_point(contour)
        if max_y_point is None:
            return []

        x, y = int(max_y_point[0]), int(max_y_point[1])
        max_real_y = (y - cy) * 0.001 * self.depth_image[y, x]
        upper_edge = max_real_y - HEIGHT

        collected_contour = [
            [int(point[0]), int(point[1])] for points in contour for point in points
            if upper_edge <= (int(point[1]) - cy) * 0.001 * self.depth_image[int(point[1]), int(point[0])] <= max_real_y
        ]

        return collected_contour

    def color_callback(self, data):
        self.getImageStatus = True
        # Convert raw color data to numpy arrays
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        cv2.imshow('Origin', self.color_image)
        results = self.model(self.color_image, show=False, conf=CONF, device=self.device)
        cv2.imshow('Segment', results[0].plot())

        self.process_result(results)

        cv2.waitKey(1)

    def process_result(self, results):
        if results[0].masks is None:
            return

        contours = [result.xy for result in results[0].masks]
        # Height filter
        collected_contours = [self.height_filter(contour) for contour in contours]
        #print(collected_contour)
        #print('-----------------------------')
        non_empty_contours = [contour for contour in collected_contours if contour]

        if non_empty_contours:
            contours_np = [np.array(contour, dtype=np.int32).reshape((-1, 1, 2)) for contour in non_empty_contours]
            image_with_contours = self.color_image.copy()
            cv2.drawContours(image_with_contours, contours_np, -1, (0, 230, 0), 2)
            cv2.imshow('AfterFilter', image_with_contours)


def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    YOLO_Segment()
    rospy.spin()

if __name__ == "__main__":
    main()

    # start_time=time.time()
    # elasped_time=time.time()-start_time
    # print(1/elasped_time)
