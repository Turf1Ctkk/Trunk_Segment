#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import torch
import rospy
import numpy as np
import random
from ultralytics import YOLO
import time
from geometry_msgs.msg import Point32, PointStamped
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image
#from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes, Contour, ContourArray
from cv_bridge import CvBridge, CvBridgeError
#import message_filters

fx = 598.7568359375
fy = 598.7568969726562
cx = 326.3443298339844
cy = 250.24488830566406
BUFF_SIZE = 52428800
CONF = 0.35
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

        self.diameter_pub = rospy.Publisher('/diameter', Float32, queue_size=10)
        self.point_pub = rospy.Publisher('/nearest_point', PointStamped, queue_size=10)
        # If no image messages
        while not self.getImageStatus:
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)    
    
    def calculate_real_world_coordinates(self, x, y, depth):
        z = depth * 0.001
        x_real = (x - cx) * z / fx
        y_real = (y - cy) * z / fy
        return x_real, y_real, z
    
    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_image = self.depth_image.astype(np.float32)
            self.depth_image = cv2.medianBlur(self.depth_image, 5)
            self.depth_image[self.depth_image == 0] = np.nan
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
    
    def find_ymin_point(self, contour):
        ymin_value = float('inf')
        ymin_point = None

        for point in contour:
            x, y = int(point[0]), int(point[1])
            depth = self.depth_image[y, x]
            x_real, y_real, z_real = self.calculate_real_world_coordinates(x, y, depth)
            if y_real < ymin_value:
                ymin_value = y_real
                ymin_point = (x_real, y_real, z_real)

        return ymin_point


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
    
    def clamp_target_filter(self, contour, range):
        if self.depth_image is None or len(contour) == 0:
            rospy.logerr("Depth image is empty or contour is empty.")
            return [], None

        min_y_point = self.find_ymin_point(contour)
        if min_y_point is None:
            return [], None

        _, min_y_value, _ = min_y_point
        lower_edge = min_y_value + range * 0.001

        filtered_points = []
        for point in contour:
            x, y = int(point[0]), int(point[1])
            depth = self.depth_image[y, x]
            x_real, y_real, z_real = self.calculate_real_world_coordinates(x, y, depth)
            if min_y_value <= y_real <= lower_edge:
                filtered_points.append((x_real, y_real, z_real))
        
        return filtered_points, min_y_value

    def compute_and_publish(self, filtered_points, y_min):
        if not filtered_points:
            return

        x_values = [p[0] for p in filtered_points]
        z_values = [p[2] for p in filtered_points]

        x_max = max(x_values)
        x_min = min(x_values)
        diameter = x_max - x_min
        x_avg = sum(x_values) / len(x_values)
        z_min = min(z_values)

        nearest_point = (x_avg, y_min, z_min)
        rospy.loginfo(f"Nearest Point: {nearest_point}, Diameter: {diameter}")

        point_msg = PointStamped()
        point_msg.header = Header()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.point.x = nearest_point[0]
        point_msg.point.y = nearest_point[1]
        point_msg.point.z = nearest_point[2]
        self.point_pub.publish(point_msg)

        self.diameter_pub.publish(diameter)
    
    def average_distance(self, contour, sample_size=100):
        if self.depth_image is None or len(contour) == 0:
            rospy.logerr("Depth image is empty or contour is empty.")
            return 0

        sample_size = min(sample_size, len(contour))
        sampled_points = random.sample(contour, sample_size)
        
        distances = []
        for point in sampled_points:
            x, y = int(point[0][0]), int(point[0][1])
            depth = self.depth_image[y, x]

            if not np.isnan(depth):
                x_real, y_real, z_real = self.calculate_real_world_coordinates(x, y, depth)
                distance = np.sqrt(x_real**2 + y_real**2 + z_real**2)
                distances.append(distance)
        
        if distances:
            avg_distance = np.mean(distances)
            return avg_distance
        else:
            return 0
    
    def find_nearest_object(self, contours):
        min_distance = float('inf')
        nearest_contour = None

        for contour in contours:
            avg_distance = self.average_distance(contour)
            if avg_distance < min_distance:
                min_distance = avg_distance
                nearest_contour = contour

        return nearest_contour, min_distance

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
        nearest_contour, min_distance = self.find_nearest_object(contours)
        rospy.loginfo(f"Nearest object distance: {min_distance}")
        
        # Height filter
        filtered_contour = self.height_filter(nearest_contour)
        if filtered_contour:
            filtered_points, y_min = self.clamp_target_filter(filtered_contour, range=100)
            if filtered_points is not None and y_min is not None:
                self.compute_and_publish(filtered_points, y_min)

            image_with_contours = self.color_image.copy()
            
            cv2.drawContours(image_with_contours, np.array(filtered_contour, dtype=np.int32).reshape((-1, 1, 2)), -1, (0, 230, 0), 2)
            cv2.imshow('AfterFilter', image_with_contours)


        # Height filter
        #collected_contours = [self.height_filter(contour) for contour in contours]
        #print(collected_contour)
        #print('-----------------------------')
        #non_empty_contours = [contour for contour in collected_contours if contour]

        # if non_empty_contours:
        #     contours_np = [np.array(contour, dtype=np.int32).reshape((-1, 1, 2)) for contour in non_empty_contours]
        #     image_with_contours = self.color_image.copy()
        #     cv2.drawContours(image_with_contours, contours_np, -1, (0, 230, 0), 2)
        #     cv2.imshow('AfterFilter', image_with_contours)


def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    YOLO_Segment()
    rospy.spin()

if __name__ == "__main__":
    main()

    # start_time=time.time()
    # elasped_time=time.time()-start_time
    # print(1/elasped_time)
