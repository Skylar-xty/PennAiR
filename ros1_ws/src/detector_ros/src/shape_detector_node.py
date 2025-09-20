#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS Node - Shape Detector
#
# This node subscribes to an image topic, runs the shape detection algorithm,
# and publishes the detection results and an annotated image.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detector_ros.msg import ShapeDetection, ShapeDetections

def classify_shape(contour):
    """
    Classifies a contour into a shape category.
    This function is copied from your original script.
    """
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return "unknown"
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    v = len(approx)
    area = cv2.contourArea(contour)
    if area < 200:
        return None  # Ignore very small contours
    
    # Calculate circularity
    circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
    
    if v == 3:
        return "triangle"
    elif v == 4:
        # Could be a square or rectangle
        return "quadrilateral"
    elif v == 5:
        return "pentagon"
    else:
        # Use circularity to distinguish circles from other polygons
        if circ > 0.75:
            return "circle"
        return "polygon"

def color_masks(hsv):
    """
    Creates color masks for detecting specific colors in HSV space.
    This function is adapted from your original script.
    """
    ranges = {
        "red1": ((0, 120, 70), (10, 255, 255)),
        "red2": ((170, 120, 70), (179, 255, 255)),
        "yellow": ((20, 120, 120), (35, 255, 255)),
        "blue": ((100, 120, 70), (130, 255, 255)),
        "magenta": ((140, 120, 70), (170, 255, 255)),
        "bright_green": ((45, 80, 161), (85, 255, 255)),
    }
    masks = {}
    for name, (lo, hi) in ranges.items():
        lo = np.array(lo, dtype=np.uint8)
        hi = np.array(hi, dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
        masks[name] = m

    # Merge red wrap-around hue range
    red = cv2.bitwise_or(masks["red1"], masks["red2"])
    masks_out = {"red": red, "yellow": masks["yellow"], "blue": masks["blue"],
                 "magenta": masks["magenta"], "bright_green": masks["bright_green"]}
    return masks_out

class ShapeDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('shape_detector_node', anonymous=True)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Publisher for detections
        self.detections_pub = rospy.Publisher('/shape_detections', ShapeDetections, queue_size=10)
        
        # Publisher for annotated image
        self.image_pub = rospy.Publisher('/camera/image_annotated', Image, queue_size=10)
        
        # Subscriber for raw image
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("Shape detector node is running...")

    def image_callback(self, ros_image):
        """
        Callback function for the image subscriber.
        Processes the incoming image to detect shapes.
        """
        try:
            # Convert ROS Image message to OpenCV image
            frame_bgr = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform detection
        annotated_frame, detections_list = self.detect_on_frame(frame_bgr)

        # Prepare and publish detections message
        detections_msg = ShapeDetections()
        detections_msg.header = ros_image.header  # Use the same header as the input image

        for shape, color, cx, cy, area in detections_list:
            detection = ShapeDetection()
            detection.shape = shape
            detection.color = color
            detection.cx = cx
            detection.cy = cy
            detection.area = area
            detections_msg.detections.append(detection)
        
        self.detections_pub.publish(detections_msg)

        # Publish the annotated image
        try:
            annotated_ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(annotated_ros_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_on_frame(self, frame_bgr):
        """
        Processes a single frame to find and classify colored shapes.
        This function is adapted from your original script.
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        masks = color_masks(hsv)

        annot = frame_bgr.copy()
        detections_result = []

        for color_label, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < 600:
                    continue
                
                shape_label = classify_shape(c)
                if shape_label is None:
                    continue
                
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                else:
                    (x, y), _ = cv2.minEnclosingCircle(c)
                    cx, cy = int(x), int(y)

                # Annotate the image
                cv2.drawContours(annot, [c], -1, (255, 255, 255), 3)
                cv2.circle(annot, (cx, cy), 6, (0, 0, 255), -1)
                text = f"{color_label} {shape_label}"
                cv2.putText(annot, text, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(annot, text, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

                detections_result.append((shape_label, color_label, cx, cy, area))
                
        return annot, detections_result

def main():
    try:
        ShapeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
