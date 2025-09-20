#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS Node - Streams video file as sensor_msgs/Image messages
#
# This node reads a video file and publishes each frame to a ROS topic.
# It's the first part of the pipeline, providing the image data for other nodes.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

def video_streamer():
    """
    Streams a video file as ROS Image messages.
    """
    # Initialize the ROS node
    rospy.init_node('video_streamer', anonymous=True)

    # --- Parameters ---
    # Get the video file path from the ROS Parameter Server
    video_path = rospy.get_param('~video_path', '')
    if not video_path:
        rospy.logerr("Video path is not set. Please provide the 'video_path' parameter.")
        rospy.logerr("Example: _video_path:=/path/to/your/video.mp4")
        return

    # Get the publishing rate (frames per second)
    # If set to 0, it will publish as fast as the video is read.
    rate_hz = rospy.get_param('~rate', 30) # Default to 30 Hz

    # Create an image publisher
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # Open video capture
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        rospy.logerr(f"Error opening video file: {video_path}")
        return

    rospy.loginfo(f"Publishing video from: {video_path} at ~{rate_hz} Hz")

    # Set the loop rate
    if rate_hz > 0:
        rate = rospy.Rate(rate_hz)

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("End of video file reached. Looping...")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Loop the video
                continue

            try:
                # Convert the OpenCV image to a ROS Image message and publish
                image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_pub.publish(image_message)
            except CvBridgeError as e:
                rospy.logerr(e)

            # Sleep to maintain the desired publishing rate
            if rate_hz > 0:
                rate.sleep()

    finally:
        # Release the video capture object on shutdown
        cap.release()
        rospy.loginfo("Video streamer node shut down.")

if __name__ == '__main__':
    try:
        video_streamer()
    except rospy.ROSInterruptException:
        pass
