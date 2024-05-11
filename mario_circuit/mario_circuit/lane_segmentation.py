#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from .goal_from_image import cd_color_segmentation

class LaneSegmentation(Node):
    """
    A class for lane segmentation of the robot camera input.

    subscribes to: image_topic (Image) : the live RGB image from the onboard ZED camera
    publishes to: pixel_topic (Point) : the coordinates of the goal point in the image frame (units are pixels)
    """
    def __init__(self):
        super().__init__("lane_segmentation")
        self.declare_parameter('image_topic', "default")
        self.declare_parameter('debug_topic', "default")
        self.declare_parameter('pixel_topic', "default")
        self.declare_parameter('lookahead_pixel_height', "default")

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        self.pixel_topic = self.get_parameter('pixel_topic').get_parameter_value().string_value
        self.LOOKAHEAD_PIXEL_HEIGHT = self.get_parameter('lookahead_pixel_height').get_parameter_value().integer_value

        self.image_sub = self.create_subscription(Image,
                                                  self.image_topic,
                                                  self.image_callback,
                                                  5)
        self.pixel_pub = self.create_publisher(Point,
                                               self.pixel_topic,
                                               10)
        self.debug_pub = self.create_publisher(Image,
                                               self.debug_topic,
                                               10)
        
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.last_goal = None
        self.get_logger().info("Lane Segmentation Initialized")

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # self.get_logger().info(str(image.shape))

        goal_pixel = cd_color_segmentation(image, last=self.last_goal)
        if type(goal_pixel) is tuple:
            output_point = Point()
            output_point.x = goal_pixel[0] + 0.0
            output_point.y = goal_pixel[1] + 0.0
            self.last_goal = (goal_pixel[0], goal_pixel[1])
            self.pixel_pub.publish(output_point)
        
            # For debugging: visualize detection
            debug_image = goal_pixel[2].copy()
            cv2.circle(debug_image, (goal_pixel[0], goal_pixel[1]), 5, (0, 0, 255), -1)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
        
        else:
            debug_msg = self.bridge.cv2_to_imgmsg(goal_pixel, "bgr8")
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    lane_segmentation = LaneSegmentation()
    rclpy.spin(lane_segmentation)
    rclpy.shutdown()

if __name__ == '__main__':
    main()