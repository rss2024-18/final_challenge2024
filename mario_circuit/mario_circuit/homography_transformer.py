#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [ [209, 260],
                    [514, 281],
                    [426, 224],
                    [288, 214]
                   #[214, 230],
                   #[412, 210],
                   #[369, 190],
                   #[422, 228]
                    ] # dummy points published to /zed/rgb/image_rect_color_mouse_left
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [ [29, 11],
                     [23.5, -11],
                     [47, -12],
                     [56, 7]
                    #[45, 18],
                    #[82, -19],
                    #[121, -15.7],
                    #[42, -9]
                    ] # dummy points measured
######################################################

METERS_PER_INCH = 0.0254
     
class HomographyTransformer(Node):
    """ Homography transformer for converting pixels in image frame to points in car frame.
    
        subscribes to: pixel_topic (Point) : the coordinates of the goal point in the image frame (units are pixels)
        publishes to: goal_topic (PointStamped) : relative position of goal point to car (+x forward, +y left)
    """
    def __init__(self):
        super().__init__("homography_transformer")
        self.declare_parameter('pixel_topic', "default")
        self.declare_parameter('marker_topic', "default")
        self.declare_parameter('goal_topic', "default")

        self.pixel_topic = self.get_parameter('pixel_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        self.pixel_sub = self.create_subscription(Point,
                                                  self.pixel_topic,
                                                  self.goal_pixel_callback, 
                                                  1)
        self.marker_pub = self.create_publisher(Marker,
                                               self.marker_topic,
                                               1)
        self.goal_pub = self.create_publisher(PointStamped,
                                              self.goal_topic,
                                              10)

        self.model = PinholeCameraModel()

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")

    
    def goal_pixel_callback(self, msg):
        #Extract information from message
        u = msg.x
        v = msg.y

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = PointStamped()
        relative_xy_msg.header.stamp = self.get_clock().now().to_msg()
        relative_xy_msg.header.frame_id = "/map"
        relative_xy_msg.point.x = x
        relative_xy_msg.point.y = y

        # self.get_logger().info(str(relative_xy_msg.x_pos))
        # self.get_logger().info(str(relative_xy_msg.y_pos))

        self.goal_pub.publish(relative_xy_msg)


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)
    

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()