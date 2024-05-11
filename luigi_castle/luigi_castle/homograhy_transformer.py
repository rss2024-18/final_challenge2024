#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from vs_msgs.msg import ConeLocation, ConeLocationPixel
from geometry_msgs.msg import PointStamped, Point
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

# TODO: define Node that publishes to /relative_cone_px
class ImageToCV(Node):
    def __init__(self):
        super().__init__("image_to_cv")

        pass
        self.ros_sub = self.create_subscription(Image, "/zed/zed_node/rgb_raw/image_raw_color", self.ros_to_cv, 10)
        #self.cv_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px") # TODO: get rid of this
        self.bridge = CvBridge()
    
    def ros_to_cv(self, msg):
        self.get_logger().info("TIME: " + str(msg.header.stamp))
        # TODO: convert msg (ROS Image) to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        

class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.sign_pub = self.create_publisher(ConeLocation, "/relative_cone", 10)
        self.marker_pub = self.create_publisher(Marker, "/cone_marker", 1)
        self.cone_px_sub = self.create_subscription(ConeLocationPixel, "/relative_cone_px", self.cone_detection_callback, 1)

        #
        #self.ros_sub = self.create_subscription(Image, "/zed/zed_node/rgb_raw/image_raw_color", self.ros_to_cv, 10)
        #self.click_sub = self.create_subscription(PointStamped, "/clicked_point", self.click_callback, 10)
        self.click_pub = self.create_subscription(Point, "/zed/rgb/image_rect_color_mouse_left", self.mouse_callback, 10)
        self.actual_light = self.create_publisher(Point, "/actual_light")
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

    
    def cone_detection_callback(self, msg):
        #Extract information from message
        u = msg.u
        v = msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        light_location = Point()
        light_location.x = x
        light_location.y = y

        self.get_logger().info(str(relative_xy_msg.x_pos))
        self.get_logger().info(str(relative_xy_msg.y_pos))

        self.actual_light.publish(light_location)


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
    
    def click_callback(self, msg):
        # publish the points you click to the map
        # 0) turn clicked-on points to pixels
        u, v = self.model.project3dToPixel((msg.point.x, msg.point.y, msg.point.z))
        # 1) convert the pixel locations you click on to xy coordinates w homography matrix
        self.get_logger().info("(" + msg.point.x + ", " + msg.point.y + "," + msg.point.z + ")")
        self.get_logger().info(u + "," + v)
        
        ans = ConeLocationPixel()
        ans.u, ans.v = u, v

        # 2) publish the xy coordinates with marker
        pass

    def mouse_callback(self, msg):
        # for debugging pixel -> location
        u, v = msg.x, msg.y
        loc = self.transformUvToXy(u, v)
        self.get_logger().info(str(loc[0]))
        self.get_logger().info(str(loc[1]))
        pass

    

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()