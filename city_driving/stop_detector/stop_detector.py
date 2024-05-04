import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector
from geometry_msgs.msg import PointStamped, Point

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        self.stop_sign_pub = self.create_publisher(Point, "/stop_sign")
        self.stop_light_pub = self.create_publisher(Point, "/stop_light")
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.min_light_size = 1000
        self.light_rad = 10
        self.buffer = 5

        self.get_logger().info("Stop Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        stop_image, stop_bb = StopSignDetector.predict(image)
        redlight, redlight_c = self.stoplight_detector(image, stop_bb)

        light = Point()
        if redlight:
            light.x_pos = redlight_c[1]
            light.y_pos = redlight_c[2]

        stop = Point()
        if stop_image:
            x, y, w, h = stop_bb
            center_x = x + w // 2
            center_y = y + h // 2
            stop.x_pos = center_x
            stop.y_pos = center_y

        self.stop_light_pub.publish(light)
        self.stop_sign_pub.publish(stop)


        
    

    def stoplight_detector(self, image, stop_bb = None):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        redlight = False

        #threshold image (red is both at the very bottom of hue and very top)
        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)

        lower_red_2 = (170, 100, 100)
        upper_red_2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        biggest_x = 0.0
        biggest_y = 0.0 
        biggest = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_light_size: 
                #fit bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                # Fit a circle to the contour
                (xr, yr), radius = cv2.minEnclosingCircle(contour)
                center = (int(xr), int(yr))
                radius = int(radius)
                #check that it is not detecting a red light inside a stop sign
                if stop_bb is not None:
                    exclude_x, exclude_y, exclude_w, exclude_h = stop_bb
                    exclude_x -= self.buffer
                    exclude_y -= self.buffer
                    exclude_w += 2 * self.buffer
                    exclude_h += 2 * self.buffer
                    # Check if the contour is within the excluded bounding box
                    if (x >= exclude_x and y >= exclude_y and x + w <= exclude_x + exclude_w and y + h <= exclude_y + exclude_h):
                        continue 
                #if red circle has been found that is the traffic light
                if radius > self.light_rad: 
                    redlight = True
                    return redlight, center
                
                #if no red cirle found return the largest red object
                if area > biggest:
                    redlight = True
                    center_x = x + w // 2
                    center_y = y + h // 2  

                    biggest = area
                    biggest_x = center_x
                    biggest_y = center_y
            return redlight, (biggest_x, biggest_y)
           #nothing red found return none 
        return redlight, (None, None)


def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()