
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
#from detector import StopSignDetector
#from . import detector 
from geometry_msgs.msg import PointStamped, Point

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        #self.detector = detector.StopSignDetector()
        #self.stop_sign_pub = self.create_publisher(Point, "/stop_sign")
        self.stop_light_pub = self.create_publisher(Point, "/stop_light", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.min_light_size = 10
        self.light_rad = 10
        self.buffer = 5
        
        timer_period = 2.0 #seconds
        self.image = cv2.imread('/home/racecar/racecar_ws/src/final_challenge2024/media/lightOn.png')
        if self.image is None:
            self.get_logger().info("IMAGE FAILED")
        self.timer = self.create_timer(timer_period, self.callback)
        self.get_logger().info("Stop Detector Initialized")

    def callback(self):
        # Process image with CV Bridge
        #image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        #self.image = img_msg
        self.get_logger().info("callback")
        image = self.image
        #image_print(image)

        #stop_image, stop_bb = StopSignDetector.predict(image)
        stop_image = False
        stop_bb = None
        redlight, redlight_c = self.stoplight_detector(image, stop_bb)

        # light = Point()
        # if redlight:
        #     light.x_pos = redlight_c[1]
        #     light.y_pos = redlight_c[2]
        #     self.get_logger().info("redlight detected")

        # stop = Point()
        # if stop_image:
        #     x, y, w, h = stop_bb
        #     center_x = x + w // 2
        #     center_y = y + h // 2
        #     stop.x_pos = center_x
        #     stop.y_pos = center_y
        #     self.get_logger().info("stoplight detected")

        # self.stop_light_pub.publish(light)
        #self.stop_sign_pub.publish(stop)

 

    def stoplight_detector(self, image, stop_bb = None):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        redlight = False

        #threshold image (red is both at the very bottom of hue and very top)
        lower_red = (0, 90, 90)
        upper_red = (25, 255, 255)

        # lower_red_2 = (0, 0, 0)
        # upper_red_2 = (180, 255, 255)

        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        # mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        # mask = cv2.bitwise_or(mask1, mask2)

        erode_kernel = np.ones((3,3), np.uint8)
        eroded_image = cv2.erode(mask, erode_kernel, iterations=2)

        dilate_kernel = np.ones((5,5), np.uint8)
        dilated_image = cv2.dilate(eroded_image, dilate_kernel, iterations=2) 

        contours, _ = cv2.findContours(dilated_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        biggest_x = 0.0
        biggest_y = 0.0 
        biggest = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            self.get_logger().info(str(area))
            if area > self.min_light_size: 
                self.get_logger().info("area greater")
                #fit bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                # Fit a circle to the contour
                (xr, yr), radius = cv2.minEnclosingCircle(contour)
                center = (int(xr), int(yr))
                radius = int(radius)
                #check that it is not detecting a red light inside a stop sign
                if stop_bb is not None:
                    self.get_logger().info("stop_bb")
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
                    self.get_logger().info(str(radius))
                    redlight = True
                    image_with_circle = cv2.circle(image.copy(), center, radius, (0, 255, 0), 2)
                    image_print(image_with_circle)
                    return redlight, center
                
                #if no red cirle found return the largest red object
                if area > biggest:
                    redlight = True
                    center_x = x + w // 2
                    center_y = y + h // 2  

                    biggest = area
                    biggest_x = center_x
                    biggest_y = center_y
            image_with_rectangle = cv2.rectangle(image.copy(), (center_x-w//2, center_y+h//2), (center_x+w//2, center_y-h//2), (0, 255, 0), 2)
            image_print(image_with_rectangle)
            return redlight, (biggest_x, biggest_y)
           #nothing red found return none 
        return redlight, (None, None)

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()