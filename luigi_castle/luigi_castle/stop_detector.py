import rclpy
from rclpy.node import Node
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
#from detector import StopSignDetector
#from . import detector 
from geometry_msgs.msg import PointStamped, Point
from vs_msgs.msg import ConeLocation, ConeLocationPixel
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

import time


class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        #self.detector = detector.StopSignDetector()
        #self.stop_sign_pub = self.create_publisher(Point, "/stop_sign")
        self.stop_light_pub = self.create_publisher(Bool, "/stop_light", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.test_callback, 5)
        self.drive_sub = self.create_subscription(Odometry, "/pf/pose/odom", self.odom_callback, 1)
        self.bridge = CvBridge()
        self.stoplight_region = False
        self.range = 2.0
        self.stoplight_1 = (-10.57, 16.17)
        self.stoplight_2 = (-31.64, 34.09)
        self.stoplight_3 = (-54.59, 24.50)


        self.max_light_size = 800
        self.ignore_threshold = 50
        self.x_pixels = 100
        self.buffer = 5

        self.debug_pub = self.create_publisher(Image, "/debug", 1)
        self.debug_pub1 = self.create_publisher(Image, "/debug1", 1)
        self.debug_pub2 = self.create_publisher(Image, "/debug2", 1)
        self.debug_pub3 = self.create_publisher(Image, "/debug3", 1)
        
        # timer_period = 2.0 #seconds
        # self.image = cv2.imread('/home/racecar/racecar_ws/src/final_challenge2024/media/download-7.jpg')
        # if self.image is None:
            # self.get_logger().info("IMAGE FAILED TO LOAD")
        # self.timer = self.create_timer(timer_period, self.callback)
        self.get_logger().info("Stop Detector Initialized")

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        dist_1 = math.sqrt((self.stoplight_1[0]-current_x)**2+(self.stoplight_1[1]-current_y)**2)
        dist_2 = math.sqrt((self.stoplight_2[0]-current_x)**2+(self.stoplight_2[1]-current_y)**2)
        dist_3 = math.sqrt((self.stoplight_3[0]-current_x)**2+(self.stoplight_3[1]-current_y)**2)

        if dist_1 < self.range or dist_2 < self.range or dist_3 < self.range:
            self.stoplight_region = True
        else:
            self.stoplight_region = False



    def test_callback(self, img_msg):
        # Process image with CV Bridgeself.get_logger().info("callback")
        # self.get_logger().info("callback0")
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        # image = self.image
        #image_print(image)

        #stop_image, stop_bb = StopSignDetector.predict(image)
        stop_image = False
        stop_bb = None
        # self.stoplight_region = True
        self.curr_time = None
        # self.get_logger().info(str(self.stoplight_region))
        if self.stoplight_region:
            # self.get_logger().info("In Range")
            redlight = self.stoplight_detector(image, stop_bb)
        else:
            redlight = False

        # self.get_logger().info("callback2")
        # light = ConeLocationPixel()
        light = Bool()
        if redlight:
            # light.u = float(redlight_c[0])
            # light.v = float(redlight_c[1])
            light.data = True
            # self.get_logger().info("redlight detected")
        else:
            light.data = False
        # self.get_logger().info("here")
        # self.get_logger().info(str(redlight))
        self.stop_light_pub.publish(light)

        # stop = Point()
        # if stop_image:
        #     x, y, w, h = stop_bb
        #     center_x = x + w // 2
        #     center_y = y + h // 2
        #     stop.x_pos = center_x
        #     stop.y_pos = center_y
        #     self.get_logger().info("stoplight detected")

        #self.stop_sign_pub.publish(stop)

 

    def stoplight_detector(self, image, stop_bb = None):
        # self.get_logger().info("here")
        image[:self.x_pixels+20, :] = 0  # Set all channels to 0 (black)
        image[360-self.x_pixels-40:, :] = 0  # Set all channels to 0 (black)
        image[:, 320:] = 0
        hsv_image = cv2.cvtColor(cv2.bitwise_not(image), cv2.COLOR_BGR2HSV)

        #threshold image
        # lower_red = (0, 20, 200)
        # upper_red = (15, 255, 255)
        lower_cyan = (85, 100, 200)
        upper_cyan = (95, 255, 255)

        mask = cv2.inRange(hsv_image, lower_cyan, upper_cyan)
        # result = cv2.bitwise_and(image, image, mask=mask)

        dilate_kernel = np.ones((9,9), np.uint8)
        open_image = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, dilate_kernel, iterations = 2)
        # image_print(dilated_image, "dilated_image")

        # Perform closing to merge nearby white regions
        closed_image = cv2.morphologyEx(open_image, cv2.MORPH_OPEN, np.ones((9,9)), iterations = 1)
        
        # image_print(closed_image, "closed_image")

        contours, _ = cv2.findContours(closed_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # biggest_x = 0.0
        # biggest_y = 0.0 
        # biggest = 0.0
        
        debug_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(open_image, cv2.COLOR_GRAY2BGR), "bgr8")
        debug_msg1 = self.bridge.cv2_to_imgmsg(cv2.cvtColor(closed_image, cv2.COLOR_GRAY2BGR), "bgr8")
        debug_msg2 = self.bridge.cv2_to_imgmsg(cv2.bitwise_not(image), "bgr8")
        debug_msg3 = self.bridge.cv2_to_imgmsg(cv2.bitwise_not(image), "bgr8")
        self.debug_pub.publish(debug_msg)
        self.debug_pub1.publish(debug_msg1)
        self.debug_pub2.publish(debug_msg2)
        self.debug_pub3.publish(debug_msg3)

        if len(contours) == 0:
            return False
        return max([cv2.contourArea(contour) for contour in contours]) > self.ignore_threshold
        

        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     self.get_logger().info(str(area))

        #     if area < self.max_light_size:
        #         # self.get_logger().info("area greater")
        #         #fit bounding rectangle
        #         x, y, w, h = cv2.boundingRect(contour)

        #         if stop_bb is not None:
        #             # self.get_logger().info("stop_bb")
        #             exclude_x, exclude_y, exclude_w, exclude_h = stop_bb
        #             exclude_x -= self.buffer
        #             exclude_y -= self.buffer
        #             exclude_w += 2 * self.buffer
        #             exclude_h += 2 * self.buffer
        #             # Check if the contour is within the excluded bounding box
        #             if (x >= exclude_x and y >= exclude_y and x + w <= exclude_x + exclude_w and y + h <= exclude_y + exclude_h):
        #                 continue 
                
        #         #check size of this red object
        #         if area > biggest:
        #             redlight = True
        #             center_x = x + w // 2
        #             center_y = y + h // 2  
        #             biggest = area
        #             biggest_x = center_x
        #             biggest_y = center_y
        # if biggest > 0.0:            
        #     image_with_rectangle = cv2.rectangle(image.copy(), (center_x-w//2, center_y+h//2), (center_x+w//2, center_y-h//2), (0, 255, 0), 2)
        #     # image_print(image_with_rectangle, "image_w_rectangle")
        #     return redlight, (biggest_x, biggest_y)
        # #nothing red found return none 
        # return redlight, (None, None)
    
def image_print(img, label):
    cv2.namedWindow(label, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(label, img)
    cv2.waitKey(1500)
    cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()