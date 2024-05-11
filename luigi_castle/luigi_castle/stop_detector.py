
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
#from detector import StopSignDetector
#from . import detector 
from geometry_msgs.msg import PointStamped, Point
from vs_msgs.msg import ConeLocation, ConeLocationPixel

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        #self.detector = detector.StopSignDetector()
        #self.stop_sign_pub = self.create_publisher(Point, "/stop_sign")
        self.stop_light_pub = self.create_publisher(ConeLocationPixel, "/stop_light", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.max_light_size = 800
        self.x_pixels = 100
        self.buffer = 5
        
        timer_period = 2.0 #seconds
        self.image = cv2.imread('/home/racecar/racecar_ws/src/final_challenge2024/media/download-5.jpg')
        if self.image is None:
            self.get_logger().info("IMAGE FAILED TO LOAD")
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

        light = ConeLocationPixel()
        if redlight:
            light.u = float(redlight_c[0])
            light.v = float(redlight_c[1])
            self.get_logger().info("redlight detected")
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
        image[:self.x_pixels, :] = 0  # Set all channels to 0 (black)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        redlight = False

        #threshold image
        lower_red = (0, 100, 200)
        upper_red = (25, 255, 255)

        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        erode_kernel = np.ones((1,1), np.uint8)
        eroded_image = cv2.erode(mask, erode_kernel, iterations=1)

        dilate_kernel = np.ones((4,4), np.uint8)
        dilated_image = cv2.dilate(eroded_image, dilate_kernel, iterations=1) 
        image_print(dilated_image, "dilated_image")

        # Perform closing to merge nearby white regions
        closed_kernel = np.ones((6,6), np.uint8)
        closed_image = cv2.morphologyEx(dilated_image, cv2.MORPH_CLOSE, closed_kernel, iterations = 4)
        image_print(closed_image, "closed_image")

        contours, _ = cv2.findContours(closed_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        biggest_x = 0.0
        biggest_y = 0.0 
        biggest = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            self.get_logger().info(str(area))

            if area < self.max_light_size:
                self.get_logger().info("area greater")
                #fit bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)

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
                
                #check size of this red object
                if area > biggest:
                    redlight = True
                    center_x = x + w // 2
                    center_y = y + h // 2  
                    biggest = area
                    biggest_x = center_x
                    biggest_y = center_y
        if biggest > 0.0:            
            image_with_rectangle = cv2.rectangle(image.copy(), (center_x-w//2, center_y+h//2), (center_x+w//2, center_y-h//2), (0, 255, 0), 2)
            image_print(image_with_rectangle, "image_w_rectangle")
            return redlight, (biggest_x, biggest_y)
        #nothing red found return none 
        return redlight, (None, None)
    
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