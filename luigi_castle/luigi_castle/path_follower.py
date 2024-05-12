import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from .utils import LineTrajectory
from .detector import StopSignDetector

import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
import math
import cv2
from cv_bridge import CvBridge

import time

class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("path_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_filter_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.drive_filter_topic = self.get_parameter('drive_filter_topic').get_parameter_value().string_value

        self.lookahead = 1.2  # FILL IN #
        self.speed = 2.0 # FILL IN #
        self.wheelbase_length = 0.3  # FILL IN #

        self.trajectory = LineTrajectory("/planned_trajectory")

        self.odom_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/planned_trajectory/path",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        self.drive_filter_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_filter_topic,
                                               1)
        self.debug_pub = self.create_publisher(PointStamped,
                                               "/debug/target",
                                               1)

        self.initialized_traj = False

        self.angle_pub = self.create_publisher(Float32, "/steering_angle", 1)
        self.dist_pub = self.create_publisher(Float32, "/perp_dist", 1)

        # subscribe to path stops
        # self.stop1_sub = self.create_subscription(Marker, "/stop1", self.receive_stop, 1)

        # self.stops = [None, None, None, None]
        # self.last_stop = 3

        # self.stopping = False
        # self.stop_start = None

        self.stop_now = False
        self.stoplight_sub = self.create_subscription(Bool,
                                                      "/stop_light",
                                                      self.stoplight_cb,
                                                      10)

        self.active_braking = False
        self.active_braking_time = None

        # self.ssd = StopSignDetector()
        # self.image_sub = self.create_subscription(Image,
        #                                           "/zed/zed_node/rgb/image_rect_color",
        #                                           self.image_cb,
        #                                           10)
        # self.bridge = CvBridge()
        # self.spotted_hold = False
        # self.stop_threshold = 10000
        self.get_logger().info("Path follower initialized")
        
    # def image_cb(self, image_msg):
    #     if self.spotted_hold:
    #         return
    #     image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
    #     image = np.asarray(image[:,:])
    #     detected, bounding_box = self.ssd.predict(image)
    #     if detected:
    #         self.get_logger().info(str((bounding_box[2]-bounding_box[0]) * (bounding_box[3]-bounding_box[1])))
    #         if ((bounding_box[2]-bounding_box[0]) * (bounding_box[3]-bounding_box[1]) > self.stop_threshold):
    #             self.active_braking = True
    #             self.active_braking_time = time.time()
    #             self.spotted_hold = True


    def stoplight_cb(self, bool_msg):
        # self.get_logger().info("Bool Message: " + str(bool_msg.data))
        self.stop_now = bool_msg.data

    # def receive_stop(self, marker_msg):
    #     # Marker(index, x, y)
    #     # stops array is 1-indexed
    #     self.stops[marker_msg.id] = (marker_msg.pose.position.x, marker_msg.pose.position.y)

    def pose_callback(self, odometry_msg):
        if self.active_braking and time.time() - self.active_braking_time < 1.0:
            self.get_logger().info("ACTIVE BRAKING")
            return
        else:
            self.active_braking = False
        # if self.spotted_hold and time.time() - self.active_braking_time > 5.0:
        #     self.spotted_hold = False
        if self.stop_now:
            self.get_logger().info("LIGHT")
            command = AckermannDriveStamped()
            command.header = odometry_msg.header
            command.header.stamp = self.get_clock().now().to_msg()
            command.drive.steering_angle = 0.0
            command.drive.speed = 0.0
            self.drive_filter_pub.publish(command)
            self.active_braking = True
            self.active_braking_time = time.time()
            return
        # curr_time = self.get_clock().now().to_msg()
        # if self.stop_sign_stop:
        #     if curr_time.sec <= self.time_last_stop_sign.sec + 1:
        #         command = AckermannDriveStamped()
        #         command.header = odometry_msg.header
        #         command.header.stamp = self.get_clock().now().to_msg()
        #         command.drive.steering_angle = 0.0
        #         command.drive.speed = 0.0
        #         self.drive_filter_pub.publish(command)
        #         return
        #     else:
        #         self.stop_sign_stop = False
        # early return if stopped
        # if self.stopping:
        #     self.get_logger().info(str(odometry_msg.header.stamp - self.stop_start))
        #     if (odometry_msg.header.stamp - self.stop_start < 5.0):
        #         return
        #     self.stopping = False
        
        # self.get_logger().info("B: " + str(self.initialized_traj))
        if not self.initialized_traj: 
            return
        
        robot_pose = odometry_msg.pose.pose 
        robot_point = np.array([robot_pose.position.x, robot_pose.position.y])
        trajectory_points = np.array(self.trajectory.points) 

        ## distances to line segments, vectorized 
        ## https://stackoverflow.com/a/58781995
        starts = trajectory_points[:-1]
        ends = trajectory_points[1:]
        vectors = ends - starts
        normalized = np.divide(vectors, (np.hypot(vectors[:,0], vectors[:,1]).reshape(-1,1)))
        parallel_start = np.multiply(starts-robot_point, normalized).sum(axis=1)
        parallel_end = np.multiply(robot_point-ends, normalized).sum(axis=1)
        clamped = np.maximum.reduce([parallel_start, parallel_end, np.zeros(len(parallel_start))])
        start_vectors = robot_point - starts
        perpendicular = start_vectors[:,0] * normalized[:,1] - start_vectors[:,1] * normalized[:,0]
        distances = np.hypot(clamped, perpendicular)

        closest_segment_index = np.argmin(distances)
        # print(str(closest_segment_index))
        
        ## search for lookahead point
        ## https://codereview.stackexchange.com/a/86428
        found = False
        target = None
        for i in range(closest_segment_index, len(starts)):
            P1 = starts[i]
            V = vectors[i]
            a = np.dot(V, V)
            b = 2 * np.dot(V, P1 - robot_point)
            c = np.dot(P1, P1) + np.dot(robot_point, robot_point) - 2*np.dot(P1, robot_point) - self.lookahead**2
            disc = b**2 - 4*a*c
            if disc < 0:
                continue
            sqrt_disc = np.sqrt(disc)
            t1 = (-b + sqrt_disc) / (2*a)
            t2 = (-b - sqrt_disc) / (2*a)
            # self.get_logger().info(str(t1) + " " + str(t2))
            if not (0 <= t1 <= 1): ## or 0 <= t2 <= 1
                continue
            found = True
            target = P1 + t1 * V
            # self.get_logger().info(str(np.linalg.norm(ends[i] - robot_point)))
            self.lookahead = np.sqrt(np.linalg.norm(ends[i] - robot_point)) + 0.2 
            break
        
        # self.get_logger().info("C: " + str(found))
        if not found:
            command = AckermannDriveStamped()
            command.header = odometry_msg.header
            command.header.stamp = self.get_clock().now().to_msg()
            command.drive.steering_angle = 0.0
            command.drive.speed = 0.0
            self.drive_filter_pub.publish(command)
            raise Exception("can't find target point")
        
        # visualize target
        pose = PointStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "/map"
        pose.point.x, pose.point.y = target[0], target[1]
        self.debug_pub.publish(pose)

        # ## check if close enough to stop point
        # if self.last_stop != 3:
        #     dist_to_next_stop = np.linalg.norm(self.stops[self.last_stop+1] - robot_point)
        #     if dist_to_next_stop < 0.3:
        #         command = AckermannDriveStamped()
        #         command.header = odometry_msg.header
        #         command.header.stamp = self.get_clock().now().to_msg()
        #         command.drive.steering_angle = 0.0
        #         command.drive.speed = 0.0
        #         self.drive_filter_pub.publish(command)

        #         self.last_stop += 1
        #         ## hold for 5 seconds!
        #         self.stopping = True
        #         self.stop_start = odometry_msg.header.stamp



        ## get pure pursuit control action
        orientation = robot_pose.orientation
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = r.as_euler('xyz', degrees=False)
        angle = euler[2] - np.arctan2(target[1]-robot_point[1], target[0]-robot_point[0])
        l1 = np.linalg.norm(target - robot_point)
        delta = np.arctan(2*self.wheelbase_length*np.sin(angle) / l1)

        # publish delta, the steering angle
        ang = Float32()
        ang.data = delta
        self.angle_pub.publish(ang)

        # publish the distance between target point and car
        dist = Float32()
        dist.data = math.sqrt((target[1]-robot_point[1])**2 + (target[0]-robot_point[0])**2)
        

        # publish perpendicular distance to nearest trajectory thing
        perp = Float32()
        perp.data = perpendicular[closest_segment_index]
        self.dist_pub.publish(perp)
        
        
        # send command
        # self.get_logger().info("D: " + str((self.speed, delta)))
        command = AckermannDriveStamped()
        command.header = odometry_msg.header
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.steering_angle = -delta
        command.drive.speed = self.speed
        self.drive_filter_pub.publish(command)


    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()