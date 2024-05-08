import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from .utils import LineTrajectory, Map
from std_msgs.msg import Int32, Int32MultiArray, Header
from . import utils as util
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

import time

import numpy as np

class PathPlan(Node):
    """ Listens for 3 poses (published from Rviz by TAs clicking) and plots a path that passes each one without going backwards
    (aka it goes through the Luigi's Basement course)"""

    def __init__(self):
        super().__init__("path_builder") # TODO: change to new name
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.get_logger().info(str(self.map_topic))
        self.get_logger().info(str(self.initial_pose_topic))

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)
        self.stops_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.stop_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/planned_trajectory/path",
            10
        )

        self.drive_sub = self.create_subscription(AckermannDriveStamped, "/drive_filter", self.drive_cb, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.stop1_pub = self.create_publisher(
            Marker,
            "/stop1",
            10
        )
        self.stop2_pub = self.create_publisher(
            Marker,
            "/stop2",
            10
        )
        self.stop3_pub = self.create_publisher(
            Marker,
            "/stop3",
            10
        )

        self.index_sub = self.create_subscription(Int32, "/index", self.index_cb, 10)

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
        self.start = None
        self.stops = []
        self.reached = [False, False, False]
        self.index = 0
        self.map = None

        self.stop_time = None
    
    def odom_cb(self, msg):
        # check if coordinates have reached self.reached[self.index]
        #if they have reached the next stop, make it tell drive command to have zero velocity
        if self.map is not None and self.start is not None and len(self.stops)==3:
            robot_pose = msg.pose.pose 
            robot_point = self.map.discretization(robot_pose.position.x, robot_pose.position.y)
            stop_point = self.map.discretization(self.stops[self.index][0], self.stops[self.index][1])
            self.get_logger().info("ROBOT:  " + str(robot_point)) 
            self.get_logger().info("STOP " + str(self.index) + ":  " + str(stop_point)) 

            
            if robot_point[0]==stop_point[0] and robot_point[1]==stop_point[1]:
                self.get_logger().info("STOP REACHED") 
                self.index += 1
                self.stop_time = self.get_clock().now().to_msg()
        #pass

    def map_cb(self, msg):
        self.map = Map(msg, self)
        self.get_logger().info("MAP HELP")
        self.get_logger().info(str(self.start))
        
    def pose_cb(self, msg):
        self.get_logger().info("BITCH")
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info("INITIAL POSE")

    def index_cb(self, msg):
        self.index = msg.data
    
    def makeHeader(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "/map"
        return header
    
    def drive_cb(self, msg):
        # else, just publish whatever was sent to /drive_filter

        current_time = self.get_clock().now().to_msg()
        if self.stop_time!=None and current_time.sec <= self.stop_time.sec + 5:
            # make it publish zero-velocity drive commends if we haven't stopped for 5 seconds yet
            command = AckermannDriveStamped()
            command.header = self.makeHeader()
            command.header.stamp = self.get_clock().now().to_msg()
            command.drive.steering_angle = 0.0
            command.drive.speed = 0.0
            self.drive_pub.publish(command)
        else:
        #   # make it publish whatever came from /drive_filter
            self.drive_pub.publish(msg)

    def makeMarker(self, index, x, y):
        marker = Marker()

        header = self.makeHeader()
        marker.header = header

        marker.ns = "/stop"
        marker.id = index
        marker.type = 2  # sphere
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        
        marker.action = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 1.0, 1.0
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 0.0
        if index==1: marker.color.r = 1.0
        elif index==2: marker.color.g = 1.0
        elif index==3: marker.color.b = 1.0
        return marker
        
    def stop_cb(self, msg):
        self.get_logger().info("STOPS")
        if len(self.stops) < 3:
            pose = (msg.point.x, msg.point.y)
            self.stops.append(pose)
            self.get_logger().info(str(self.stops))

            # publish the newest stop
            index = len(self.stops)
            marker = self.makeMarker(index, msg.point.x, msg.point.y)
            if index==1:
                self.stop1_pub.publish(marker)
                self.get_logger().info("stop " + str(index) + "published")
            elif index==2:
                self.stop2_pub.publish(marker)
            if len(self.stops) == 3:
                self.stop3_pub.publish(marker)
                if self.map is not None and self.start is not None:
                    self.get_logger().info("path time BITCH")
                    path = self.plan_path(self.start, self.stops[0], self.stops[1], self.stops[2], self.map)
    

    def plan_path(self, start_point, stop_1, stop_2, stop_3, map):
        # construct a trajectory
        path0 = self.map.a_star(self.start, self.stops[0])
        print(str(path0))
        path1 = self.map.a_star(self.stops[0], self.stops[1])
        print(str(path1))
        path2 = self.map.a_star(self.stops[1], self.stops[2])
        print(str(path2))

        path = path0 + path1[1:] + path2[1:]
        
        
        # add to trajectory
        self.trajectory.clear()
        if path is not None and len(path) > 0:
            for point in path:
                self.trajectory.addPoint(point)
        else:
            self.get_logger().info("path is none?")
            return
        self.get_logger().info("YAY?")
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()