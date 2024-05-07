import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory, Map
from std_msgs.msg import Int32, Int32MultiArray

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

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.get_logger().info(str(self.map_topic))
        self.get_logger().info(str(self.initial_pose_topic))

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

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.index_sub = self.create_subscription(Int32, "/index", self.index_cb, 10)
        #self.index_pub = self.create_publisher(Int32MultiArray, "/stops", 10)
        self.index_pub = self.create_publisher(Int32, "/num_stops", 10)

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
        self.start = None
        self.stops = []
        self.map = None
    
    def map_cb(self, msg):
        self.map = Map(msg, self)
        self.get_logger().info("MAP HELP")
        
    def pose_cb(self, msg):
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info("INITIAL POSE")

    def index_cb(self, msg):
        self.index = msg.data

    def stop_cb(self, msg):
        if len(self.stops) < 3:
            pose = (msg.point.x, msg.point.y)
            self.stops.append(pose)
            # if len(self.stops) == 3:
            #     stops_msg = Int32MultiArray()
            #     stops_msg.data = self.stops
            #     self.index_pub.publish(stops_msg)
            numStops = Int32()
            #self.index_pub.publish(len(self.stops))
            numStops.data = len(self.stops)
            self.index_pub.publish(numStops)
            self.get_logger().info(str(self.stops))
        else:
            self.get_logger().info(str(self.map))
            self.get_logger().info(str(self.start))
            if self.map is not None and self.start is not None:
                self.get_logger().info("path time BITCH")
                # stops_msg = Int32MultiArray()
                # stops_msg.data = self.stops
                # self.index_pub.publish(stops_msg)
                path = self.plan_path(self.start, self.stops[0], self.stops[1], self.stops[2], self.map)
                self.get_logger().info(str(self.trajectory.points))
            

        pass
        #self.end = (msg.pose.position.x, msg.pose.position.y)
        # self.get_logger().info("End Pose: " + str(self.end))
        #if self.map is not None and self.start is not None:
            #path = self.plan_path(self.start, self.end, self.map)
    

    def plan_path(self, start_point, stop_1, stop_2, stop_3, map):
        # construct a trajectory
        self.get_logger().info("PATH?")
        path0 = self.map.a_star(self.start, self.stops[0])
        self.get_logger().info(str(path0))
        path1 = self.map.a_star(self.stops[0], self.stops[1])
        self.get_logger().info(str(path1))
        path2 = self.map.a_star(self.stops[1], self.stops[2])
        self.get_logger().info(str(path2))

        # TODO: how make stop at stops?
        path = path0 + path1 + path2
        
        
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