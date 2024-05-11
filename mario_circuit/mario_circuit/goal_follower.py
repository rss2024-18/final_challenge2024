import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

import numpy as np

class GoalFollower(Node):
    """ A controller that drives and follows a goal position.
    
        subscribes to: goal_topic (PointStamped) : relative position of goal point to car (+x forward, +y left)
        publishes to: drive_topic (AckermannDriveStamped) : drive command to the car
    """

    def __init__(self):
        super().__init__("goal_follower")
        self.declare_parameter('goal_topic', "default")
        self.declare_parameter('drive_topic', "default")
        self.declare_parameter('speed', "default")
        self.declare_parameter('wheelbase_length', "default")

        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SPEED = self.get_parameter('speed').get_parameter_value().double_value
        self.SPEED = 4.0
        self.WHEELBASE_LENGTH = self.get_parameter('wheelbase_length').get_parameter_value().double_value

        self.timer = self.create_timer(1 / 20, self.timer_callback)
        self.goal_sub = self.create_subscription(PointStamped,
                                                 self.goal_topic,
                                                 self.goal_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        self.drive_cmd = AckermannDriveStamped()
        self.get_logger().info("Goal follower initialized")

        self.last_delta = 0.0
        self.alpha = 0.8

        self.last_error = 0.0
        self.kp = 1.5 # 0.4 # 0.5
        self.kd = 3.0 # 0.25 # 0.3

    def timer_callback(self):
        # self.get_logger().info(str(self.drive_cmd))
        self.drive_pub.publish(self.drive_cmd)


    def goal_callback(self, msg):
        # self.get_logger().info("goal received")
        
        # tentatively... pure pursuit?
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "/map"
        cmd.drive.speed = self.SPEED
        # delta = np.arctan(2*self.WHEELBASE_LENGTH*msg.point.y / (msg.point.x**2 + msg.point.y**2)) * 1.2
        # delta = self.alpha * self.last_delta + (1-self.alpha) * delta
        # self.last_delta = delta
        error = np.arctan(msg.point.y / msg.point.x)
        delta = self.kp * error + self.kd * (error - self.last_error)
        delta = np.clip(delta, -np.pi/100, np.pi/100)
        self.last_error = error
        if (np.abs(msg.point.y) > 0.4):
            delta = 0.0
        cmd.drive.steering_angle = delta

        self.drive_cmd = cmd


def main(args=None):
    rclpy.init(args=args)
    follower = GoalFollower()
    rclpy.spin(follower)
    rclpy.shutdown()