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
        self.SPEED = self.get_parameter('speed').get_parameter_value().float_value
        self.WHEELBASE_LENGTH = self.get_parameter('wheelbase_length').get_parameter_value().float_value

        self.goal_sub = self.create_subscription(PointStamped,
                                                 self.goal_topic,
                                                 self.goal_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

    def goal_callback(self, msg):
        self.get_logger().info("goal received")
        
        # tentatively... pure pursuit?
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "/map"
        cmd.drive.speed = self.SPEED
        delta = np.arctan(2*self.WHEELBASE_LENGTH*msg.point.y / (msg.point.x**2 + msg.point.y**2))
        cmd.drive.steering_angle = delta

        self.drive_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    follower = GoalFollower()
    rclpy.spin(follower)
    rclpy.shutdown()