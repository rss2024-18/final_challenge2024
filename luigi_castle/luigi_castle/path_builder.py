import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Point, Pose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from .utils import LineTrajectory, Map
import math

# LANES = [
#     (-20.116, 1.093), (-18.731, 7.383), (-15.034, 11.242),
#     (-13.643, 13.093), (-10.292, 16.786),
#     (-8.761, 18.183), (-5.468, 22.145),
#     (-5.107, 23.488), (-5.887, 25.306),
#     (-7.772, 25.727), (-19.329, 25.699), (-20.008, 26.060), (-20.329, 32.887), (-20.662, 33.840), (-27.514, 33.872),
#     (-29.520, 33.763), (-51.985, 33.969),
#     (-54.789, 33.969),
#     (-54.789, 32.614), (-54.665, 25.793),
#     (-54.665, 23.936), (-54.665, 1.118)
# ]

FORBIDDEN_ZONE = [
    (-19.716, -5.688), (-58.919, -4.180), (-59.832, 1.861), (-22.721, 1.801), (-20.010, 1.041)
]

LANES = [
    (-19.999, 1.336), (-18.434, 7.591), (-15.413, 10.617),
    (-6.186, 21.115), (-5.536, 25.662),
    (-19.717, 25.677), (-20.308, 26.207), (-20.442, 33.975),
    (-55.072, 34.078), (-55.301, 1.446)
]

LEFT_LANE = [(-18.058806264276193, 0.8505638375047564), (-16.493806264276195, 7.105563837504756), (-17.01861757008636, 6.177956272052507), (-13.997617570086357, 9.203956272052507), (-13.910775565278565, 9.296650899297516), (-4.683775565278564, 19.794650899297512), (-4.206127146562666, 20.831974410658837), (-3.5561271465626665, 25.378974410658838), (-5.5338844945196115, 27.661998881158826), (-19.71488449451961, 27.676998881158827), (-18.38171732934095, 27.16596614784811), (-18.972717329340952, 27.69596614784811), (-18.30829750498758, 26.241495382895426), (-18.44229750498758, 34.00949538289543), (-20.436051426831625, 35.97499115359975), (-55.06605142683163, 36.077991153599754), (-53.07204924561206, 34.06396504281825), (-53.301049245612056, 1.4319650428182509)]


CROSSWALKS = [

    [(-13.475281715393066, 10.334341049194336),
    (-15.432265281677246, 12.418288230895996),
    (-14.491107940673828, 13.802837371826172),
    (-12.456966400146484, 11.767603874206543)],

    [(-10.585527420043945, 18.065744400024414),
    (-9.522796630859375, 19.4090576171875),
    (-6.618139266967773, 17.08186912536621),
    (-8.206509590148926, 15.503101348876953)],

    [(-5.1608381271362305, 19.436222076416016),
    (-7.338576793670654, 26.61824607849121),
    (-4.740976333618164, 26.66748809814453),
    (-3.9178836345672607, 21.225791931152344)],

    [(-27.671483993530273, 32.62946701049805),
    (-30.002927780151367, 32.62947082519531),
    (-30.07505226135254, 35.14862823486328),
    (-28.076095581054688, 35.12565231323242)],

    [(-52.10889434814453, 32.72785949707031),
    (-56.51856231689453, 32.39398956298828),
    (-56.7427978515625, 35.872032165527344),
    (-52.02388000488281, 35.375770568847656)],

    [(-52.21940612792969, 23.420629501342773),
    (-56.712989807128906, 23.30794334411621),
    (-56.646087646484375, 25.40519142150879),
    (-52.24714660644531, 25.216827392578125)]

]

class PathPlan(Node):
    """Listens for goal pose published by RViz and uses it to plan a path from
    current car pose."""

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.polygons_pub = self.create_publisher(MarkerArray, "/polygons", 10)
        self.perpendicular_line_pub = self.create_publisher(Marker, "/perpendicular_line", 10)
        self.arrow_pub = self.create_publisher(
            Marker,
            "/driving_direction_arrow",
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.lane_pub = self.create_publisher(
            PoseArray,
            f"/lanes",
            10
        )
        self.lane_trajectory = LineTrajectory(node=self, viz_namespace=f"/planned_lane_trajectory")
        self.left_lane_traj = LineTrajectory(node=self, viz_namespace=f"/left_lane_trajectory")
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
        self.start = None
        self.end = None
        self.map = None

    def map_cb(self, msg):
        self.map = Map(msg, self, LANES, FORBIDDEN_ZONE)
        self.get_logger().info("Map published")
        self.publish_polygons()

    def pose_cb(self, pose):

        self.start = (pose.pose.pose.position.x, pose.pose.pose.position.y)
        self.get_logger().info("Start Pose: " + str(self.start))
        self.map.point_inside_forbidden(self.start[0], self.start[1])
        slope, intercept = self.map.perpendicular_line_through_point(self.start, LANES)
        perpendicular_marker = self.perpendicular_line_marker(slope, intercept)
        self.perpendicular_line_pub.publish(perpendicular_marker)

        # POTENTIAL GARBARGE

        orientation = self.map.perpendicular_orientation_at_point(self.start)
        
        # Create a Marker message for the arrow
        arrow_marker = Marker()
        
        # Set the frame ID
        arrow_marker.header.frame_id = "map"
        
        # Set the marker type to ARROW
        arrow_marker.type = Marker.ARROW
        
        # Set the marker action (ADD, MODIFY, DELETE)
        arrow_marker.action = Marker.ADD
        
        # Set the scale of the arrow (length, width, height)
        arrow_marker.scale.x = 1.0  # Length of the arrow
        arrow_marker.scale.y = 0.1  # Width of the arrow
        arrow_marker.scale.z = 0.1  # Height of the arrow
        
        # Set the color of the arrow (RGBA)
        arrow_marker.color.r = 1.0  # Red
        arrow_marker.color.g = 0.0  # Green
        arrow_marker.color.b = 0.0  # Blue
        arrow_marker.color.a = 1.0  # Alpha (transparency)
        
        # Set the pose of the arrow (position and orientation)
        arrow_marker.pose.position.x = self.start[0]  # X-coordinate of the arrow's position
        arrow_marker.pose.position.y = self.start[1]  # Y-coordinate of the arrow's position
        arrow_marker.pose.position.z = 0.0  # Z-coordinate of the arrow's position
        arrow_marker.pose.orientation.z = math.sin(orientation / 2)  # Z-component of the arrow's orientation quaternion
        arrow_marker.pose.orientation.w = math.cos(orientation / 2)  # W-component of the arrow's orientation quaternion
        
        # Publish the arrow Marker message
        self.arrow_pub.publish(arrow_marker)

    def goal_cb(self, msg):
        self.end = msg.pose.position
        self.get_logger().info("End Pose: " + str(self.end))
        if self.map is not None and self.start is not None:
            self.plan_path()

    def plan_path(self):
        
        path = self.map.bfs(self.start, self.end)
        self.trajectory.clear()
        if path is not None and len(path) > 0:
            for point in path:
                self.trajectory.addPoint(point)
        else:
            self.get_logger().info("Path not found")
            return

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

        # Publish lanes
        for point in LANES:
            self.get_logger().info("Lane point: " + str(point))
            self.lane_trajectory.addPoint(point)
        self.lane_pub.publish(self.lane_trajectory.toPoseArray())
        self.lane_trajectory.publish_viz()

    def publish_polygons(self):
        marker_array = MarkerArray()
        for i, polygon in enumerate(CROSSWALKS):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "crosswalks"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Adjust thickness if needed
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Adjust transparency if needed

            # Define the vertices of the polygon
            points = []
            for point_tuple in polygon:
                point = Point()
                point.x, point.y = point_tuple
                points.append(point)

            marker.points = points
            marker_array.markers.append(marker)

        self.polygons_pub.publish(marker_array)

    def perpendicular_line_marker(self, slope, intercept):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Define points for the perpendicular line
        point1 = Point()
        point1.x = self.start[0]  # x-coordinate is taken from self.start
        point1.y = self.start[1]  # y-coordinate is taken from self.start
        point1.z = 0.0
        marker.points.append(point1)
        
        point2 = Point()
        # Calculate y-coordinate of point2 using slope-intercept form of the perpendicular line equation
        point2.x = self.start[0] + 10  # Increment x-coordinate of start point to get a point on the line
        point2.y = slope * point2.x + intercept  # Calculate y-coordinate using slope-intercept form
        point2.z = 0.0
        marker.points.append(point2)
        
        return marker
    


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
