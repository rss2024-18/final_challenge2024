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

U_ZONES = [
    [
        (-13.163928031921387, 18.158140182495117),
        (-7.987878799438477, 14.049873352050781),
        (-16.72947883605957, 1.21099853515625),
        (-16.30608367919922, -4.633331298828125),
        (-23.06682014465332, -5.082618713378906),
        (-23.728614807128906, 2.0563011169433594),
        (-23.742555618286133, 3.503938674926758),
        (-20.70946502685547, 13.605573654174805) 
    ],
    [
        (-9.039815902709961, 22.558256149291992),
        (-22.06470489501953, 23.15163803100586),
        (-23.178926467895508, 31.732242584228516), # edge
        (-22.879558563232422, 37.28616714477539), # edge
        (-16.84946060180664, 36.3623046875),
        (-16.676712036132812, 28.495525360107422),
        (1.4966697692871094, 28.641599655151367),
        (0.4956321716308594, 19.883113861083984),
        (-7.987878799438477, 14.049873352050781),
        (-13.163928031921387, 18.158140182495117)
    ],
    [
        (-26.334346771240234, 31.171545028686523),
        (-46.38508605957031, 29.910722732543945), # edge
        (-45.877777099609375, 37.048973083496094), # edge
        (-25.655637741088867, 36.96967315673828),
        (-22.879558563232422, 37.28616714477539),
        (-23.178926467895508, 31.732242584228516)
    ],
    [
        (-49.91654586791992, 31.122419357299805),
        (-51.55299377441406, 0.6833267211914062),
        (-58.33567810058594, 1.7894706726074219),
        (-60.50545120239258, 15.915384292602539),
        (-60.017112731933594, 40.50865173339844),
        (-49.50034713745117, 38.92525100708008),
        (-45.877777099609375, 37.048973083496094),
        (-46.38508605957031, 29.910722732543945)
    ]
]

FORBIDDEN_ZONE = [
    (-19.716, -5.688), (-58.919, -4.180), (-59.832, 1.861), (-22.721, 1.801), (-20.010, 1.041)
]

LANES = [
    (-19.999, 1.336), (-18.434, 7.591), (-15.413, 10.617),
    (-6.186, 21.115), (-5.536, 25.662),
    (-19.717, 25.677), (-20.308, 26.207), (-20.442, 33.975),
    (-55.072, 34.078), (-55.301, 1.446)
]

RIGHT_ZONE = [
    (-23.132278442382812, 2.665557861328125),
    (-21.89523696899414, 10.86410903930664),
    (-18.261850357055664, 12.106391906738281),
    (-14.600807189941406, 15.751995086669922),
    (-16.781803131103516, 21.14411163330078),
    (-16.635761260986328, 23.107376098632812),
    (-22.210615158081055, 23.36255645751953),
    (-23.319927215576172, 30.498191833496094),
    (-50.028167724609375, 28.978065490722656),
    (-51.207271575927734, 1.0568389892578125),
    (-55.301, 1.446),
    (-55.072, 34.078),
    (-20.442, 33.975),
    (-20.308, 26.207),
    (-19.717, 25.677),
    (-5.536, 25.662),
    (-6.186, 21.115),
    (-15.413, 10.617),
    (-18.434, 7.591),
    (-19.999, 1.336)
]

LEFT_ZONE = [
    (-59.04590606689453, 2.483642578125),
    (-59.46685028076172, 15.604923248291016),
    (-59.266361236572266, 27.183212280273438),
    (-60.56454849243164, 36.41203308105469),
    (-54.90285110473633, 39.670562744140625),
    (-48.01030349731445, 37.004798889160156),
    (-17.67970848083496, 36.46495056152344),
    (-16.8974609375, 28.116024017333984),
    (1.3782501220703125, 28.176055908203125),
    (1.0861568450927734, 19.802021026611328),
    (-0.84906005859375, 9.894290924072266),
    (-16.652690887451172, 1.2210006713867188),
    (-19.999, 1.336), (-18.434, 7.591), (-15.413, 10.617),
    (-6.186, 21.115), (-5.536, 25.662),
    (-19.717, 25.677), (-20.308, 26.207), (-20.442, 33.975),
    (-55.072, 34.078), (-55.301, 1.446)

]

U_TURNS = [
    # green to blue
    [
        [
            (-55.88079833984375, 3.1826858520507812),
            (-54.80546569824219, 2.4565563201904297),
            (-53.99618911743164, 2.8528800010681152)
        ],
        [
            (-9.138984680175781, 17.062294006347656),
            (-9.205830574035645, 17.685787200927734),
            (-10.066749572753906, 17.600753784179688)
        ],
        [
            (-22.389629364013672, 33.33021545410156),
            (-21.7149600982666, 33.93490219116211),
            (-22.074859619140625, 34.35870361328125)
        ],
        [
            (-45.14533615112305, 33.45314407348633),
            (-44.48274612426758, 33.87034225463867),
            (-45.089473724365234, 34.29652786254883)
        ]


    ],
    # blue to green
    [ 
        [
            (-20.514204025268555, 2.4343719482421875),
            (-19.93574333190918, 1.544703483581543),
            (-18.993911743164062, 1.9550728797912598)
        ],
        [
            (-11.030861854553223, 16.344257354736328),
            (-10.943384170532227, 15.602993965148926),
            (-10.152714729309082, 16.009159088134766)
        ],
        [
            (-24.0711727142334, 34.35116195678711),
            (-24.6066837310791, 33.96518325805664),
            (-24.564105987548828, 33.55561065673828)        
        ],
        [
            (-46.725677490234375, 34.407379150390625),
            (-47.262733459472656, 33.9921875),
            (-46.94284439086914, 33.546104431152344)
        ]

    ]
]

LEFT_LANE = [
    (-18.725404739379883, 2.8445847034454346),
    (-17.968454, 7.2379),
    (-17.143102645874023, 8.28244686126709),
    (-16.040660858154297, 9.267585754394531),
    (-14.784688949584961, 10.594837188720703),
    (-13.2698335647583, 12.193135261535645),
    (-12.01342487335205, 13.762849807739258),
    (-10.480502128601074, 15.313055038452148),
    (-9.579690933227539, 16.292278289794922), # start here after U 1
    (-8.740646362304688, 17.41200065612793),
    (-7.117569923400879, 19.318763732910156),
    (-5.863956451416016, 20.785799026489258),
    (-4.527, 22.495),
    (-4.190, 24.587),
    (-5.011040687561035, 25.96796417236328),
    (-5.896455764770508, 26.16507911682129),
    (-13.876848220825195, 26.170467376708984),
    (-19.422794342041016, 26.210678100585938),
    (-19.84954833984375, 27.100994110107422),
    (-19.89433479309082, 32.19474792480469),
    (-19.809860229492188, 34.312171936035156),
    (-21.228172302246094, 34.70301818847656),
    (-23.459335327148438, 34.68435287475586), # u2
    (-28.433284759521484, 34.62849044799805),
    (-36.69542694091797, 34.54281997680664),
    (-45.558265686035156, 34.66493225097656),
    (-45.99287414550781, 34.51799011230469), # u3
    (-52.546539306640625, 34.56803894042969),
    (-55.442771911621094, 34.62120056152344),
    (-56.026, 34.113),
    (-56.090423583984375, 33.3248176574707),
    (-56.0438117980957, 29.925796508789062),
    (-56.078731536865234, 25.91656494140625),
    (-56.197444915771484, 20.145841598510742),
    (-56.054866790771484, 13.907846450805664),
    (-55.9172248840332, 6.858772277832031),
    (-56.057708740234375, 4.128641128540039)
]

RIGHT_LANE = [
    (-53.72200393676758, 3.751293182373047),
    (-53.76587677001953, 5.811088562011719),
    (-54.495880126953125, 8.26742935180664),
    (-54.45588302612305, 12.665342330932617),
    (-53.75273132324219, 14.638647079467773),
    (-53.53679656982422, 23.6800537109375),
    (-53.939884185791016, 32.602569580078125),
    (-53.134090423583984, 33.437870025634766),
    (-46.48407745361328, 33.277679443359375), # u3
    (-44.95112228393555, 33.263309478759766),
    (-38.18071365356445, 33.220916748046875),
    (-29.851320266723633, 33.355003356933594),
    (-24.666767120361328, 33.16566467285156),
    (-23.23598861694336, 33.248374938964844), # u2
    (-21.372081756591797, 33.260597229003906),
    (-20.972854614257812, 32.60691833496094),
    (-21.000377655029297, 30.20135498046875),
    (-20.72274398803711, 27.31175422668457),
    (-20.791677474975586, 25.409427642822266),
    (-20.105955123901367, 24.965473175048828),
    (-14.576354026794434, 24.91936683654785),
    (-11.062684059143066, 25.049264907836914),
    (-7.02130126953125, 24.721200942993164),
    (-6.559017181396484, 23.091920852661133),
    (-6.533161163330078, 21.560195922851562),
    (-7.6263108253479, 20.48574447631836),
    (-9.707321166992188, 18.23645782470703),
    (-10.457362174987793, 17.257844924926758), # right before U turn 1
    (-13.273554801940918, 14.368036270141602),
    (-15.09896469116211, 11.506546020507812),
    (-16.131643295288086, 10.521530151367188),
    (-16.621583938598633, 9.45796012878418),
    (-17.910009384155273, 8.8543701171875),
    (-19.187942504882812, 7.742585182189941),
    (-20.6666316986084, 3.1752207279205322)
]

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
        self.right_lane_traj = LineTrajectory(node=self, viz_namespace=f"/right_lane_trajectory")

        self.uturn1_trajectory = LineTrajectory(node=self, viz_namespace="/uturn1_trajectory")
        self.uturn2_trajectory = LineTrajectory(node=self, viz_namespace="/uturn2_trajectory")
        self.uturn3_trajectory = LineTrajectory(node=self, viz_namespace="/uturn3_trajectory")
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
        self.map = Map(msg, self, LANES, FORBIDDEN_ZONE, LEFT_LANE, RIGHT_LANE, U_TURNS, LEFT_ZONE, RIGHT_ZONE, U_ZONES)
        self.get_logger().info("Map published")
        self.publish_polygons()
        self.publish_uturn_trajectories()

        # Publish lanes
        for point in LANES:
            # self.get_logger().info("Lane point: " + str(point))
            self.lane_trajectory.addPoint(point)
        self.lane_pub.publish(self.lane_trajectory.toPoseArray())
        self.lane_trajectory.publish_viz()

        for point in LEFT_LANE:
            # self.get_logger().info("Lane point: " + str(point))
            self.left_lane_traj.addPoint(point)
        self.lane_pub.publish(self.left_lane_traj.toPoseArray())
        self.left_lane_traj.publish_viz()

        for point in RIGHT_LANE:
            # self.get_logger().info("Lane point: " + str(point))
            self.right_lane_traj.addPoint(point)
        self.lane_pub.publish(self.right_lane_traj.toPoseArray())
        self.right_lane_traj.publish_viz()

    def pose_cb(self, pose):

        self.start = (pose.pose.pose.position.x, pose.pose.pose.position.y)

        with open("right.txt", "a") as file:
            file.write(f"{self.start}\n")

        self.map.zone_identifier(self.start[0], self.start[1])
        self.get_logger().info("Start Pose: " + str(self.start))
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
        
        path = self.map.closest_point_to_lanes(self.start, self.end)
        self.trajectory.clear()
        if path is not None and len(path) > 0:
            for point in path:
                self.trajectory.addPoint(point)
        else:
            self.get_logger().info("Path not found")
            return

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()


    def publish_uturn_trajectories(self):
        # Generate and publish trajectory for U-turn 1
        self.generate_and_publish_uturn_trajectory(U_TURNS[1][3], self.uturn1_trajectory)

        # Generate and publish trajectory for U-turn 2
        # self.generate_and_publish_uturn_trajectory(U_TURNS[1][1], self.uturn2_trajectory)

        # Generate and publish trajectory for U-turn 3
        # self.generate_and_publish_uturn_trajectory(U_TURNS[0][0], self.uturn3_trajectory)

    def generate_and_publish_uturn_trajectory(self, uturn_points, trajectory_object):
        # Clear existing points from trajectory object
        trajectory_object.clear()

        # Add points to the trajectory object
        for point in uturn_points:
            trajectory_object.addPoint(point)

        # Publish the trajectory
        trajectory_object.publish_viz()


    def publish_polygons(self):
        marker_array = MarkerArray()
        for i, polygon in enumerate(U_ZONES):
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
            point = Point()
            point.x, point.y = polygon[0]
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
