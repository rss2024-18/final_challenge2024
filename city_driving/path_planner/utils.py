import rclpy

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Header
import os
from typing import List, Tuple
import json
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import math

EPSILON = 0.00000000001

''' These data structures can be used in the search function
'''


class LineTrajectory:
    """ A class to wrap and work with piecewise linear trajectories. """

    def __init__(self, node, viz_namespace=None):
        self.points: List[Tuple[float, float]] = []
        self.distances = []
        self.has_acceleration = False
        self.visualize = False
        self.viz_namespace = viz_namespace
        self.node = node

        if viz_namespace:
            self.visualize = True
            self.start_pub = self.node.create_publisher(Marker, viz_namespace + "/start_point", 1)
            self.traj_pub = self.node.create_publisher(Marker, viz_namespace + "/path", 1)
            self.end_pub = self.node.create_publisher(Marker, viz_namespace + "/end_pose", 1)

    # compute the distances along the path for all path segments beyond those already computed
    def update_distances(self):
        num_distances = len(self.distances)
        num_points = len(self.points)

        for i in range(num_distances, num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i - 1]
                p1 = self.points[i]
                delta = np.array([p0[0] - p1[0], p0[1] - p1[1]])
                self.distances.append(self.distances[i - 1] + np.linalg.norm(delta))

    def distance_to_end(self, t):
        if not len(self.points) == len(self.distances):
            print(
                "WARNING: Different number of distances and points, this should never happen! Expect incorrect results. See LineTrajectory class.")
        dat = self.distance_along_trajectory(t)
        if dat == None:
            return None
        else:
            return self.distances[-1] - dat

    def distance_along_trajectory(self, t):
        # compute distance along path
        # ensure path boundaries are respected
        if t < 0 or t > len(self.points) - 1.0:
            return None
        i = int(t)  # which segment
        t = t % 1.0  # how far along segment
        if t < EPSILON:
            return self.distances[i]
        else:
            return (1.0 - t) * self.distances[i] + t * self.distances[i + 1]

    def addPoint(self, point: Tuple[float, float]) -> None:
        #print("adding point to trajectory:", point)
        self.points.append(point)
        self.update_distances()
        self.mark_dirty()

    def clear(self):
        self.points = []
        self.distances = []
        self.mark_dirty()

    def empty(self):
        return len(self.points) == 0

    def save(self, path):
        print("Saving trajectory to:", path)
        data = {}
        data["points"] = []
        for p in self.points:
            data["points"].append({"x": p[0], "y": p[1]})
        with open(path, 'w') as outfile:
            json.dump(data, outfile)

    def mark_dirty(self):
        self.has_acceleration = False

    def dirty(self):
        return not self.has_acceleration

    def load(self, path):
        print("Loading trajectory:", path)

        # resolve all env variables in path
        path = os.path.expandvars(path)

        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                self.points.append((p["x"], p["y"]))
        self.update_distances()
        print("Loaded:", len(self.points), "points")
        self.mark_dirty()

    # build a trajectory class instance from a trajectory message
    def fromPoseArray(self, trajMsg):
        for p in trajMsg.poses:
            self.points.append((p.position.x, p.position.y))
        self.update_distances()
        self.mark_dirty()
        print("Loaded new trajectory with:", len(self.points), "points")

    def toPoseArray(self):
        traj = PoseArray()
        traj.header = self.make_header("/map")
        for i in range(len(self.points)):
            p = self.points[i]
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            traj.poses.append(pose)
        return traj

    def publish_start_point(self, duration=0.0, scale=0.1):
        should_publish = len(self.points) > 0
        self.node.get_logger().info("Before Publishing start point")
        if self.visualize and self.start_pub.get_subscription_count() > 0:
            self.node.get_logger().info("Publishing start point")
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 0
            marker.type = 2  # sphere
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[0][0]
                marker.pose.position.y = self.points[0][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.start_pub.publish(marker)
        elif self.start_pub.get_subscription_count() == 0:
            self.node.get_logger().info("Not publishing start point, no subscribers")

    def publish_end_point(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.end_pub.get_subscription_count() > 0:
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 1
            marker.type = 2  # sphere
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[-1][0]
                marker.pose.position.y = self.points[-1][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.end_pub.publish(marker)
        elif self.end_pub.get_subscription_count() == 0:
            print("Not publishing end point, no subscribers")

    def publish_trajectory(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.traj_pub.get_subscription_count() > 0:
            self.node.get_logger().info("Publishing trajectory")
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 2
            marker.type = marker.LINE_STRIP  # line strip
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = marker.ADD
                marker.scale.x = 0.3
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                for p in self.points:
                    pt = Point()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = 0.0
                    marker.points.append(pt)
            else:
                # delete
                marker.action = marker.DELETE
            self.traj_pub.publish(marker)
            print('publishing traj')
        elif self.traj_pub.get_subscription_count() == 0:
            print("Not publishing trajectory, no subscribers")

    def publish_viz(self, duration=0):
        if not self.visualize:
            print("Cannot visualize path, not initialized with visualization enabled")
            return
        self.publish_start_point(duration=duration)
        self.publish_trajectory(duration=duration)
        self.publish_end_point(duration=duration)

    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = self.node.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

class Map:
    """
    2D map discretization Abstract Data Type
    """
    def __init__(self, msg, node, lanes) -> None:
        
        self.node = node
        self.lanes = lanes
        self.height = msg.info.height
        self.width = msg.info.width
        self.resolution = msg.info.resolution
        orientation = msg.info.origin.orientation
        self.poseOrientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.angles = euler_from_quaternion(self.poseOrientation)
        self.posePoint = np.array([[msg.info.origin.position.x], [msg.info.origin.position.y], [msg.info.origin.position.z]])
        self.data = np.array(msg.data).reshape((self.height, self.width))

    def z_axis_rotation_matrix(self, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    def pixel_to_real(self, pixelCoord: Tuple[int, int]) -> Tuple[float, float]:
        pixelCoord = [i*self.resolution for i in pixelCoord]
        rotatedCoord = np.array([pixelCoord[0]], [pixelCoord[1]], [0.0]) * self.z_axis_rotation_matrix(self.angles[2])
        rotatedCoord = rotatedCoord + self.posePoint
        return (rotatedCoord[0, 0], rotatedCoord[1, 0])      

    def real_to_pixel(self, realCoord: Tuple[float, float]) -> Tuple[int, int]:

        x, y = realCoord
        
        xtrans = x-self.posePoint[0]
        ytrans = y-self.posePoint[1]
        theta = self.angles[2]

        # Apply inverse rotation using the negative of the yaw
        x_rotated = xtrans * math.cos(-theta) - ytrans * math.sin(-theta)
        y_rotated = xtrans * math.sin(-theta) + ytrans * math.cos(-theta)

        # Scale (x_rotated, y_rotated) by the inverse of the resolution to get grid coordinates
        u = int(abs(x_rotated) / self.resolution)
        v = int(y_rotated / self.resolution)

        return (u, v) 
    
    def distance_point_to_line_segment(self, point, line_segment):
        # Unpack points of the line segment
        p1, p2 = line_segment
        
        # Convert points to numpy arrays
        p1 = np.array(p1)
        p2 = np.array(p2)
        point = np.array(point)
        
        # Vector representing the line segment
        v = p2 - p1
        
        # Vector from p1 to the point
        w = point - p1
        
        # Projection of w onto v
        c1 = np.dot(w, v)
        c2 = np.dot(v, v)
        
        # Distance from the point to the line segment
        b = c1 / c2
        if b < 0:
            return np.linalg.norm(point - p1)
        elif b > 1:
            return np.linalg.norm(point - p2)
        else:
            pb = p1 + b * v
            return np.linalg.norm(point - pb)


    def closest_line_segment(self, point, line_segments):
        min_distance = float('inf')
        closest_segment = None

        # Iterate through each consecutive pair of points
        for i in range(len(line_segments) - 1):
            segment_start = line_segments[i]
            segment_end = line_segments[i + 1]
            
            # Form line segment from consecutive points
            segment = (segment_start, segment_end)
            
            # Calculate distance from point to line segment
            distance = self.distance_point_to_line_segment(point, segment)
            
            # Update closest segment if needed
            if distance < min_distance:
                min_distance = distance
                closest_segment = segment

        return (closest_segment, min_distance)

    def perpendicular_line_through_point(self, point, line_segments):
        closest_segment = self.closest_line_segment(point, line_segments)[0]

        # Calculate the equation of the perpendicular line
        p1, p2 = closest_segment
        x1, y1 = p1
        x2, y2 = p2

        # Calculate the slope of the closest lane segment
        if x2 - x1 == 0:  # Handling the case where the line segment is vertical
            slope_lane = float('inf')  # Assigning infinity as slope
        else:
            slope_lane = (y2 - y1) / (x2 - x1)

        # Calculate the perpendicular slope
        if slope_lane == 0:  # Handling the case where the lane segment is horizontal
            slope_perpendicular = float('inf')  # Assigning infinity as slope
        else:
            slope_perpendicular = -1 / slope_lane

        # Calculate the intercept of the perpendicular line with the lane segment
        intercept = point[1] - slope_perpendicular * point[0]

        return slope_perpendicular, intercept

    
    def point_to_segment_distance(self, point, segment_start, segment_end):
        x0, y0 = point
        x1, y1 = segment_start
        x2, y2 = segment_end
        dx = x2 - x1
        dy = y2 - y1
        if dx == dy == 0:  # Check if the segment is just a point
            return math.hypot(x1 - x0, y1 - y0)

        t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)

        if t < 0:
            closest_point = x1, y1
        elif t > 1:
            closest_point = x2, y2
        else:
            closest_point = x1 + t * dx, y1 + t * dy

        return math.hypot(x0 - closest_point[0], y0 - closest_point[1])

    def point_not_within_distance(self, path, point, distance_threshold):
        for i in range(len(path) - 1):
            segment_start = path[i]
            segment_end = path[i + 1]
            # for point in points:
            distance = self.point_to_segment_distance(point, segment_start, segment_end)
            if distance <= distance_threshold:
                return False  # If point is not within the threshold, return True
        return True
    

    def perpendicular_orientation_at_point(self, point):
        closest_segment = self.closest_line_segment(point, self.lanes)[0]

        # Calculate the closest point on the line segment to the given point
        p1, p2 = closest_segment
        x1, y1 = p1
        x2, y2 = p2

        # Calculate the vector from the given point to the closest point on the line segment
        dx = x2 - x1
        dy = y2 - y1
        vx = point[0] - x1
        vy = point[1] - y1

        # Calculate the length of the line segment
        segment_length = math.sqrt(dx * dx + dy * dy)

        # Project the point onto the line segment
        t = max(0, min((vx * dx + vy * dy) / (segment_length * segment_length), 1))
        closest_point_x = x1 + t * dx
        closest_point_y = y1 + t * dy

        # Calculate the orientation angle from the given point to the closest point on the line segment
        orientation = math.atan2(closest_point_y - point[1], closest_point_x - point[0]) - math.pi / 2

        return orientation


    def get_pixel(self, u, v):
        return self.data[v][u]
    
    def bfs(self, start_point, end_point):

        start_point = self.discretization(start_point[0], start_point[1])
        end_point = self.discretization(end_point.x, end_point.y)
        # start_point = self.discretization(11.094404220581055, -1.1191177368164062)
        # end_point = self.discretization(-16.592947006225586, 25.76806640625)

        visited = set(start_point) 
        queue = [(start_point, [start_point])]
        
        while queue:
            node, path = queue.pop(0)
            if node == end_point:
                return path   
                
            for neighbor in self.get_neighbors(node[0], node[1]):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

    def discretization(self, x, y):
        rounded_x = round(x * 2) / 2  # Round x to the nearest multiple of 0.5
        rounded_y = round(y * 2) / 2  # Round y to the nearest multiple of 0.5
        return (rounded_x, rounded_y)


    # def get_neighbors(self, x, y):
    #     neighbors = []
    #     directions = [(0, 0.5), (0.5, 0), (0, -0.5), (-0.5, 0), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]
    #     for dx, dy in directions:
    #         nx, ny = x + dx, y + dy
    #         u, v = self.real_to_pixel((nx, ny))
    #         if (-self.width <= u and u < self.width) and (-self.height <= v and v < self.height) and (self.get_pixel(u, v) == 0): # self.point_not_within_distance(self.lanes, (nx, ny), 0.7):
    #             neighbors.append((nx, ny))

    #     return neighbors

    def get_neighbors(self, x, y):
        neighbors = []
        directions = [(0, 0.5), (0.5, 0), (0, -0.5), (-0.5, 0), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]
        
        # Get the perpendicular orientation at the current point
        orientation = self.perpendicular_orientation_at_point((x, y))
        
        # Convert orientation to degrees for comparison
        orientation_degrees = math.degrees(orientation)
        
        # Define the acceptable orientation range (e.g., +/- 45 degrees from perpendicular)
        orientation_range = 45


        # if you are far enough from the lane, then you can consider all neighbors
        
        # this is to account for points that may be in corners/edges that will require 
        # the car to move in a direction that is not directly perpendicular to the lane

        min_distance = self.closest_line_segment((x, y), self.lanes)[1] 
        
        for dx, dy in directions:

            if min_distance < 1:
                # Calculate the orientation of the current direction
                direction_orientation = math.atan2(dy, dx)
                
                # Convert direction orientation to degrees for comparison
                direction_orientation_degrees = math.degrees(direction_orientation)
                
                # Check if the direction is within the acceptable range of orientations
                if abs(direction_orientation_degrees - orientation_degrees) <= orientation_range:
                    nx, ny = x + dx, y + dy
                    u, v = self.real_to_pixel((nx, ny))
                    if (-self.width <= u < self.width) and (-self.height <= v < self.height): # and (self.get_pixel(u, v) == 0):
                        neighbors.append((nx, ny))
            else:
                nx, ny = x + dx, y + dy
                u, v = self.real_to_pixel((nx, ny))
                if (-self.width <= u and u < self.width) and (-self.height <= v and v < self.height) and (self.get_pixel(u, v) == 0): # self.point_not_within_distance(self.lanes, (nx, ny), 0.7):
                    neighbors.append((nx, ny))

        return neighbors