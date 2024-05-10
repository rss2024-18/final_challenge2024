import rclpy

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Header
import os
from typing import List, Tuple
import json
from tf_transformations import euler_from_quaternion
from collections import deque
import math
from numpy.linalg import inv
from scipy.ndimage import binary_dilation
from scipy.ndimage import binary_erosion
from scipy.spatial import distance

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
        print("adding point to trajectory:", point)
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
        #self.node.get_logger().info("Before Publishing start point")
        if self.visualize and self.start_pub.get_subscription_count() > 0:
            #self.node.get_logger().info("Publishing start point")
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
            #self.node.get_logger().info("Publishing trajectory")
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


class Map():
    """
    2D map discretization Abstract Data Type
    """
    def __init__(self, msg, node, lanes, forbidden, left, right, u_turns, left_zone, right_zone) -> None:
        self.node = node
        self.height = msg.info.height #1300
        self.width = msg.info.width #1730
        self.resolution = msg.info.resolution
        self.orientation = msg.info.origin.orientation
        poseOrientation = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.angles = euler_from_quaternion(poseOrientation)
        self.posePoint = np.array([[msg.info.origin.position.x], [msg.info.origin.position.y], [msg.info.origin.position.z]])
        self.map_translation = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.data = np.reshape(np.array(msg.data), (self.height, self.width))
        self.raw_data = np.array(msg.data).shape
        self.transformation_matrix = None
        self.dilated = self.erode_map(self.dilate_map(self.data))
        self.lanes = lanes
        self.forbidden = forbidden
        self.left_traj = left
        self.right_traj = right
        self.u_turns = u_turns
        self.left_zone = left_zone
        self.right_zone = right_zone

    def dilate_map(self, data):
        struct_element = np.ones((11, 11))
        dilated_map = binary_dilation(data, structure=struct_element).astype(np.uint8)
        return dilated_map
    
    def erode_map(self, data):
        struct_element = np.ones((11, 11))
        eroded_map = binary_erosion(data, structure=struct_element).astype(np.uint8)
        return eroded_map

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
    
    def zone_identifier(self, x, y):
        in_right = self.point_inside_forbidden(x, y, self.right_zone)
        in_left = self.point_inside_forbidden(x, y, self.left_zone)
        self.node.get_logger().info("In right is: " + str(in_right) + "and in left is: " + str(in_left))
        return (in_left, in_right)

    def closest_point_to_lanes(self, start, finish):

        start = (start[0], start[1])
        finish = (finish.x, finish.y)

        start_zone = self.zone_identifier(start[0], start[1])
        finish_zone = self.zone_identifier(finish[0], finish[1])

        if start_zone[0]:
            shortest_start = self.closest_line_segment(start, self.left_traj)
            index1 = self.left_traj.index(shortest_start[0][1])
        else:
            shortest_start = self.closest_line_segment(start, self.right_traj)
            index1 = self.right_traj.index(shortest_start[0][1])

        if finish_zone[0]:
            shortest_finish = self.closest_line_segment(finish, self.left_traj)
            index2 = self.left_traj.index(shortest_finish[0][0])
        else:
            shortest_finish = self.closest_line_segment(finish, self.right_traj)
            index2 = self.right_traj.index(shortest_finish[0][0])
        
        if start_zone[0] and finish_zone[0]:
            return [start] + self.left_traj[index1:index2 + 1] + [finish]
        elif start_zone[0] and finish_zone[1]:
            return [start] + self.left_traj[index1:] + self.u_turns[0] + self.right_traj[:index2] + [finish]
        elif start_zone[1] and finish_zone[0]:
            return [start] + self.right_traj[index1:] + self.u_turns[1] + self.left_traj[:index2] + [finish]
        else:
            return [start] + self.right_traj[index1:index2 + 1] + [finish]
        
        # shortest_left_start = self.closest_line_segment(start, self.left_traj)
        # shortest_right_start = self.closest_line_segment(start, self.right_traj)
        # shortest_left_finish = self.closest_line_segment(finish, self.left_traj)
        # shortest_right_finish = self.closest_line_segment(start, self.right_traj)

        # if shortest_left_start[1] < shortest_right_start[1]:
        #     index1 = self.left_traj.index(shortest_left_start[0][1])
        #     if shortest_left_finish[1] < shortest_right_finish[1]:
        #         index2 = self.left_traj.index(shortest_left_finish[0][0])
        #         return [start] + self.left_traj[index1:index2 + 1] + [finish]
        #     else:
        #         index2 = self.right_traj.index(shortest_right_finish[0][0])
        #         return [start] + self.left_traj[index1:] + self.u_turns[0] + self.right_traj[:index2] + [finish]
        # else:
        #     index1 = self.right_traj.index(shortest_right_start[0][1])
        #     if shortest_left_finish[1] < shortest_right_finish[1]:
        #         index2 = self.left_traj.index(shortest_left_finish[0][0])
        #         return [start] + self.right_traj[index1:] + self.u_turns[1] + self.left_traj[:index2] + [finish]
        #     else:
        #         index2 = self.right_traj.index(shortest_right_finish[0][0])
        #         return [start] + self.right_traj[index1:index2 + 1] + [finish]
            
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
        # self.node.get_logger().info(str(math.degrees(orientation)))
        return orientation


    def get_pixel(self, u, v):
        return self.dilated[v][u]
    
    def clean_path(self, path, threshold=0.1):
        cleaned_path = [path[0]]  # Start with the first point
        for i in range(1, len(path)):
            # Compute distance between consecutive points
            dist = distance.euclidean(path[i], path[i-1])
            # If distance is greater than threshold, add the point to cleaned path
            if dist > threshold:
                cleaned_path.append(path[i])
        return np.array(cleaned_path)
    
    def bfs(self, start_point, end_point):

        start_point = self.discretization(start_point[0], start_point[1])
        end_point = self.discretization(end_point.x, end_point.y)

        visited = set(start_point) 
        queue = [(start_point, [start_point])]
        
        while queue:
            node, path = queue.pop(0)
            if node == end_point:
                return self.clean_path(path)  
                
            for neighbor in self.get_neighbors(node[0], node[1]):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

    def point_inside_forbidden(self, x, y, polygon):
        # Function to determine if a point (x, y) lies inside forbidden zone
        # self.forbidden is a list of (x, y) points defining the vertices of the forbidden zone
        
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y
        # self.node.get_logger().info(str(inside))
        return inside

    def discretization(self, x, y):
        rounded_x = round(x * 4) / 4  # Round x to the nearest multiple of 0.25
        rounded_y = round(y * 4) / 4  # Round y to the nearest multiple of 0.25
        return (rounded_x, rounded_y)
    
    def within_angle_range(self, alpha, beta, target):
        # Calculate the absolute difference between alpha and beta
        difference = abs(alpha - beta)
        
        # If the absolute difference is greater than 180 degrees, subtract 360 degrees
        while difference > 360:
            difference -= 360
        
        # Take the absolute value of the result
        difference = abs(difference)
        
        # Check if the absolute difference is less than or equal to the target
        return difference <= target

    def get_neighbors(self, x, y):
        neighbors = []
        directions = [(0, 0.25, -90), (0.25, 0, 0), (0, -0.25, 90), (-0.25, 0, -180), (-0.25, 0.25, -45), (0.25, 0.25, -135), (0.25, -0.25, 135), (-0.25, -0.25, 45)]
        
        # Get the perpendicular orientation at the current point
        orientation = self.perpendicular_orientation_at_point((x, y))
        
        # Convert orientation to degrees for comparison
        orientation_degrees = math.degrees(orientation)

        # if you are far enough from the lane, then you can consider all neighbors

        # this is to account for points that may be in corners/edges that will require 
        # the car to move in a direction that is not directly perpendicular to the lane

        min_distance = self.closest_line_segment((x, y), self.lanes)[1] 
        
        for dx, dy, theta in directions:

            nx, ny = x + dx, y + dy
            u, v = self.real_to_pixel((nx, ny))

            if (-self.width <= u and u < self.width) and (-self.height <= v and v < self.height) and (self.get_pixel(u, v) == 0) and not self.point_inside_forbidden(nx, ny):

                if min_distance < 0.25:
                    range = 20
                    reference = orientation_degrees + 90
                else:
                    range = 100
                    reference = orientation_degrees

                if self.within_angle_range(reference, theta, range):
                    neighbors.append((nx, ny))

        return neighbors
