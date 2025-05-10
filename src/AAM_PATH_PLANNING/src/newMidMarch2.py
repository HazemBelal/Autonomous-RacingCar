#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import operator


class Cone:
    """
    Represents a cone detected by the camera.
    """

    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color


class PathPlanner:

    def __init__(self, namespace='rrt'):
        """
        Initialize the planner.

        Args:
            namespace (str): ROS namespace for parameters.
        """
        rospy.init_node("rrt_path_planner", anonymous=True)
        rospy.Subscriber("/camera_cones_marker", MarkerArray, self.handle_cones)
        self.should_publish_waypoints = rospy.get_param('~publish_waypoints', True)
        self.waypoints_visual_pub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

    def distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points.

        Args:
            point1 (tuple): Coordinates of the first point (x, y).
            point2 (tuple): Coordinates of the second point (x, y).

        Returns:
            float: Euclidean distance between the two points.
        """
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def handle_cones(self, cones_msg):
        """
        Handle detected cones from the camera.

        Args:
            cones_msg (MarkerArray): Message containing detected cones.
        """
        cones = []
        cones_yellow = []
        cones_blue = []
        cones_orange = []

        for cone_marker in cones_msg.markers:
            cone_x = cone_marker.pose.position.x
            cone_y = cone_marker.pose.position.y
            cone_color = ""

            if cone_marker.color.r == 0 and cone_marker.color.g == 0 and cone_marker.color.b == 200:
                cone_color = "blue"
                cones_blue.append((cone_x, cone_y))
            elif cone_marker.color.r == 200 and cone_marker.color.g == 200 and cone_marker.color.b == 0:
                cone_color = "yellow"
                cones_yellow.append((cone_x, cone_y))
            elif cone_marker.color.r == 200 and cone_marker.color.g == 0 and cone_marker.color.b == 0:
                cone_color = "orange"
                cones_orange.append((cone_x, cone_y))


            cone_object = Cone(cone_x, cone_y, cone_color)
            cones.append(cone_object)

        filtered_yellow_cones = []
        filtered_blue_cones = []
        if(len(cones_orange) > 1):
            cones_blue, cones_yellow = self.appendToTheRightSide(cones_blue, cones_yellow, cones_orange)
        if len(cones_yellow) >= 2:
            sorted_yellow_cones = self.sort_cones(cones_yellow)
            filtered_yellow_cones = self.filter_cones(sorted_yellow_cones)
        if len(cones_blue) >= 2:
            sorted_blue_cones = self.sort_cones(cones_blue)
            filtered_blue_cones = self.filter_cones(sorted_blue_cones)

        if (len(filtered_yellow_cones) >= 2 and len(filtered_blue_cones) >= 2):
            self.get_midpoint_simple(filtered_blue_cones,filtered_yellow_cones)

        elif len(filtered_blue_cones) >= 2:
            self.get_midpoint(filtered_blue_cones, (3 * math.pi) / 2)        
        
        elif len(filtered_yellow_cones) >= 2:
            self.get_midpoint(filtered_yellow_cones, (math.pi) / 2)

    import math

    def appendToTheRightSide(self, blue, yellow, orange):
        blue_cones = blue[:]
        yellow_cones = yellow[:]
        orange_cones = orange[:]
        
        # Lists to store results
        blue_nearest_oranges = blue[:]
        yellow_nearest_oranges = yellow[:]

        
        # Find nearest cones for each orange cone
        for orange_cone in orange_cones:
            min_blue_distance = float('inf')
            min_yellow_distance = float('inf')
            closest_blue_index = -1
            closest_yellow_index = -1
            
            # Find nearest blue cone
            for i, blue_cone in enumerate(blue_cones):
                dist = self.distance(orange_cone, blue_cone)
                if dist < min_blue_distance:
                    min_blue_distance = dist
                    closest_blue_index = i
            
            # Find nearest yellow cone
            for i, yellow_cone in enumerate(yellow_cones):
                dist = self.distance(orange_cone, yellow_cone)
                if dist < min_yellow_distance:
                    min_yellow_distance = dist
                    closest_yellow_index = i
            
            # Append orange cone to the appropriate list based on distances
            if min_yellow_distance < min_blue_distance:
                yellow_nearest_oranges.append(orange_cone)
            else:
                blue_nearest_oranges.append(orange_cone)
        
        return blue_nearest_oranges, yellow_nearest_oranges


    
    def filter_cones(self, cones):
        """
        Filter cones based on the maximum distance between them.

        Args:
            cones (list): List of cone coordinates.

        Returns:
            list: Filtered list of cone coordinates.
        """
        max_dist_between_cones = rospy.get_param('~max_distance_between_cones', 6)
        distances = [self.distance(cones[i], cones[i+1]) for i in range(len(cones) - 1)]
        filtered_cones = []

        for i in range(len(cones)):
            if i < len(distances) and distances[i] > max_dist_between_cones:
                break
            filtered_cones.append(cones[i])

        return filtered_cones

    def sort_cones(self, cone_list):
        """
        Sort cones based on their positions.

        Args:
            cone_list (list): List of cone coordinates.

        Returns:
            list: Sorted list of cone coordinates.
        """
        current_point = min(cone_list, key=lambda x: self.distance([0, 0], x))
        sorted_cones = [current_point]
        cone_list.remove(current_point)

        while cone_list:
            nearest_cone = min(cone_list, key=lambda x: self.distance(current_point, x))
            current_point = nearest_cone
            sorted_cones.append(current_point)
            cone_list.remove(current_point)

        return sorted_cones

    def get_midpoint(self, track, angle):
        """
        Calculate the midpoint of the track.

        Args:
            track (list): List of cone coordinates defining the track.
            angle (float): Angle of the track.

        Returns:
            tuple: Midpoint coordinates.
        """
        track_width = rospy.get_param('~track_width', 3.2)
        inner_side = []
        number_of_points = min(len(track),4)
        for i in range(number_of_points):
            if i == len(track) - 1:
                from_idx = len(track) - 2
                to_idx = len(track) - 1
            else:
                from_idx = i
                to_idx = i + 1

            V0 = (float(track[from_idx][0]), float(track[from_idx][1]))
            V1 = (float(track[to_idx][0]), float(track[to_idx][1]))
            V1_0 = tuple(map(operator.sub, V1, V0))
            norm = math.sqrt((V1_0[0] ** 2) + (V1_0[1] ** 2))
            vv = tuple(map(lambda x: x / norm, V1_0))
            wx = vv[0] * math.cos(angle) - vv[1] * math.sin(angle)
            wy = vv[0] * math.sin(angle) + vv[1] * math.cos(angle)
            wx = float(track[i][0]) + wx * track_width
            wy = float(track[i][1]) + wy * track_width
            inner_side.append((wx, wy))

        filtered_points = self.filter_cones(inner_side)
        self.publish_waypoints_visuals(filtered_points)

    def get_midpoint_simple(self, cones_blue, cones_yellow):
        """
        Calculate midpoints between corresponding cones of the same index.

        Args:
            cones_blue (list): List of blue cone coordinates.
            cones_yellow (list): List of yellow cone coordinates.

        Returns:
            list: List of midpoints.
        """
        midpoints = []
        the_midpoint = []

        min_length = min(len(cones_blue), len(cones_yellow))
        for i in range(min_length):
            midpoint_x = (cones_blue[i][0] + cones_yellow[i][0]) / 2
            midpoint_y = (cones_blue[i][1] + cones_yellow[i][1]) / 2
            midpoints.append((midpoint_x, midpoint_y))

        # if(len(cones_orange) > 2):
        #     the_sum_of_orange_x = 0
        #     the_sum_of_orange_y = 0
        #     for i in range(len(cones_orange)):
        #         the_sum_of_orange_x += cones_orange[i][0]
        #         the_sum_of_orange_y += cones_orange[i][1]
        #     midpoints.append(((the_sum_of_orange_x)/len(cones_orange), (the_sum_of_orange_y)/len(cones_orange)))


        self.publish_waypoints_visuals(midpoints)

    def publish_waypoints_visuals(self, new_waypoints=None):
        """
        Publish waypoints for visualization.

        Args:
            new_waypoints (list): List of new waypoint coordinates.
        """
        marker_array = MarkerArray()

        saved_waypoints_marker = Marker()
        saved_waypoints_marker.header.frame_id = "base_link"
        saved_waypoints_marker.header.stamp = rospy.Time.now()
        saved_waypoints_marker.lifetime = rospy.Duration(100)
        saved_waypoints_marker.ns = "saved-publishWaypointsVisuals"
        saved_waypoints_marker.id = 1

        saved_waypoints_marker.type = saved_waypoints_marker.SPHERE_LIST
        saved_waypoints_marker.action = saved_waypoints_marker.ADD
        saved_waypoints_marker.scale.x = rospy.get_param('~waypoint_scale', 0.3)
        saved_waypoints_marker.scale.y = rospy.get_param('~waypoint_scale', 0.3)
        saved_waypoints_marker.scale.z = rospy.get_param('~waypoint_scale', 0.3)
        saved_waypoints_marker.pose.orientation.w = 1

        saved_waypoints_marker.color.a = 1
        saved_waypoints_marker.color.b = 1

        for waypoint in new_waypoints:
            p = Point(waypoint[0], waypoint[1], 0.0)
            saved_waypoints_marker.points.append(p)

        marker_array.markers.append(saved_waypoints_marker)

        self.waypoints_visual_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        my_path = PathPlanner()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()



# edits of 20 june in 2024
# I have added an Orange cone handler by appending
# the orange cone detected to either blue or yellow cone
# depending in which side the orange cone deteceted we calculate the
# min distance between the oragne cone and rest of cones and choose the
# color depending on the closest one

