#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
import random

class Cone:
    def __init__(self, x, y, c):
        self.x = x
        self.y = y
        self.c = c

class Node:
    def __init__(self, x, y, yaw=0, cost=0.0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw #Orientation of the car at the node
        self.cost = cost #Cost to reach this node from the start node.
        self.parent = parent #parent node, linking it to the tree

class Planner:
    def __init__(self, namespace='rrt'):
        rospy.init_node("rrt_path_planner", anonymous=True)

        # Subscribe to the camera for cone detection
        rospy.Subscriber("/camera_cones_marker", MarkerArray, self.cones_pipeline)

        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)

        # Publish waypoints to be visualized in rviz and for the controller to extend
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        # Initialize cone lists as attributes of the class
        self.cones_blue = []
        self.cones_yellow = []

        # distance for terminating RRT   
        self.planDistance = 10 #MAY NEED TO INCREASE

    # Callback function for cones detection
    def cones_pipeline(self, cones_msg):
        self.cones_blue = []
        self.cones_yellow = []

        for c in cones_msg.markers:
            cone_x = c.pose.position.x
            cone_y = c.pose.position.y

            if c.color.r == 0 and c.color.g == 0 and c.color.b == 200:
                self.cones_blue.append((cone_x, cone_y))
            elif c.color.r == 200 and c.color.g == 200 and c.color.b == 0:
                self.cones_yellow.append((cone_x, cone_y))
        
        self.cones_blue.sort(key=lambda cone: (cone[0], cone[1]))
        self.cones_yellow.sort(key=lambda cone: (cone[0], cone[1]))

        self.rrt1()

    def rrt1(self):
        max_iterations = 3000
        step_size = 1.5 
        start_node = Node(0, 0, 0)  
        self.nodes = [start_node]  

        for i in range(max_iterations):
            #rand_point = self.sample_random_point() 
            rand_point = self.sample_random_point(self.cones_blue, self.cones_yellow)
            nearest_node = self.nearest_node(self.nodes, rand_point) #Find the nearest existing node in the tree to the sampled point
            new_node = self.steerConstrained(nearest_node, rand_point, step_size) #Create a (new_node) by steering toward the sampled point

            if not self.check_collision_extend(nearest_node, new_node): #Check for collisions. If a collision exists, skip this iteration
                continue
            
            #Find nearby nodes and update the parent of the new node if it improves the cost
            near_nodes = [node for node in self.nodes if self.distance((node.x, node.y), (new_node.x, new_node.y)) < 5.0]
            new_node = self.choose_parent(new_node, near_nodes) 
            self.nodes.append(new_node) #Add the new node to the tree
            self.rewire_tree(self.nodes, new_node) #rewire nearby nodes for cost efficiency

            if new_node.cost >= self.planDistance: #if the cost of the new node exceeds planDistance
                path = self.extract_path(new_node) #extract the path and publish it
                self.publishWaypointsVisuals(path)
                return path

    def sample_random_point(self, cones_blue, cones_yellow):
        max_distance = 3  # Maximum distance for random sampling

        while True:
            #sample within track bounds in front of the car
            x_rand = random.uniform(1, max_distance)  
            y_rand = random.uniform(-2.5, 2.5)      

        #sampled point is at least 1 meter away from all cones
            if all(self.distance((x_rand, y_rand), cone) >= 1.0 for cone in cones_blue + cones_yellow):
                return (x_rand, y_rand)


    def nearest_node(self, nodes, point):
        return min(nodes, key=lambda n: self.distance((n.x, n.y), point))

    def steerConstrained(self, from_node, to_point, step_size):
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x) #angle btw current pos and sampled point rad
        angle_change = self.pi_2_pi(theta - from_node.yaw) # difference between target angle(theta)where car wants to move & (yaw) where car is now facing

        max_turn = math.radians(45) #increased max turn angle from 30 to 45 
        angle_change = max(-max_turn, min(max_turn, angle_change))

        new_yaw = from_node.yaw + angle_change
        new_x = from_node.x + step_size * math.cos(new_yaw)
        new_y = from_node.y + step_size * math.sin(new_yaw)

        return Node(new_x, new_y, new_yaw, from_node.cost + step_size, from_node)

    def check_collision_extend(self, from_node, to_node, step_size=0.1):
        steps = int(self.distance((from_node.x, from_node.y), (to_node.x, to_node.y)) / step_size)
        for i in range(steps):
            x = from_node.x + i * step_size * math.cos(from_node.yaw)
            y = from_node.y + i * step_size * math.sin(from_node.yaw)
            if any(self.distance((x, y), cone) < 1 for cone in self.cones_blue + self.cones_yellow):
                return False
        return True

    def rewire_tree(self, nodes, new_node):
        for node in nodes:
            if node != new_node and self.distance((node.x, node.y), (new_node.x, new_node.y)) < 2.0:
                new_cost = new_node.cost + self.distance((node.x, node.y), (new_node.x, new_node.y))
                if new_cost < node.cost and self.check_collision_extend(new_node, node):
                    node.parent = new_node
                    node.cost = new_cost

    def choose_parent(self, new_node, near_nodes):
        min_cost = float("inf")
        best_parent = None
        for node in near_nodes:
            cost = node.cost + self.distance((node.x, node.y), (new_node.x, new_node.y))
            if cost < min_cost and self.check_collision_extend(node, new_node):
                best_parent = node
                min_cost = cost
        new_node.cost = min_cost
        new_node.parent = best_parent
        return new_node

    def extract_path(self, end_node):
        path = []
        current_node = end_node
        while current_node is not None:
            path.append((current_node.x, current_node.y))
            current_node = current_node.parent
        path.reverse()
        return path

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def publishWaypointsVisuals(self, newWaypoints=None):
        if not newWaypoints:
            return

        markerArray = MarkerArray()
        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = "base_link"
        savedWaypointsMarker.header.stamp = rospy.Time.now()
        savedWaypointsMarker.lifetime = rospy.Duration(100)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1
        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.scale.x = 0.3
        savedWaypointsMarker.scale.y = 0.3
        savedWaypointsMarker.scale.z = 0.3
        savedWaypointsMarker.pose.orientation.w = 1
        savedWaypointsMarker.color.a = 1
        savedWaypointsMarker.color.b = 1

        for waypoint in newWaypoints:
            p = Point(waypoint[0], waypoint[1], 0.0)
            savedWaypointsMarker.points.append(p)

        markerArray.markers.append(savedWaypointsMarker)
        self.waypointsVisualPub.publish(markerArray)

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

if __name__ == '__main__':
    try:
        my_path = Planner()
        rospy.spin()  # Keeps the program alive and listening for messages
    except rospy.ROSInterruptException:
        pass
