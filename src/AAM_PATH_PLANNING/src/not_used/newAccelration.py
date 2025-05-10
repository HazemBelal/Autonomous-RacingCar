#!/usr/bin/env python3
import rospy
import math
import numpy as np
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import operator
import time

class node:
    def __init__(self,coordinate_x,coordinate_y,cost,parent,flag,angle,level):

        self.x=coordinate_x
        self.y=coordinate_y
        self.cost=cost
        self.parent=parent
        self.flag=flag
        self.angle = angle
        self.level = level

class cone:
    def __init__(self,x,y,c):

        self.x = x
        self.y = y
        self.c = c


GT = False

class planner:
   def __init__(self,namespace='rrt'):
        rospy.init_node("rrt_path_planner", anonymous = True)
        rospy.Subscriber("/camera_cones_marker", MarkerArray,self.cones_pipeline)



        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)

        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.notSelectedPub = rospy.Publisher("/notSelected", MarkerArray, queue_size=1)

        self.car_pos= node(0,0,0,None,True,90,0)
        self.cone_L_list=[]
        self.cone_R_list=[]
        self.chosen=[]

        self.node_list = []

        self.test = []
        self.list_itr = []



        self.ryt = 0
        self.costTol = 1.3

        self.depth = 0
        self.listOfNodes = []

   def distance(self,pont1_x,pont1_y,point2_x,pont2_y):
        return math.sqrt(math.pow(pont1_x-point2_x,2)+math.pow(pont1_y-pont2_y,2))

   def cones_pipeline(self,cones_msg):
        global GT

        cones_yellow = []
        cones_blue = []
        cones_orange = []

        cones=[]
        print("halaaa")

        for c in cones_msg.markers:

            cone_x=c.pose.position.x
            cone_y=c.pose.position.y
            cone_c = "x"
            if(c.color.r == 0 and c.color.g == 0 and c.color.b == 200):
                cone_c = "blue"
                cones_blue.append((cone_x, cone_y))
            
            elif(c.color.r == 200 and c.color.g == 200 and c.color.b == 0):
                cone_c = "yellow"
                cones_yellow.append((cone_x, cone_y))

            elif(c.color.r == 200 and c.color.g == 0 and c.color.b == 0):
                cone_c = "orange"
                cones_orange.append((cone_x, cone_y))       

            cone_T= cone(cone_x,cone_y,cone_c)
            cones.append(cone_T)

        self.car_pos = node(2,0,0,None,True,90,0)
        if(GT == False):
            self.generateTree(self.car_pos)
            GT = True

        if(len(cones_orange) > 1 and (len(cones_yellow) > 0 or len(cones_blue) > 0)):
            cones_blue, cones_yellow = self.appendToTheRightSide(cones_blue, cones_yellow, cones_orange)        

        if(len(cones_yellow) > 0 and len(cones_blue) > 0):
            self.oldAccelaration(cones_blue, cones_yellow)

        elif(len(cones) > 0):
            self.updateCostAndFlag(cones, self.node_list)

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
            
            # Find nearest blue cone
            for blue_cone in blue_cones:
                dist = self.distance(orange_cone[0], orange_cone[1],blue_cone[0], blue_cone[1])
                if dist < min_blue_distance:
                    min_blue_distance = dist
            
            # Find nearest yellow cone
            for yellow_cone in yellow_cones:
                dist = self.distance(orange_cone[0], orange_cone[1], yellow_cone[0], yellow_cone[1])
                if dist < min_yellow_distance:
                    min_yellow_distance = dist
            
            # Append orange cone to the appropriate list based on distances
            if min_yellow_distance < min_blue_distance:
                yellow_nearest_oranges.append(orange_cone)
            else:
                blue_nearest_oranges.append(orange_cone)
        
        return blue_nearest_oranges, yellow_nearest_oranges
        


   def generateTree(self, root):
        node_list=[]
        tol = 5 ##tolarance angle
        radius = 1.3
        degree = root.angle

        while(degree <= root.angle + 70):
            
            x_random = root.x + (radius*(math.sin(math.radians(degree))))
            y_random = root.y + (radius*1.25*(math.cos(math.radians(degree))))

            new_node= node(x_random,y_random,0,root,True,degree,(self.depth+1))

            degree = degree + tol

            node_list.append(new_node)


        degree = root.angle + tol
        while(degree > root.angle - 70):

            x_random = root.x + (radius*(math.sin(math.radians(degree))))
            y_random = root.y + (radius*1.25*(math.cos(math.radians(degree)))) ##1.5 is scaling factor to match elipse

            new_node= node(x_random,y_random,0,root,True,degree,(self.depth+1))

            node_list.append(new_node)
                
            degree = degree - tol

  

        if(self.depth>= 3):
            self.node_list = self.node_list + node_list
            return self.node_list
            
        else:
            self.depth = self.depth + 1
            self.node_list = self.node_list + node_list
            newRoot = root
            newRoot.x = newRoot.x + radius
            self.generateTree(newRoot)        

   def updateCostAndFlag(self,cone_list,nodes):
        for Qnode in range(len(nodes)):
            nodes[Qnode].cost = self.cost(nodes[Qnode].x, nodes[Qnode].y, cone_list)
            nodes[Qnode].flag = self.flag(nodes[Qnode].x, nodes[Qnode].y, cone_list)

            # Check if the node's previous level has any nodes with a true flag
            if nodes[Qnode].level > 1:
                previous_level_nodes = [node for node in nodes if node.level == nodes[Qnode].level - 1]
                
                # Check if any node in the previous level has a true flag
                if any(node.flag for node in previous_level_nodes):
                    # Do nothing, the flag is already true
                    pass
                else:
                    # Set cost and flag to 0 and False if the previous level has no true flag
                    nodes[Qnode].cost = 0
                    nodes[Qnode].flag = False

        self.ppp2(self.node_list)
        self.find_best_nodes(self.node_list)


   def find_best_nodes(self,tree_nodes):
        # Dictionary to store the best node for each level
        best_nodes_by_level = {}

        # Iterate through all nodes in the tree
        for node in tree_nodes:
            current_level = node.level
            current_cost = node.cost  

            # If the level is not in the dictionary or the current node has a higher cost, update the best node
            if current_level not in best_nodes_by_level or (current_cost > best_nodes_by_level[current_level]['cost'] and current_cost < 2):
                best_nodes_by_level[current_level] = {'node': node, 'cost': current_cost}

        # Extract the best nodes from the dictionary
        best_nodes = [info['node'] for info in best_nodes_by_level.values()]
        print(best_nodes_by_level)

        self.ppp(best_nodes)


   def ppp(self,node_list):
        iu = []
        for u in node_list:
            if(u.flag == True):
                iu.append((u.x,u.y))
                print('(',u.x,u.y,')',u.cost)
        for i in range(15):
            self.publishWaypointsVisuals(iu)

   def ppp2(self,node_list):
        iu = []
        for u in node_list:
            if(u.flag == True):
                iu.append((u.x,u.y))
                # print('(',u.x,u.y,')',u.cost)
        for i in range(15):
            self.publishNotSelected(iu)


   def cost(self,point_x,point_y,cone_list):

        counter=0
        cost=0
        min = 100
        min_y = 777
        min_b = 777
        while(counter!=len(cone_list)):
            dst = self.distance(point_x,point_y,cone_list[counter].x,cone_list[counter].y)

            if(dst<= self.costTol or point_x < 0.2 ):
                cost=0
                return cost
            
            if(cone_list[counter].c == "yellow"):
                if(min_y > dst):
                    min_y = dst
            elif(cone_list[counter].c == "blue"):
                if(min_b > dst):
                    min_b = dst
            # distance between nearest cone
            if min > dst:
                min = dst

            counter = counter+1

        cost = min

        return cost


   def flag(self,point_x,point_y,cone_list):
        counter=0
        min_y = 777
        min_b = 777
        while(counter!=len(cone_list)):
            dst = self.distance(point_x,point_y,cone_list[counter].x,cone_list[counter].y)

            if(dst<= self.costTol or point_x < 0.2 ):
                return False
            
            if(cone_list[counter].c == "yellow"):
                if(min_y > dst):
                    min_y = dst
            elif(cone_list[counter].c == "blue"):
                if(min_b > dst):
                    min_b = dst

            counter = counter+1

        return True   

   def oldAccelaration(self, cones_blue, cones_yellow):
        """
        Calculate midpoints between corresponding cones of the same index.

        Args:
            cones_blue (list): List of blue cone coordinates.
            cones_yellow (list): List of yellow cone coordinates.

        Returns:
            list: List of midpoints.
        """
        midpoints = []

        min_length = min(len(cones_blue), len(cones_yellow))
        for i in range(min_length):
            midpoint_x = (cones_blue[i][0] + cones_yellow[i][0]) / 2
            midpoint_y = (cones_blue[i][1] + cones_yellow[i][1]) / 2
            midpoints.append((midpoint_x, midpoint_y))


        self.publishWaypointsVisuals(midpoints)

   def publishWaypointsVisuals(self, newWaypoints = None):

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




   def publishNotSelected(self, newWaypoints = None):

            markerArray = MarkerArray()

            savedWaypointsMarker = Marker()
            savedWaypointsMarker.header.frame_id = "base_link"
            savedWaypointsMarker.header.stamp = rospy.Time.now()
            savedWaypointsMarker.lifetime = rospy.Duration(100)
            savedWaypointsMarker.ns = "saved-publishNotSelected"
            savedWaypointsMarker.id = 1

            savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
            savedWaypointsMarker.action = savedWaypointsMarker.ADD
            savedWaypointsMarker.scale.x = 0.3
            savedWaypointsMarker.scale.y = 0.3
            savedWaypointsMarker.scale.z = 0.3
            savedWaypointsMarker.pose.orientation.w = 1

            savedWaypointsMarker.color.a = 0.5
            savedWaypointsMarker.color.r = 1
            savedWaypointsMarker.color.b = 0.3

            for waypoint in newWaypoints:

                p = Point(waypoint[0], waypoint[1], 0.0)
                savedWaypointsMarker.points.append(p)

            markerArray.markers.append(savedWaypointsMarker)

            self.notSelectedPub.publish(markerArray)





if __name__ == '__main__':
    try:
        my_path = planner()
    except BaseException:
        pass
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

