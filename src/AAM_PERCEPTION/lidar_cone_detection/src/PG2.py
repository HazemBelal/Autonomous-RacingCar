#!/usr/bin/env python3
import rospy
import math
import numpy as np
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import multiprocessing
import time

##copied from fromwhat.py
##first try for the new approach

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

        self.cone_x=x
        self.cone_y=y
        self.color=c


GT = False

class planner:
   def __init__(self,namespace='rrt'):
        rospy.init_node("rrt_path_planner", anonymous = True)
        rospy.Subscriber("/converted_pc", MarkerArray,self.cones_pipeline)



        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)

        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.notSelectedPub = rospy.Publisher("/notSelected", MarkerArray, queue_size=1)

        self.car_pos= node(0,0,0,None,True,90,0)
        self.track_width=3
        self.cone_L_list=[]
        self.cone_R_list=[]
        self.chosen=[]

        self.node_list = []
        self.node_expand=0.3
        self.expand_angle=180
        self.expand=6

        self.test = []
        self.list_itr = []



        self.ryt = 0
        self.costTol = 1.9

        self.depth = 0
        self.listOfNodes = []

   def distance(self,pont1_x,pont1_y,point2_x,pont2_y):
        return math.sqrt(math.pow(pont1_x-point2_x,2)+math.pow(pont1_y-pont2_y,2))

   def cones_pipeline(self,cones_msg):
        global GT

        cones=[]
        rospy.loginfo("test")

        for cone in cones_msg.markers:
            cone_x=cone.pose.position.x
            cone_y=cone.pose.position.y
            cone_T=(cone_x,cone_y)
            cones.append(cone_T)

        self.car_pos = node(2,0,0,None,True,90,0)
        if(GT == False):
            self.generateTree(self.car_pos)
            GT = True
           
        self.updateCostAndFlag(cones, self.node_list)        


   def generateTree(self, root):
        node_list=[]
        tol = 5 ##tolarance angle
        radius = 1.3
        degree = root.angle

        while(degree <= root.angle + 70):
            
            x_random = root.x + (radius*(math.sin(math.radians(degree))))
            y_random = root.y + (radius*1.25*(math.cos(math.radians(degree))))

            new_node= node(x_random,y_random,0,root,False,degree,(self.depth+1))

            degree = degree + tol

            node_list.append(new_node)


        degree = root.angle + tol
        while(degree > root.angle - 70):

            x_random = root.x + (radius*(math.sin(math.radians(degree))))
            y_random = root.y + (radius*1.25*(math.cos(math.radians(degree)))) ##1.5 is scaling factor to match elipse

            new_node= node(x_random,y_random,0,root,False,degree,(self.depth+1))

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
            if current_level not in best_nodes_by_level or (current_cost > best_nodes_by_level[current_level]['cost'] and current_cost < 2.4):
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




   def cost(self,point_x,pointy,cone_list):

        counter=0
        cost=0
        min = 100
        while(counter!=len(cone_list)):
            dst = self.distance(point_x,pointy,cone_list[counter][0],cone_list[counter][1])
            if(dst<= self.costTol or point_x < 0.2 ):
                cost=0
                return cost
            # distance between nearest cone
            if min > dst:
                min = dst

            counter = counter+1

        cost = min

        return cost




   def flag(self,point_x,pointy,cone_list):
        counter=0
        ft = False
        min = 100
        while(counter!=len(cone_list)):
            dst = self.distance(point_x,pointy,cone_list[counter][0],cone_list[counter][1])
            if(dst<= self.costTol or point_x < 0.2):
                ft = False
                return ft

            if min > dst:
                min = dst

            counter=counter+1

        fg =self.sig(min)
        if(fg == 0):
            return False
        else:
            return True


   def sig(self,x):
        return 1/(1 + np.exp(-x))




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





##10th of novamber
## well this verion of code now can generate the trees one time
## and update their cost and flags dunamicly after each update 
## form the subscriber, now what's next
## create new pathsampler function that creates the best branch
## from the avalible nodes

##17th of novamber
## I have added new function called PPP2 for publishing true but
## not best nodes to trace the code bahviour and assign good tolerance
## for the cost functions
##The problem is the same until now the turns are rough which
## makes the car slid near to a cone very roughly each time


##26th of novamber
## Well I have done the following things:
## 1- modified the polar equation of the tree to be more elipsce shaped in y-axis
## 2- modified the tolerance to be 1.9 (the best one until now)
## 3- added condition to check that the highest cost of node be at min 2.4 to avoid
##   the turning too much on the corners
## Resultd: it worked in the small track amazingly well
