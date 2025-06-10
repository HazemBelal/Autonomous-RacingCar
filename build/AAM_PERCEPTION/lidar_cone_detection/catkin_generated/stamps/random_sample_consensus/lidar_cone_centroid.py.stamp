#!/usr/bin/env python

from cmath import cos, sin
from threading import local
import numpy as np
import math
from numpy.core.fromnumeric import shape, size
from numpy.lib.function_base import append
from numpy.lib.type_check import asfarray
import rospy
from sensor_msgs import msg
import tf
import matplotlib
import matplotlib.pyplot as plt
import ros_numpy as rnp
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import random

file = open('putlidar')

class projection_pipeline():

    def __init__(self, namespace = 'lidar_cone_centroid'):

        rospy.init_node("lidar_cone_centroid", anonymous = True)

        self.no_ground_pc = rospy.Subscriber("/clustering_points",PointCloud2,self.pc_callback)
        self.centroidpublisher = rospy.Publisher('/centroid_lidar_cones_marker', MarkerArray,queue_size=0)
        self.thresh=2
        
    



    def pc_callback(self, no_ground_pc):
        xyz_points = rnp.point_cloud2.pointcloud2_to_xyz_array(no_ground_pc, remove_nans=True)
        X=[]
        Y=[]
        for point in xyz_points:
            X.append(point[0])
            Y.append(point[1])
        self.centroid_visuals(X,Y)








    def centroid_visuals(self,resultX,resultY):
        c = 0

        self.rviz_msg = Marker()
        self.rviz_msg_array = MarkerArray()

        while c < len(resultX):
        
            
            x_cone = resultX[c]
            y_cone = resultY[c]

            c +=1
            count = 0
            MARKERS_MAX = 100

            self.rviz_msg = Marker()
            self.rviz_msg.header.frame_id = "velodyne"
            self.rviz_msg.ADD
            self.rviz_msg.SPHERE
            self.rviz_msg.pose.position.x = x_cone
            self.rviz_msg.pose.position.y = y_cone
            self.rviz_msg.pose.position.z = 0
            self.rviz_msg.lifetime = rospy.Duration(0.5)
            self.rviz_msg.pose.orientation.w = 1
            self.rviz_msg.scale.x = 0.2
            self.rviz_msg.scale.y = 0.2
            self.rviz_msg.scale.z = 0.2
            self.rviz_msg.color.a = 1
            self.rviz_msg.color.g = 1

            self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
            self.rviz_msg.type = self.rviz_msg.SPHERE
	    #self.rviz_msg.type = Marker.MESH_RESOURCE
            self.rviz_msg.mesh_use_embedded_materials = True

            if(count > MARKERS_MAX):
                self.rviz_msg_array.markers.pop(0)
            
            self.rviz_msg_array.markers.append(self.rviz_msg)
            m = 0
            id = 0
            for m in self.rviz_msg_array.markers:
                m.id = id
                id += 1
        
        self.centroidpublisher.publish(self.rviz_msg_array)

        return




if __name__ == '__main__':
    try:
        project = projection_pipeline()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
