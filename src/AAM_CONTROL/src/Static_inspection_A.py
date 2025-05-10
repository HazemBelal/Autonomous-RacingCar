#!/usr/bin/env python3
import time ,math
import numpy as np
import math
import rospy
import copy
from sensor_msgs import msg
import tf
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
from tf.transformations import euler_from_quaternion
from scipy.spatial import Delaunay
import sys
from sensor_msgs.msg import Imu
import bisect
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
from os import path
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import time
import math
import numpy as np
from std_msgs.msg import Float64

L = 1.5
v_act = 0
flag = 0
r = 0
def waypoints_callback(wp):
        global v_act

def rpm():

        speed = (2 * 22 )/60
        

def odom_callback(odom):
        global flag
        global v_act
   
        v_des = math.sqrt(0.6* 9.81 * r) # v_max
        new_v_des = v_des - (v_des * 75/100) 
        if new_v_des  < 2 :
                new_v_des = 2
                                
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.sqrt(pow(vx,2)+pow(vy,2))
        print("v_Act = ",v_act)
        while flag == 0:
                        
                        print("innnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn")
                        SA = AckermannDriveStamped()
                        SA.drive.steering_angle = 0.47492063492
                        SA.drive.speed = 0
                        SA.drive.steering_angle_velocity = 0
                        SA.drive.acceleration = 0
                        SA.drive.jerk = 0
                        robot_control_pub.publish(SA)
                        time.sleep(1.5)
                        SA = AckermannDriveStamped()
                        SA.drive.steering_angle = 0.0
                        SA.drive.speed = 0.0
                        SA.drive.steering_angle_velocity = 0
                        SA.drive.acceleration = 0
                        SA.drive.jerk = 0
                        time.sleep(1.5)
                        robot_control_pub.publish(SA)
                        SA.drive.steering_angle = -0.47492063492
                        SA.drive.speed = 0
                        SA.drive.steering_angle_velocity = 0
                        SA.drive.acceleration = 0
                        SA.drive.jerk = 0
                        time.sleep(1.5) 
                        robot_control_pub.publish(SA)
                        SA.drive.steering_angle = 0.0
                        SA.drive.speed = 0.0
                        SA.drive.steering_angle_velocity = 0
                        SA.drive.acceleration = 0
                        SA.drive.jerk = 0
                        robot_control_pub.publish(SA)
                        time.sleep(1.5)
                        print("outtttttttttttttttttttttttttttttttttttt")
                        flag = 1









def listner():
        global robot_control_pub
        rospy.init_node('pd_control',anonymous = True)
        rospy.Subscriber('/visual/waypoints',MarkerArray,waypoints_callback)
        rospy.Subscriber("/ground_truth/state_raw",Odometry,odom_callback)
        robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        pub1 = rospy.Publisher("/v_target", Float64, queue_size=1)
        pub2 = rospy.Publisher("/v_actual", Float64, queue_size=1)

if __name__ == "__main__":
        listner()
        rospy.spin()