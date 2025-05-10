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

error = 0
kp = 0.0225
kd = 0.01
v_actual = 0
L = 1.5
e = 0
steering_ang = 0
p = 0
d = 0
x = 0
v_act = 0
rad = 0
r = 0
prev_error = 0
velocity = 0
A = 0
radius = 0
x_ref = 0
y_ref = 0
alpha = 0
x_target = 0
y_target = 0
def waypoints_callback(wp):
        global steering_ang
        global v_des
        global r
        flag = 0 
        arrX=[]
        arrY=[]

        for point in wp.markers[0].points:
                arrX.append(point.x)
                arrY.append(point.y)
                flag += 1
                if flag == 3:
                        break

        print("flag = ")
        print(flag)
        if flag == 3:
                a = math.sqrt(pow(arrX[1] - arrX[0],2)+ pow(arrY[1] - arrY[0],2))
                print("a =")
                print(a)
                b = math.sqrt(pow(arrX[2] - arrX[1],2)+ pow(arrY[2] - arrY[1],2))
                print("b =")
                print(b)
                c = math.sqrt(pow(arrX[2] - arrX[0],2)+ pow(arrY[2] - arrY[0],2))
                print("c =")
                print(c)
                s = (a+b+c)/2
                Area = math.sqrt(s*(s-a)*(s-b)*(s-c))
                # sq_a = math.sqrt(a)
                # sq_b = math.sqrt(b)
                # sq_c = math.sqrt(c)
                try:
                        #r = (sq_a * sq_b * sq_c) / (math.sqrt((sq_a + sq_b + sq_c)*(-sq_a + sq_b + sq_c)*(sq_a - sq_b + sq_c)*(sq_a + sq_b - sq_c)))
                        r = (a * b * c)/(4 * Area)
                        print("r =")
                        print(r)
                except:
                        v_des = 31.5
                        steering_ang = 0
                        x = 0
                        alpha = 0
                        r = 168.75

        x_points = []
        y_points = []
        for i in wp.markers[0].points:
                x_points.append(i.x)
                y_points.append(i.y)
        if(v_act <= 10):
                x_target = x_points[0]
                y_target = y_points[0]
                print("x_target")
                print(x_target)
                print("y_target")
                print(y_target)
        elif (v_act <= 20):
                try:
                        x_target = x_points[1]
                        y_target = y_points[1]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target)
                except:
                        x_target = x_points[0]
                        y_target = y_points[0]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target) 
        elif (v_act > 20):
                try:
                        x_target = x_points[2]
                        y_target = y_points[2]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target) 
                except:
                        try:
                                x_target = x_points[1]
                                y_target = y_points[1]
                        except:
                                x_target = x_points[0]
                                y_target = y_points[0]    
       
        e = y_target # cross hatching error...the distance between the target point and point on the heading line
        ld = math.sqrt(pow(x_target,2)+ pow(y_target,2))
        try:
                x = e/ld
        except:
                v_des = 31.5
                steering_ang = 0
                alpha = 0
                x = 0
        
        if -1<= x <= 1:
                alpha = math.asin(x) # the angle between heading point and the line between ref point and target point
        try:
                steering_ang = math.atan(2 * 1.535 * ( math.sin(alpha) / ld ))   
        except:
                v_des = 31.5
                steering_ang = 0   
        print("steering_angle")
        print(steering_ang)
        SA = AckermannDriveStamped()
        SA.drive.steering_angle = steering_ang
        SA.drive.speed = 4
        SA.drive.steering_angle_velocity = 0
        SA.drive.acceleration = 0
        SA.drive.jerk = 0
        robot_control_pub.publish(SA)                         


def imu_callback(data):
        global v_des

 
def control_callback(Control):
        global v_act


def odom_callback(odom):
        global p
        global d
        global v_act
        global velocity
        x_ref = 0
        y_ref = 0
        v_des = 12 #math.sqrt(0.6* 9.81 * r) # v_max
        if v_des > 15:
                v_des = 15
        # vtmsg = Float64()
        # vtmsg.data = v_des
        # pub1.publish(vtmsg)s
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.sqrt(pow(vx,2)+pow(vy,2))
        print("v_act =")
        print(v_act)
        # vactmsg = Float64()
        # vactmsg.data = v_act
        # pub2.publish(vactmsg)
        error = v_des - v_act
        p = kp * error
        d = kd * (error / 0.07)
        velocity = p + d
        





def listner():
        global robot_control_pub
        rospy.init_node('pd_control',anonymous = True)
        rospy.Subscriber('/visual/waypoints',MarkerArray,waypoints_callback)
        rospy.Subscriber('/sensor_imu_hector',Imu,imu_callback)
        rospy.Subscriber("/ground_truth/state_raw",Odometry,odom_callback)
        rospy.Subscriber("/robot_control/command",AckermannDriveStamped,control_callback)
        #rospy.Subscriber('/car_pose', Path,self.refrence_callback)
        robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        pub1 = rospy.Publisher("/v_target", Float64, queue_size=1)
        pub2 = rospy.Publisher("/v_actual", Float64, queue_size=1)

if __name__ == "__main__":
        listner()
        rospy.spin()
