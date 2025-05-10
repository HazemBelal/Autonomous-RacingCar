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
from scipy.interpolate import CubicSpline
from scipy.integrate import quad



error = 0
kp = 0.1
kd = 0.02
ki = 0.05
I = 0 
PE = 0
steering_ang = 0
v_actual = 0
L = 1.5
e = 0
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
curvature = 0
steering_ang = 0
II = 0

def Another_speed(curv):
        v = 8.5/(1.5+curv)
        return v


def calculate_curvature(x_points,y_points):
            
            dx = np.gradient(x_points)
            dy = np.gradient(y_points)
            d2x = np.gradient(dx)
            d2y = np.gradient(dy)
                
            curvature = np.abs((dx * d2y - dy * d2x) / (dx**2 + dy**2)**(3 / 2))

            return np.mean(curvature)


def waypoints_callback(wp):
        global I
        global PE
        global curvature
        global velocity
        global radius
        global steering_ang
        global r

        x_points = []
        y_points = []
        for i in wp.markers[0].points:
                x_points.append(i.x)
                y_points.append(i.y)

        arrX=[]
        arrY=[]
        flag = 0
        for point in wp.markers[0].points:
                arrX.append(point.x)
                arrY.append(point.y)
                flag += 1
                if flag == 3:
                        break
        curv = calculate_curvature(x_points,y_points)
        vel = Another_speed(curv)
        print("velllllllllllllllllllllllll = ",vel)

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
                try:
                        r = (a * b * c)/(4 * Area)
                        print("r =")
                        print(r)
                except:
                        
                        r = 168.75
              


        # j = 0
        # if len(wp.markers[0].points) > 2 : 
        #         cs = CubicSpline(x_points, y_points)
        #         smooth_x = np.linspace(min(x_points), max(x_points), 50)
        #         smooth_y = cs(smooth_x)  
        #         for i in  wp.markers[0].points:
        #                 x_points[j] = smooth_x[j]      
        #                 y_points[j] = smooth_y[j]
        
        # if len(wp.markers[0].points) > 2 :
        #         curvature = calculate_curvature(x_points,y_points)
        # else:
        #         curvature = 1
        # print("curvature = ",curvature)

        if(v_act <= 10):
                x_target = x_points[1] + 0.75
                y_target = y_points[1]
                print("x_target")
                print(x_target)
                print("y_target")
                print(y_target)
        elif (v_act <= 20):
                try:
                        x_target = x_points[2]
                        y_target = y_points[2]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target)
                except:
                        x_target = x_points[1]
                        y_target = y_points[1]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target) 
        elif (v_act > 20):
                try:
                        x_target = x_points[3]
                        y_target = y_points[3]
                        print("x_target")
                        print(x_target)
                        print("y_target")
                        print(y_target) 
                except:
                        try:
                                x_target = x_points[2]
                                y_target = y_points[2]
                        except:
                                x_target = x_points[1]
                                y_target = y_points[1]    

        heading_angle_ref = math.atan2(y_target,x_target)
        yaw_path = (v_act * math.sin(steering_ang))/ 1.5
        yaw_diff = heading_angle_ref - yaw_path
        e = y_target # cross tracking error...the distance between the target point and point on the heading line
        P = kp * e
        I = I + (ki * e * 0.07)
        D = kd * (e - PE) / 0.07
        control_signal = P + I + D 
        PE = e
        steering_ang = heading_angle_ref + math.atan((0.003125 * e)/(2 + v_act))
        print("steereeeeeeeeeeeeeeee = ",steering_ang)

        SA = AckermannDriveStamped()
        SA.drive.steering_angle = steering_ang
        SA.drive.speed = vel
        SA.drive.steering_angle_velocity = 0
        SA.drive.acceleration = 0
        SA.drive.jerk = 0
        robot_control_pub.publish(SA)                         


def imu_callback(data):
        global angvel,yaw
        angvel = data.angular_velocity.z
        orientation_list = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

 
def control_callback(Control):
        global v_act


def odom_callback(odom):
        global p
        global d
        global v_act
        global prev_error
        global new_v_des
        global II
        global velocity
        v_des = math.sqrt(9.81 * 0.6 * r)
        new_v_des = v_des - (v_des * 85/100)
        if new_v_des < 1.7:
                new_v_des = 1.7

        vtmsg = Float64()
        vtmsg.data = v_des
        pub1.publish(vtmsg)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.sqrt(pow(vx,2)+pow(vy,2))
        print("v_act = ",v_act)
        vactmsg = Float64()
        vactmsg.data = v_act
        pub2.publish(vactmsg)


        





def listner():
        global robot_control_pub
        global pub2
        global pub1
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







