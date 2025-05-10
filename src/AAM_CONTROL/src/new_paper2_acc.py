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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

error = 0
kp = 0.05
ki = 0.03
kd = 0.02
v_actual = 0
L = 1.5
e = 0
steering_ang = 0
I = 0
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
PE = 0
integral = 0
k = 0.0001
def pid(current_speed , desired_speed):
        global integral
        global prev_error 
        error = desired_speed - current_speed
        integral = integral + error
        derivative = error - prev_error
        control_output = 0.1 * error + 0.01 * integral + 0.01 * derivative
        prev_error = error
        return control_output

def Another_speed(curv):
        v = 3/(1+curv)
        return v


def calculate_curvature(x_points,y_points):
            
            dx = np.gradient(x_points)
            dy = np.gradient(y_points)
            d2x = np.gradient(dx)
            d2y = np.gradient(dy)
                
            curvature = np.abs((dx * d2y - dy * d2x) / (dx**2 + dy**2)**(3 / 2))

            return np.mean(curvature)

def calculate_distance(x_points,y_points,i):
        try:
                x_point = x_points[i]
                y_point = y_points[i]
        except:   
                x_point = x_points[0]
                y_point = y_points[0]     

        dist2 = math.sqrt(pow(x_point,2) + pow(y_point,2))
        #distance_between_points = dist2 - dist1
        return dist2 

        



def waypoints_callback(wp):
        global steering_ang
        global v_des
        global r
        global I
        global PE
        global k
        global SA
        global vel
        x_points = []
        y_points = []
        for i in wp.markers[0].points:
                x_points.append(i.x)
                y_points.append(i.y)
        
        x_target = x_points[0] + 0.7675
        y_target = y_points[0]

        # flag = 0
        # flag2 = 0
        # i = 0
        # while(flag == 0):
        #         distttttttttttt = calculate_distance(x_points,y_points,i)
        #         print("distttttttttttttttttttttttttttttt",distttttttttttt)
        #         if(distttttttttttt >= 3.3):
        #                 x_target = x_points[i] + 0.7675
        #                 y_target = y_points[i]
        #                 print("x_target = ",x_target)
        #                 print("y_target = ",y_target)
        #                 flag = 1

        #         elif (distttttttttttt < 3.3):
        #                 i = i + 1        
        #         print("whileeeee looppppp")
        #         flag2 = flag2 + 1
        #         if flag2 > 3:
        #                 break
        # if flag2 > 3:
        #         x_target = x_points[1] + 0.7675
        #         y_target = y_points[1]
        heading_angle_ref = math.atan2(y_target,x_target)
        # #yaw = math.atan((y_points[1]-y_points[0])/(x_points[1]-x_points[0]))
        # try:
        #         k = calculate_curvature(x_points,y_points)     
        # except:
        #         k = 1         
        # vel = Another_speed(k)   
        #print("k = ",k)
        ec = y_target # cross tracking error...the distance between the target point and point on the heading line
        k_soft = 3
        kd_yaw = 0.0159375#0.001875 #0.03 #0.001875
        kss =  120 / (2*10000)
        yaw_rate = (v_act * math.sin(steering_ang))/1.53
        try:
                steering_ang = ((heading_angle_ref - kss * pow(v_act,2) * k) + math.atan((0.003125 * ec)/(k_soft + v_act)) + kd_yaw * ( yaw_rate - (v_act * k)))
        except:
                v_des = 31.5
        #         steering_ang = 0   
        # if (steering_ang >= 0.47492063492):
        #         steering_ang = 0.47492063492  
        # elif(steering_ang <= -0.47492063492):
        #         steering_ang = -0.47492063492

        print("steering_angle",steering_ang)
        
        SA = AckermannDriveStamped()
        SA.drive.steering_angle = steering_ang
        SA.drive.speed = 2
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
        global velocity
        global new_v_des
        #if k == 0:
        #        v_des = 6.5     
        #else:           
        #        v_des = math.sqrt(9.81 * 0.6 * (1/(k)) )
        #print("v_des = ",v_des)
        #new_v_des = v_des - v_des * 70/100
        #print("new_v_des =",new_v_des)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.sqrt(pow(vx,2)+pow(vy,2))
        print("v_act = ",v_act)
        #velocity = pid(v_act,new_v_des)
        #print("velocityyyyyyyyyyy = ", velocity)

def stop_car(v_act,v_target,deceleration_time):
        current_speed = v_act
        acceleration = (current_speed - v_target) / deceleration_time
        while current_speed > v_target:
                print("Current speed:", current_speed, "m/s")
                current_speed -= acceleration
        return current_speed


      
def cones_callback(cone):
        print("i am in conesssssssssssssssss callbackkkkkkkkkkkkkkkkkkkkkkkk")
        cones_yellow = []
        cones_blue = []
        cones_orange = []
        cones_orange_x = []
        cones_orange_y = []
        big_cone = []
        for cone_marker in cone.markers:
            cone_x = cone_marker.pose.position.x
            cone_y = cone_marker.pose.position.y         
            if cone_marker.color.r == 0 and cone_marker.color.g == 0 and cone_marker.color.b == 200:
                        cones_blue.append((cone_x, cone_y))
            elif cone_marker.color.r == 200 and cone_marker.color.g == 200 and cone_marker.color.b == 0:
                        cones_yellow.append((cone_x, cone_y))
            elif cone_marker.color.r == 200 and cone_marker.color.g == 100 and cone_marker.color.b == 0:
                        cones_orange_x.append(cone_x)
                        cones_orange_y.append(cone_y)
                        cones_orange.append((cone_x,cone_y))
            elif cone_marker.color.r == 200 and cone_marker.color.g == 0 and cone_marker.color.b == 0:
                        big_cone.append((cone_x,cone_y))
        
        if not cones_yellow and not cones_blue and not big_cone and cones_orange:

                print("I ammmm in orangeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")

                if cones_orange:
                        x_axis_orange_cone = cones_orange_x[0]
                        y_axis_orange_cone = cones_orange_y[0]
                        distance = math.sqrt(pow(x_axis_orange_cone,2) + pow(y_axis_orange_cone,2))
                else:
                        distance = 100
                        print("conesssssssss distanceeeeeeeeeeeeeeeeeeeeeeeee = ",distance)

                if distance < 4.7:
                        while(True):
                                SA = AckermannDriveStamped()
                                SA.drive.steering_angle = 0
                                SA.drive.speed = 0
                                SA.drive.steering_angle_velocity = 0
                                SA.drive.acceleration = 0
                                SA.drive.jerk = 0
                                robot_control_pub.publish(SA)
                

                # size_array = len(cones_orange)
                # if size_array > 5: 
                #                 print("i am in moreeeee than 44444444444444444444444")
                #                 SA = AckermannDriveStamped()
                #                 SA.drive.steering_angle = steering_ang
                #                 SA.drive.speed = 1
                #                 SA.drive.steering_angle_velocity = 0
                #                 SA.drive.acceleration = 0
                #                 SA.drive.jerk = 0
                #                 robot_control_pub.publish(SA)  
                # elif size_array < 5:
                #                 while(True):
                #                         print("i am equallllllllllllllllll 4")
                #                         SA = AckermannDriveStamped()
                #                         SA.drive.steering_angle = 0
                #                         SA.drive.speed = 0
                #                         SA.drive.steering_angle_velocity = 0
                #                         SA.drive.acceleration = 0
                #                         SA.drive.jerk = 0
                #                         robot_control_pub.publish(SA) 




def listner():
        global robot_control_pub
        global pub1
        global pub2
        global waypoints_visual_pub
        rospy.init_node('pd_control',anonymous = True)
        rospy.Subscriber('/visual/waypoints',MarkerArray,waypoints_callback)
        rospy.Subscriber('/sensor_imu_hector',Imu,imu_callback)
        rospy.Subscriber("/ground_truth/state_raw",Odometry,odom_callback)
        rospy.Subscriber("/robot_control/command",AckermannDriveStamped,control_callback)
        rospy.Subscriber('/camera_cones_marker', MarkerArray, cones_callback)
        #rospy.Subscriber('/car_pose', Path,self.refrence_callback)
        robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        waypoints_visual_pub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)
        pub1 = rospy.Publisher("/v_target", Float64, queue_size=1)
        pub2 = rospy.Publisher("/v_actual", Float64, queue_size=1)

if __name__ == "__main__":
        listner()
        rospy.spin()
