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
kp = 0.05
kd = 0.02
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
general_v = 8
v_des = 0
integral = 0

def Another_speed(curv):
        v = 5.2/(1.5+curv)
        return v


def speed(curvature,radius):
        centripetal_acceleration = curvature * radius
        total_force = 120 * (centripetal_acceleration + 9.8)
        speed = np.sqrt(total_force * radius / 120)
        return speed



def calculate_curvature(x_points,y_points):
            
            dx = np.gradient(x_points)
            dy = np.gradient(y_points)
            d2x = np.gradient(dx)
            d2y = np.gradient(dy)
                
            curvature = np.abs((dx * d2y - dy * d2x) / (dx**2 + dy**2)**(3 / 2))

            return np.mean(curvature)


def pid(current_speed , desired_speed):
        global integral
        global prev_error 
        error = desired_speed - current_speed
        integral = integral + error
        derivative = error - prev_error
        control_output = 0.1 * error + 0.01 * integral + 0.01 * derivative
        prev_error = error
        return control_output


def waypoints_callback(wp):
        global steering_ang
        global r
        global v_des
        x_points = []
        y_points = []
        for i in wp.markers[0].points:
                x_points.append(i.x)
                y_points.append(i.y)


        if len(wp.markers[0].points) > 2 :
                curvature = calculate_curvature(x_points,y_points)
        else:
                curvature = 1
                print("innnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn")

        print("curvature = ",curvature)
        r = 1/curvature
        new_speed = speed(curvature,r)
        print("new_speed = ",new_speed)
        reduce = new_speed - new_speed * (67.5/100)
        print("reduce = ",reduce)
        vtmsg = Float64()
        vtmsg.data = reduce
        pub3.publish(vtmsg)
        last_speed = pid(v_act,reduce)   
        print("last_speed = ",last_speed)     
        vpidmsg = Float64()
        vpidmsg.data = last_speed
        pub1.publish(vpidmsg)
        if(v_act <= 10):
                x_target = x_points[0]
                y_target = y_points[0]
                print("x_target")
                print(x_target)
                print("y_target")
                print(y_target)
        elif (v_act > 10):
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
        vel = Another_speed(curvature)
        e = y_target # cross hatching error...the distance between the target point and point on the heading line
        ld = math.sqrt(pow(x_target-0.7675,2)+ pow(y_target,2))
        try:
                x = e/ld
        except:
                v_des = general_v
                steering_ang = 0
                alpha = 0
                x = 0
        
        if -1<= x <= 1:
                alpha = math.asin(x) # the angle between heading point and the line between ref point and target point
        try:
                steering_ang = math.atan(2 * 1.535 * ( math.sin(alpha) / ld ))   
        except:
                v_des = general_v
                steering_ang = 0   
        SA = AckermannDriveStamped()
        SA.drive.steering_angle = steering_ang
        SA.drive.speed = vel
        SA.drive.steering_angle_velocity = 0
        SA.drive.acceleration = 0
        SA.drive.jerk = 0
        robot_control_pub.publish(SA)                         


def imu_callback(data):
        global v_des

 
def control_callback(Control):
        global v_act


def odom_callback(odom):
        global v_act
   
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.sqrt(pow(vx,2)+pow(vy,2))
        print("v_act =")
        print(v_act)
        vactmsg = Float64()
        vactmsg.data = v_act
        pub2.publish(vactmsg)





def listner():
        global robot_control_pub
        global pub2
        global pub1
        global pub3
        rospy.init_node('pd_control',anonymous = True)
        rospy.Subscriber('/visual/waypoints',MarkerArray,waypoints_callback)
        rospy.Subscriber('/sensor_imu_hector',Imu,imu_callback)
        rospy.Subscriber("/ground_truth/state_raw",Odometry,odom_callback)
        rospy.Subscriber("/robot_control/command",AckermannDriveStamped,control_callback)
        #rospy.Subscriber('/car_pose', Path,self.refrence_callback)
        robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        pub1 = rospy.Publisher("/v_target", Float64, queue_size=1)
        pub2 = rospy.Publisher("/v_actual", Float64, queue_size=1)
        pub3 = rospy.Publisher("/After_pid", Float64, queue_size=1)

if __name__ == "__main__":
        listner()
        rospy.spin()