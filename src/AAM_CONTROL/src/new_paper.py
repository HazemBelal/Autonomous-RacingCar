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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
error = 0
from ma_rrt_path_plan.msg import WaypointsArray
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
big_cone_counter = 0
last_cone_time = time.time()
def pid(current_speed , desired_speed):
        global prev_error 
        global integral
        error = desired_speed - current_speed
        integral = integral + error * 0.07
        derivative = (error-prev_error) / 0.07
        control_output = 0.1 * error+0.02 *integral + 0.01 * derivative
        prev_error = error
        return control_output

def Another_speed(curv):
        v = 6/(1.5+curv)
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
    global steering_ang, v_des, v_act, k, robot_control_pub
    # Create lists for the x and y coordinates from the received waypoints.
    x_points = []
    y_points = []
    for waypoint in wp.waypoints:
        x_points.append(waypoint.x)
        y_points.append(waypoint.y)
    rospy.loginfo("Received %d waypoints", len(x_points))
    
    size_array = len(x_points)
    if size_array == 0:
        rospy.logwarn("No waypoints received.")
        return

    # For local planning, we assume the car is at (0,0) in base_link.
    # (In your odometry callback you already set carPosX, carPosY = 0.)
    # So the waypoint positions are already relative to the car.
    
    # Use a simple method to pick a target waypoint.
    flag = 0
    flag2 = 0
    i = 0
    # We look for the first waypoint that is at least 3.3 m away.
    while flag == 0 and i < size_array:
        dist = math.sqrt(x_points[i]**2 + y_points[i]**2)
        if dist >= 3.3:
            x_target = x_points[i] + 0.7675  # adjust offset if needed
            y_target = y_points[i]
            flag = 1
        else:
            i += 1
            flag2 += 1
            if flag2 > 3:
                break
    if flag == 0 and size_array > 1:
        x_target = x_points[1] + 0.7675
        y_target = y_points[1]
    
    # Calculate the heading angle reference from the target point.
    heading_angle_ref = math.atan2(y_target, x_target)
    rospy.loginfo("Target point: (%.2f, %.2f), heading angle ref: %.2f", x_target, y_target, heading_angle_ref)
    
    # Calculate curvature from the waypoint list.
    try:
        k = calculate_curvature(x_points, y_points)
    except Exception as e:
        rospy.logwarn("Error calculating curvature: %s", e)
        k = 1

    # Determine desired speed based on curvature.
    vel = Another_speed(k)
    
    # Compute cross-track error; here we simply use y_target.
    ec = y_target  
    k_soft = 3
    kd_yaw = 0.0159375
    kss =  120 / (2*10000)
    # For a local frame with car at (0,0), v_act is taken from odometry (which we have set to zero locally)
    # so for testing, you might assign a nominal value:
    if v_act == 0:
        v_act = 5.0  # example nominal speed
    
    # For testing, assume yaw_rate is zero.
    yaw_rate = 0.0
    try:
        steering_ang = ((heading_angle_ref - kss * pow(v_act, 2) * k) +
                        math.atan((0.003125 * ec) / (k_soft + v_act)) +
                        kd_yaw * (yaw_rate - (v_act * k)))
    except Exception as e:
        rospy.logwarn("Error computing steering angle: %s", e)
        v_des = 31.5
    
    # Publish control command.
    SA = AckermannDriveStamped()
    SA.drive.steering_angle = steering_ang
    SA.drive.speed = vel
    SA.drive.steering_angle_velocity = 0
    SA.drive.acceleration = 0
    SA.drive.jerk = 0
    robot_control_pub.publish(SA)
    rospy.loginfo("Published control: speed=%.2f, steering_angle=%.2f", vel, steering_ang)

                      


def imu_callback(data):
        global angvel,yaw
        angvel = data.angular_velocity.z
        orientation_list = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

 
def control_callback(Control):
        global v_act


def odom_callback(odom):
    global v_act, v_des, mini_final_speed, k, steering_ang
    # Extract the actual speed from odometry.
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    v_act = math.sqrt(vx**2 + vy**2)
    rospy.loginfo("Control odom_callback: Actual speed (v_act) = %.4f", v_act)

    # Force the car’s local pose to (0,0,0) for consistent local planning.
    local_x = 0.0
    local_y = 0.0
    local_yaw = 0.0
    rospy.loginfo("Control odom_callback: Using local car pose: pos=(%.2f, %.2f), yaw=%.2f",
                  local_x, local_y, local_yaw)

    # Compute desired speed (and clamp it as needed).
    if k == 0:
        v_des = 0.3
    else:
        v_des = math.sqrt(9.81 * 0.3 * (1/k))
    max_speed = 0.3
    v_des = min(v_des, max_speed)
    new_v_des = v_des - (20/100.0) * v_des
    rospy.loginfo("odom_callback: Raw computed v_des=%.4f, Clamped v_des=%.4f, Adjusted v_des=%.4f",
                  math.sqrt(9.81 * 0.6 * (1/k)) if k != 0 else 0.1, v_des, new_v_des)

    # Publish the actual speed for monitoring.
    from std_msgs.msg import Float64
    v_act_msg = Float64()
    v_act_msg.data = v_act
    pub2.publish(v_act_msg)

    # Compute final speed using the PID controller.
    final_speed = pid(v_act, v_des)
    rospy.loginfo("odom_callback: PID final speed = %.4f", final_speed)
    mini_final_speed = final_speed - 0.7 * final_speed
    rospy.loginfo("odom_callback: mini_final_speed = %.4f", mini_final_speed)

    # Publish control command using the stored steering angle from waypoints_callback.
    SA = AckermannDriveStamped()
    SA.drive.steering_angle = steering_ang  # Use the computed steering angle!
    SA.drive.speed = mini_final_speed
    SA.drive.steering_angle_velocity = 0
    SA.drive.acceleration = 0
    SA.drive.jerk = 0
    robot_control_pub.publish(SA)
    rospy.loginfo("odom_callback: Published control command with speed = %.4f, steering = %.4f", SA.drive.speed, SA.drive.steering_angle)



def stop_car(v_act,v_target,deceleration_time):
        current_speed = v_act
        acceleration = (current_speed - v_target) / deceleration_time
        while current_speed > v_target:
                #print("Current speed:", current_speed, "m/s")
                current_speed -= acceleration
        return current_speed


      

def listner():
        global robot_control_pub
        global pub1
        global pub2
        global waypoints_visual_pub
        rospy.init_node('pd_control',anonymous = True)
        # rospy.Subscriber('/visual/waypoints',MarkerArray,waypoints_callback)

        rospy.Subscriber('/sensor_imu_hector',Imu,imu_callback)
        rospy.Subscriber("/ground_truth/state_raw",Odometry,odom_callback)
        rospy.Subscriber("/robot_control/command",AckermannDriveStamped,control_callback)
        rospy.Subscriber('/waypoints', WaypointsArray, waypoints_callback)

        #rospy.Subscriber('/car_pose', Path,self.refrence_callback)
        robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        # waypoints_visual_pub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)
        pub1 = rospy.Publisher("/v_target", Float64, queue_size=1)
        pub2 = rospy.Publisher("/v_actual", Float64, queue_size=1)

if __name__ == "__main__":
        listner()
        rospy.spin()
