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
from ma_rrt_path_plan.msg import WaypointsArray
import osqp
from scipy import sparse

error = 0
ref_waypoints = []
steering_ang = 0.0
last_waypoint_vel = 0.0



class VehicleState:
    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.psi = 0.0
        self.v = 0.0

    def update_from_odom(self, odom_msg):
        # We only care about speed for now; we force local pose to (0,0,0)
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        self.v = math.hypot(vx, vy)

    def update_from_imu(self, imu_msg):
        ori = [
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ]
        (_, _, self.psi) = euler_from_quaternion(ori)


vehicle_state = VehicleState()


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

        


'''
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
'''
def waypoints_callback(wp_msg):
    """
    Updated waypoints_callback with a guard so that we only print ref_waypoints[0..2]
    when there are at least three points in the list.
    """
    global ref_waypoints, steering_ang, k, last_waypoint_vel

    # 1) Clear and refill ref_waypoints with (x, y) from wp_msg.waypoints
    ref_waypoints.clear()
    for waypoint in wp_msg.waypoints:
        ref_waypoints.append((waypoint.x, waypoint.y))
    rospy.loginfo("+++ waypoints_callback triggered, %d points", len(ref_waypoints))

    # Only print ref_waypoints[0..2] if there are at least 3 points
    if len(ref_waypoints) >= 3:
        rospy.loginfo(
            "   ref_waypoints[0..2] = %s, %s, %s",
            ref_waypoints[0], ref_waypoints[1], ref_waypoints[2]
        )

    # If no waypoints, bail out
    if not ref_waypoints:
        rospy.logwarn("waypoints_callback: No waypoints available")
        return

    # 2) Extract x_points and y_points for curvature, target finding, etc.
    x_points = [pt[0] for pt in ref_waypoints]
    y_points = [pt[1] for pt in ref_waypoints]

    size_array = len(x_points)
    x_target = x_points[0] + 0.7675
    y_target = y_points[0]

    # 3) Find the first waypoint at least 3.3 m away from (0,0)
    found = False
    skip_count = 0
    i = 0
    while not found and i < size_array:
        dist = math.hypot(x_points[i], y_points[i])
        if dist >= 3.3:
            x_target = x_points[i] + 0.7675
            y_target = y_points[i]
            found = True
        else:
            i += 1
            skip_count += 1
            if skip_count > 3:
                break

    # 4) If none ≥ 3.3 m was found, fallback to index 1 (if it exists)
    if not found and size_array > 1:
        x_target = x_points[1] + 0.7675
        y_target = y_points[1]

    rospy.loginfo(
        "waypoints_callback: Target point chosen = (%.2f, %.2f)",
        x_target, y_target
    )

    # 5) Compute reference heading angle to that target point
    heading_angle_ref = math.atan2(y_target, x_target)
    rospy.loginfo(
        "waypoints_callback: Heading angle ref = %.3f rad",
        heading_angle_ref
    )

    # 6) Compute curvature k over all waypoints
    try:
        k = calculate_curvature(x_points, y_points)
    except Exception as e:
        rospy.logwarn("waypoints_callback: Error calculating curvature: %s", e)
        k = 1.0
    rospy.loginfo("waypoints_callback: Curvature k = %.6f", k)

    # 7) Compute vel from curvature and clamp if desired
    vel = Another_speed(k)
    # Example clamp (uncomment if you want to enforce max speed):
    vel = min(vel, 0.5)
    last_waypoint_vel = vel
    rospy.loginfo("waypoints_callback: Computed vel = %.3f", vel)

    # 8) Compute cross‐track error (assuming car at origin)
    ec = y_target
    k_soft = 3.0
    kd_yaw = 0.0159375
    kss = 120.0 / (2 * 10000)

    # 9) If v_act hasn’t been set yet, assume a nominal speed (5 m/s) for steering calc
    if vehicle_state.v == 0.0:
        v_act_local = 5.0
    else:
        v_act_local = vehicle_state.v

    # 10) No yaw rate from IMU here
    yaw_rate_local = 0.0

    # 11) Compute the “hybrid” steering angle
    try:
        steering_ang = (
            (heading_angle_ref - kss * (v_act_local ** 2) * k)
            + math.atan((0.3 * ec) / (k_soft + v_act_local))
            + kd_yaw * (yaw_rate_local - (v_act_local * k))
        )
    except Exception as e:
        rospy.logwarn("waypoints_callback: Error computing steering angle: %s", e)
        steering_ang = 0.0

    rospy.loginfo(
        "waypoints_callback: Computed steering_ang = %.3f rad",
        steering_ang
    )

    # 12) Return without publishing; control_timer_callback will handle that
    return

'''
def imu_callback(data):
        global angvel,yaw
        angvel = data.angular_velocity.z
        orientation_list = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
'''
def imu_callback(data):
    """
    Updated imu_callback that preserves the original data format:
    - Reads angular velocity from data.angular_velocity.z
    - Converts quaternion (data.orientation) to roll, pitch, yaw
    - Stores yaw into vehicle_state.psi and stores angvel globally
    """
    global angvel, vehicle_state

    # 1) Extract angular velocity (Z axis)
    angvel = data.angular_velocity.z

    # 2) Convert quaternion to (roll, pitch, yaw)
    orientation_list = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ]
    _, _, psi = euler_from_quaternion(orientation_list)

    # 3) Store yaw in vehicle_state.psi (so the rest of the code can use it)
    vehicle_state.psi = psi

    rospy.loginfo("imu_callback: yaw=%.3f rad, angvel=%.3f rad/s", psi, angvel)

 
def control_callback(Control):
        global v_act

'''
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
'''
odom_counter = 0
def odom_callback(odom_msg):
    """
    Updated odom_callback:
    - Reads v_act from odom_msg.twist.twist.linear
    - Logs the actual speed
    - Publishes v_actual for monitoring
    - Does NOT compute PID or publish any drive command
    """
    global odom_counter, vehicle_state, pub2
    odom_counter += 1

    # 1) Extract actual speed from odometry
    vx = odom_msg.twist.twist.linear.x
    vy = odom_msg.twist.twist.linear.y
    vehicle_state.v = math.hypot(vx, vy)
   # rospy.loginfo("odom_callback: Actual speed (v_act) = %.4f", vehicle_state.v)
    
    if odom_counter % 50 == 0:
        rospy.loginfo("odom_callback: v_act = %.4f", vehicle_state.v)
        odom_counter = 0
    
    # 2) Force local pose to (0,0,0) for a car-centric reference frame
    local_x = 0.0
    local_y = 0.0
    local_yaw = 0.0
    #rospy.loginfo(
    #    "odom_callback: Using forced local pose → pos=(%.2f, %.2f), yaw=%.2f",
    #    local_x, local_y, local_yaw
    #)

    # 3) Publish the actual speed on /v_actual (Float64) for telemetry
    v_act_msg = Float64()
    v_act_msg.data = vehicle_state.v
    pub2.publish(v_act_msg)

    # 4) Return without publishing any AckermannDriveStamped here.
    return


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
        
        ##new part 
        rospy.Timer(rospy.Duration(0.1), control_timer_callback)


def control_timer_callback(event):
    """
    Periodic control loop (e.g. at 10 Hz) that:
      1. Checks if any waypoints have been received.
      2. Uses the last computed steering_ang (from waypoints_callback).
      3. Uses last_waypoint_vel (from waypoints_callback) as desired speed.
      4. Runs the PID to compute a final speed command.
      5. Publishes a single AckermannDriveStamped with steering and speed.
    """
    global ref_waypoints, steering_ang, last_waypoint_vel, vehicle_state, robot_control_pub

    # 1) Verify that at least one waypoint has been processed
    if not ref_waypoints:
        rospy.logwarn("control_timer_callback: No waypoints available, skipping control.")
        return

    # 2) Desired speed is the velocity computed in waypoints_callback
    v_des = last_waypoint_vel

    # 3) Current actual speed from vehicle_state (updated in odom_callback)
    v_act_local = vehicle_state.v

    # 4) Compute PID output and scale down by 0.7
    final_speed = pid(v_act_local, v_des)
    mini_final_speed = final_speed * (1.0 - 0.7)

    rospy.loginfo(
        "control_timer_callback: v_act=%.4f, v_des=%.4f, PID output=%.4f, mini_final=%.4f",
        v_act_local, v_des, final_speed, mini_final_speed
    )

    # 5) Publish the drive command
    cmd = AckermannDriveStamped()
    cmd.drive.steering_angle = steering_ang
    cmd.drive.speed = mini_final_speed
    cmd.drive.steering_angle_velocity = 0.0
    cmd.drive.acceleration = 0.0
    cmd.drive.jerk = 0.0
    robot_control_pub.publish(cmd)

    rospy.loginfo(
        "control_timer_callback: Published command → speed=%.4f, steering=%.4f",
        mini_final_speed, steering_ang
    )



if __name__ == "__main__":
        listner()
        rospy.spin()