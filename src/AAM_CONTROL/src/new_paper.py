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
from collections import deque

SMOOTH_N = 5                     # number of waypoint sets to average
waypoint_history = deque(maxlen=SMOOTH_N)
PREVIEW_N = 5                    # how many smoothed points to preview
preview_weights = [i+1 for i in range(PREVIEW_N)]
CONTROL_DT = 0.1                 # your 10 Hz timer dt
prev_v_des = 0.0                 # for velocity feed-forward
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

def smooth_waypoints(raw_pts):
    """
    Maintain a short history of raw waypoint lists and
    return an element-wise average.
    """
    waypoint_history.append(raw_pts)
    smoothed = []
    for i in range(len(raw_pts)):
        xs = [h[i][0] for h in waypoint_history if len(h)>i]
        ys = [h[i][1] for h in waypoint_history if len(h)>i]
        smoothed.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return smoothed

def waypoints_callback(wp_msg):
    """
    waypoints_callback with:
      - dynamic speed clamp based on curvature
      - stronger cross-track gain for tight turns
      - non-zero yaw feedback from IMU
      - full component logging
    """
    global ref_waypoints, steering_ang, k, last_waypoint_vel, vehicle_state

    # 1) Pull raw waypoints
    raw = [(wp.x, wp.y) for wp in wp_msg.waypoints]
    n_raw = len(raw)
    rospy.loginfo("+++ waypoints_callback: received %d raw points", n_raw)
    if n_raw == 0:
        ref_waypoints.clear()
        return


    # 2) Smooth them
    ref_waypoints[:] = smooth_waypoints(raw)
    n = len(ref_waypoints)
    rospy.loginfo("    smoothed to %d points", n)
    if n >= 3:
        rospy.loginfo("    smoothed[0..2] = %s, %s, %s",
                      ref_waypoints[0], ref_waypoints[1], ref_waypoints[2])


    M = min(PREVIEW_N, n)
    headings = [math.atan2(ref_waypoints[i][1], ref_waypoints[i][0])
                for i in range(M)]
    wsum = sum(preview_weights[:M])
    heading_ref = sum(w*h for w,h in zip(preview_weights, headings)) / wsum
    rospy.loginfo("    preview headings[0..%d]=%s → heading_ref=%.3f",
                  M-1, [f"{h:.3f}" for h in headings], heading_ref)

    # 3) Split into x/y lists
    x_pts = [p[0] for p in ref_waypoints]
    y_pts = [p[1] for p in ref_waypoints]

    # 4) Pick a target point at ≥3.3m (fallback to index 1)
    x_t, y_t = x_pts[0] + 0.7675, y_pts[0]
    found, skips, i = False, 0, 0
    while not found and i < n and skips < 4:
        if math.hypot(x_pts[i], y_pts[i]) >= 3.3:
            x_t, y_t = x_pts[i] + 0.7675, y_pts[i]
            found = True
        else:
            i += 1; skips += 1
    if not found and n > 1:
        x_t, y_t = x_pts[1] + 0.7675, y_pts[1]
    rospy.loginfo("   target = (%.2f, %.2f)", x_t, y_t)

    # 5) Heading reference
    heading_ref = math.atan2(y_t, x_t)
    rospy.loginfo("   heading_ref = %.3f rad", heading_ref)

    # 6) Curvature + dynamic speed clamp
    try:
        k = calculate_curvature(x_pts, y_pts)
    except Exception as e:
        rospy.logwarn("   curvature error: %s", e)
        k = 1.0
    rospy.loginfo("   curvature k = %.4f", k)

    # dynamic max speed: faster on straights, slower on bends
    if k < 0.02:
        max_speed = 0.6
    elif k < 0.05:
        max_speed = 0.4
    else:
        max_speed = 0.25

    vel = min(Another_speed(k), max_speed)
    last_waypoint_vel = vel
    rospy.loginfo("   v_des(clamped) = %.3f (→ max %.2f)", vel, max_speed)

    # 7) Steering components
    ec = y_t                        # cross-track error
    # Gains
    k_soft = 3.0
    # boost cross-track gain for tight turns
    if k > 0.1:
        gain_cte = 2.0
    else:
        gain_cte = 0.5
    # reduced feed-forward weight
    kss = 0.005
    # stronger yaw-feedback
    kd_yaw = 0.1

    # 8) Vehicle state
    v_act = vehicle_state.v if vehicle_state.v > 0 else 0.1
    yaw_rate = angvel if 'angvel' in globals() else 0.0

    # 9) Compute terms
    ff = -kss * (v_act**2) * k
    xt = math.atan((gain_cte * ec) / (k_soft + v_act))
    yf = kd_yaw * (yaw_rate - (v_act * k))

    rospy.loginfo("   terms → ff=%.3f, xt=%.3f, yaw_fb=%.3f",
                  ff, xt, yf)

    # 10) Final steering
    steering_ang = heading_ref + ff + xt + yf
    rospy.loginfo("   steering_ang = %.3f rad", steering_ang)

    # done: control_timer_callback will publish (steering_ang, last_waypoint_vel)
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
    10 Hz loop that:
      • Feeds in the Stage 3 velocity FF + PID
      • Computes a proper look‐ahead heading
      • Adds your Stage 2 cross-track & curvature FF & yaw‐FB
      • Publishes a single AckermannDriveStamped
    """
    global prev_v_des, ref_waypoints, last_waypoint_vel
    global vehicle_state, robot_control_pub

    if not ref_waypoints:
        rospy.logwarn("control_timer: no waypoints, skipping")
        return

    # — Velocity feed-forward + PID speed — 
    v_des = last_waypoint_vel
    v_act = vehicle_state.v

    a_des = (v_des - prev_v_des) / CONTROL_DT
    ff_speed = v_act + a_des * CONTROL_DT
    prev_v_des = v_des

    u_pid = pid(v_act, v_des)
    speed_cmd = ff_speed + u_pid * 0.7

    # — Preview heading — 
    N = min(PREVIEW_N, len(ref_waypoints))
    headings = [math.atan2(ref_waypoints[i][1], ref_waypoints[i][0])
                for i in range(N)]
    w = preview_weights[:N]
    heading_ref = sum(w_i * h for w_i, h in zip(w, headings)) / sum(w)

    # — Stage 2 cross-track & curvature FF & yaw-FB — 
    # (recompute or reuse your globals from waypoints_callback)
    ec = ref_waypoints[0][1]   # y_target
    k_soft = 3.0
    # boost cross-track gain on tight turns
    gain_cte = 2.0 if k > 0.1 else 0.5
    kss = 0.005
    kd_yaw = 0.1
    v_local = max(v_act, 0.1)
    yaw_rate = globals().get('angvel', 0.0)

    ff_term = -kss * (v_local**2) * k
    xt_term = math.atan((gain_cte * ec) / (k_soft + v_local))
    yaw_fb = kd_yaw * (yaw_rate - (v_local * k))

    final_steering = heading_ref + ff_term + xt_term + yaw_fb

    # — Publish — 
    cmd = AckermannDriveStamped()
    cmd.drive.steering_angle = final_steering
    cmd.drive.speed = speed_cmd
    cmd.drive.steering_angle_velocity = 0.0
    cmd.drive.acceleration = 0.0
    cmd.drive.jerk = 0.0
    robot_control_pub.publish(cmd)

    rospy.loginfo(
      "control_timer → speed=%.3f, steer=%.3f (preview=%.3f, ff=%.3f, xt=%.3f, yf=%.3f)",
      speed_cmd, final_steering,
      heading_ref, ff_term, xt_term, yaw_fb
    )



if __name__ == "__main__":
        listner()
        rospy.spin()
