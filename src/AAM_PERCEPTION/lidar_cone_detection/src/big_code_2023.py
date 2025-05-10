#!/usr/bin/env python3
from fcntl import F_GETLEASE
from os import path
from re import U
from types import LambdaType
from unittest import skip
import math
import sys
from numpy.core.fromnumeric import shape
from numpy.lib.function_base import blackman
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
#mport ros_numpy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header, String, ColorRGBA
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
import numpy as np
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class TurningRadius:
    def __init__(self):
        # Intializing node
        rospy.init_node("Tuning_radius", anonymous=True)
        # Subscribing to req. nodes
        rospy.Subscriber("/sensor_imu_hector", Imu, self.imu_callback)
        rospy.Subscriber("/aamfsd/joint_states", JointState, self.wheel_callback)   # for steering and velocity
        # rospy.Subscriber("/robot_control/command", AckermannDriveStamped, self.control_callback)
        rospy.Subscriber("/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/acc", Float32MultiArray, self.acc_callback)
        rospy.Subscriber("/optical_flow", Float32MultiArray, self.optical_flow_callback)

        # Velocities Publisher
        self.pub = rospy.Publisher('vel_ekf', Float32MultiArray, queue_size=10)

        # Variables Intialization

        # These are the speeds of each wheel , which are calculated at the callback
        self.Front_Right_vel = 0
        self.Front_Left_vel = 0
        self.Back_Right_vel = 0     # M/SEC
        self.Back_Left_vel = 0

        self.Front_Left_steering = 0  # steering in rad
        self.Front_Right_steering = 0

        # Steering angle
        # self.ack=0

        # Rdii
        self.Front_Right_Radius = 0
        self.Front_Left_Radius = 0   # Meter
        self.Back_Right_Radius = 0
        self.Back_Left_Radius = 0
        self.Center_Radius = 0
        self.Turn_x = 0
        self.Turn_y = 1
        # Shifted Velocities
        self.Shifted_Front_Right = 0
        self.Shifted_Front_Left = 0  # m/sec
        self.Shifted_Back_Right = 0
        self.Shifted_Back_Left = 0

        # Angles
        self.Right_Angle = 0
        self.Left_Angle = 0
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.heading = 0

        # Accelerations
        self.ax = 0
        self.ay = 0

        # Velocities
        self.Vx = 0
        self.Vy = 0
        self.Vel_flow = np.array([0, 0])
        self.old_Vx = 0
        self.old_Vy = 0

        # Angular Velocity
        self.angular_vel = 0
        # self.yaw_rate = 0

        # Time
        self.start_time = rospy.get_rostime().to_sec()
        self.dt = 0

        # wheel initialization
        # object of every wheel , to use in their ekfs

        self.FR = Wheel(self.Shifted_Front_Right, self.Right_Angle)
        self.FL = Wheel(self.Shifted_Front_Left, self.Left_Angle)
        self.BR = Wheel(self.Shifted_Back_Right, 0)
        self.BL = Wheel(self.Shifted_Back_Left, 0)

        # objects for estimated left and right wheels
        self.RW = Wheel(0, 0)
        self.LW = Wheel(0, 0)

        # object for final estimated wheel
        self.hotWheel = Wheel(0, 0)

        # object for fusion with optical flow
        self.finalWheel = Wheel(0, 0)

        # Steering Sensor Readings
        # def control_callback(self,control_msg):
        #    self.ack = control_msg.drive.steering_angle * self.dt
        #    print("this is the steering from the robot cont ",self.ack*180/math.pi)

        # Imu Sensor Readings
    def imu_callback(self, msg):
        self.angular_vel = msg.angular_velocity.z
        explicit_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

    # Encoders Readings
    def wheel_callback(self, msg):
        # needs to be multiled by the wheel radius   dia = 0.505
        self.Front_Left_vel = msg.velocity[0] * 0.2525
        self.Back_Left_vel = msg.velocity[2] * 0.2525
        self.Front_Right_vel = msg.velocity[5] * 0.2525
        self.Back_Right_vel = msg.velocity[7] * 0.2525

        self.Front_Left_steering = msg.position[4]
        self.Front_Right_steering = msg.position[9]
        self.heading = (self.Front_Left_steering + self.Front_Right_steering) / 2.0

    # Ground Truth / GSS
    def odom_callback(self, odom_msg):
        self.Vx = odom_msg.twist.twist.linear.x
        self.Vy = odom_msg.twist.twist.linear.y

    # Accelerations
    def acc_callback(self, acc_msg):
        self.ax = acc_msg.data[0]
        self.ay = acc_msg.data[1]

    def optical_flow_callback(self, optical_flow_msg):
        self.Vel_flow = [optical_flow_msg.data[0], optical_flow_msg.data[1]]
        print(self.Vel_flow)

class Wheel:
    def __init__(self, Wheel_Velocity, Angle):
        self.Wheel_Velocity = Wheel_Velocity

        # Steering Angle
        self.Angle = Angle

        self.Z = np.zeros((2, 1))

        # Wheel Covariance
        # reinitialize it from the datasheet
        self.Wheel_Cov = np.array([[0.5, 0, 0],
                                   [0, 0.5, 0],
                                   [0, 0, 0.1]])

        # state : Vx , Vy , yaw
        self.xEst = np.zeros((3, 1))
        self.pEst = np.eye(3)

        # Prev
        self.xEstHist = []

# Global Velocities of wheels
def calc_global(TR, W):

    # calculating global angle for wheels
    '''
    if TR.Front_Left_steering<0:
        W.Angle = -(math.pi/2 - np.arctan(TR.Turn_x/TR.Turn_y) )
    else:
        W.Angle = math.pi/2 - np.arctan(TR.Turn_x/TR.Turn_y)
    '''

    # Calculating llongitudinal and lateral velocities (components of wheel velocity)
    long_vel = W.Wheel_Velocity * math.cos(W.Angle)
    lat_vel = W.Wheel_Velocity * math.sin(W.Angle)

    # angle for latteral velocity
    if TR.Front_Left_steering < 0:
        yaw_on_y = TR.yaw - math.pi/2  # right
    else:
        yaw_on_y = TR.yaw + math.pi/2  # left

    # Calculating velocity on X and Y axis (components of longitudinal and latteral velocities added)
    vel_x = long_vel * math.cos(TR.yaw) + lat_vel * math.sin(yaw_on_y)
    vel_y = long_vel * math.sin(TR.yaw) + lat_vel * math.cos(yaw_on_y)

    # wheel velocity
    W.Z = np.array([[vel_x],
                    [vel_y]])

    # print("vel_x",vel_x)
    # print("vel_y",vel_y)


# Calculating Radii
def radius(TR):
    wheelBase = 1.582  # in m
    trackWidth = 1.180  # rear trackwidth in m

    # we already have the left and right steering
    '''

    #steering_ack=TR.ack

    #Get left and right steering angles
    #steering_left=np.arctan(wheelBase*math.tan(steering_ack) / (wheelBase + 0.5*trackWidth*math.tan(steering_ack)))
    #steering_right=np.arctan(wheelBase*math.tan(steering_ack) / (wheelBase - 0.5*trackWidth*math.tan(steering_ack)))

    #  -ve  -> right, +ve -> left 


    try:
        if(TR.ack < 0):
            Angle_Inner = steering_right 
            per_ack = np.arctan(Angle_Inner/TR.ack)   
            Angle_outter = Angle_Inner - per_ack*(Angle_Inner - TR.ack)
            steering_left = Angle_outter

        else:
            Angle_Inner = steering_left
            per_ack = np.arctan(Angle_Inner/TR.ack)   
            Angle_outter = Angle_Inner - per_ack*(Angle_Inner - TR.ack)
            steering_right = Angle_outter
    except:        #already handle in the condition below
        pass
    if np.isnan(steering_right) or np.isnan(steering_left) :
        steering_right = 0
        steering_left = 0

    TR.Right_Angle = steering_right
    TR.Left_Angle = steering_left

    print("right ",TR.Right_Angle)
    print("left ",TR.Left_Angle)

    TR.FR.Angle = TR.Right_Angle
    TR.FL.Angle = TR.Left_Angle
    '''

    # Left and right radii calculations

    steering_left = TR.Front_Left_steering
    steering_right = TR.Front_Right_steering

    if steering_left != 0:
        Radius_Left = abs(wheelBase/math.tan(steering_left))
        Radius_right = abs(wheelBase/math.tan(steering_right))   # needs to  use the trig..
        # print(" steering angle left ",steering_left,"  radius Left = ",Radius_Left ," steering angle right ",steering_right," radius right ",Radius_right)
    else:
        # set radii to max int ?
        Radius_Left = np.iinfo(np.intc).max
        Radius_right = np.iinfo(np.intc).max

    TR.Back_Left_Radius = Radius_Left
    TR.Back_Right_Radius = Radius_right

    if(TR.Front_Left_steering < 0):  # +ve steering left   -ve steering right
        Radius_Center = Radius_right + 0.5 * trackWidth
    else:
        Radius_Center = Radius_Left + 0.5 * wheelBase  # was-> radius_right -0.5 *trackwidth

    # Calculating Turning radii
    Turning_Radius_Center = math.sqrt(math.pow(Radius_Center, 2) + math.pow(wheelBase * 0.5, 2))
    Turning_Radius_Left = math.sqrt(math.pow(wheelBase, 2) + math.pow(Radius_Left, 2))
    Turning_Radius_Right = math.sqrt(math.pow(wheelBase, 2) + math.pow(Radius_right, 2))

    TR.Center_Radius = Turning_Radius_Center
    TR.Front_Left_Radius = Turning_Radius_Left
    TR.Front_Right_Radius = Turning_Radius_Right
    TR.Turn_x = TR.Center_Radius
    TR.Turn_y = 0.5 * wheelBase

    TR.FR.Angle = steering_right
    TR.FL.Angle = steering_left
    # print ("Turning_Radius_Center",Turning_Radius_Center)
    # print ("Turning_Radius_left",Turning_Radius_Left)
    # print ("Turning_Radius_right",Turning_Radius_Right)


# Shifts wheels velocities to the center
def shifting(TR):
    TR.Shifted_Front_Right = (TR.Front_Right_vel / TR.Front_Right_Radius) * TR.Center_Radius
    TR.Shifted_Front_Left = (TR.Front_Left_vel / TR.Front_Left_Radius) * TR.Center_Radius
    TR.Shifted_Back_Right = (TR.Back_Right_vel / TR.Back_Right_Radius) * TR.Center_Radius
    TR.Shifted_Back_Left = (TR.Back_Left_vel / TR.Back_Left_Radius) * TR.Center_Radius

    TR.FR.Wheel_Velocity = TR.Shifted_Front_Right
    TR.FL.Wheel_Velocity = TR.Shifted_Front_Left
    TR.BR.Wheel_Velocity = TR.Shifted_Back_Right
    TR.BL.Wheel_Velocity = TR.Shifted_Back_Left


# Ensures that angle doesn't exceed 2 pi (Normalization)
# def pi_2_pi(angle):
#     return (angle + math.pi) % (2 * math.pi) - math.pi

# Predicts the state XEst -> (vel_x, vel_y, yaw)
def prediction(TR, W):

    # states calculations
    x = W.xEst[0] + TR.dt * TR.ax      # todo  : make w.xEst last vel xEst
    y = W.xEst[1] + TR.dt * TR.ay

    # ??
    # TR.yaw += TR.dt * TR.angular_vel
    # TR.yaw = pi_2_pi(TR.yaw)

    # Set estimated values
    W.xEst[0] = x
    W.xEst[1] = y
    W.xEst[2] = W.Angle


# Predicts Jacobian F matrix of front wheels
def prediction_Jacobian_Front(TR, W):
    F = np.array([[1, 0, -W.Wheel_Velocity * math.sin(W.Angle)],
                  [0, 1, W.Wheel_Velocity * math.cos(W.Angle)],
                  [0, 0, 1]])
    return F


# Updates xEst and pEst and calculates kalman gain
def update(TR, W):
    calc_global(TR, W)  # calculate global velocity
    C = np.array([[1, 0, 0],
                  [0, 1, 0]])

    y = W.Z - np.dot(C, W.xEst)
    S = np.dot(C, np.dot(W.pEst, C.T)) + np.array([[0.4, 0], [0, 0.4]])
    K = np.dot(W.pEst, np.dot(C.T, np.linalg.inv(S)))
    W.xEst = W.xEst + np.dot(K, y)
    W.pEst = np.dot((np.eye(3) - np.dot(K, C)), W.pEst)


# Extended Kalman filter for front wheels
def EKF_Front(TR, W):
    prediction(TR, W)
    F = prediction_Jacobian_Front(TR, W)  # prediction jacobian
    # Q
    p = np.dot(F, np.dot(W.pEst, F.T)) + np.array([[0.05, 0, 0],
                                                    [0, 0.05, 0],
                                                    [0, 0, 1]])
    W.pEst = p
    update(TR, W)


# Kalman filter for back wheels
def KF_Back(TR, W):
    prediction(TR, W)
    F = np.eye(3)  # prediction jacobian
    # Q
    p = np.dot(F, np.dot(W.pEst, F.T)) + np.array([[0.05, 0, 0],
                                                    [0, 0.05, 0],
                                                    [0, 0, 1]])
    W.pEst = p
    update(TR, W)


def KF_FRONT_Back(TR, FW, BW, W):
    # YA3NI MA3ANPREDICTISH
    update2(TR, FW, BW, W)


def update2(TR, FW, BW, W):
    # calc_global(TR,W) #calculate global velocity
    C = np.array([[1, 0, 0],
                  [0, 1, 0]])

    y = FW.xEst[0:2] - BW.xEst[0:2]
    S = np.dot(C, np.dot(BW.pEst, C.T)) + np.dot(C, np.dot(FW.pEst, C.T))
    K = np.dot(BW.pEst, np.dot(C.T, np.linalg.inv(S)))
    W.xEst = BW.xEst + np.dot(K, y)
    W.pEst = np.dot((np.eye(3) - np.dot(K, C)), BW.pEst)


def KF_FINAL(TR, FW, x, W):
    # YA3NI MA3ANPREDICTISH
    update3(TR, FW, x, W)


def update3(TR, FW, x, W):
    # calc_global(TR,W) #calculate global velocity
    C = np.array([[1, 0, 0],
                  [0, 1, 0]])

    y = x - FW.xEst[0:2]
    S = np.dot(C, np.dot(FW.pEst, C.T)) + np.diag([0.1063958, 0.1063958])
    K = np.dot(FW.pEst, np.dot(C.T, np.linalg.inv(S)))
    W.xEst = FW.xEst + np.dot(K, y)
    W.pEst = np.dot((np.eye(3) - np.dot(K, C)), FW.pEst)


# Calls all req functions
def mainfun(TR):
    # determine the time tick
    # TR.dt = rospy.get_rostime().to_sec() - TR.start_time
    # TR.start_time = rospy.get_rostime().to_sec()
    radius(TR)  # calculates the turning radius at the 2 front wheels and the angle of each wheel
    shifting(TR)  # calculate the velocity of each wheel after shifting to the center of gravity

    EKF_Front(TR, TR.FR)
    EKF_Front(TR, TR.FL)
    KF_Back(TR, TR.BR)
    KF_Back(TR, TR.BL)
    KF_FRONT_Back(TR, TR.FR, TR.BR, TR.RW)
    KF_FRONT_Back(TR, TR.FL, TR.BL, TR.LW)
    KF_FRONT_Back(TR, TR.RW, TR.LW, TR.hotWheel)
    # KF_FINAL(TR, TR.hotWheel, TR.Vel_flow, TR.finalWheel)
    # Publishing velocities
    arr = Float32MultiArray()
    V = [TR.hotWheel.xEst[0], TR.hotWheel.xEst[1],TR.heading,TR.yaw]
    #print("velocities", V)
    arr.data = V
    TR.pub.publish(arr)


if __name__ == '__main__':
    try:
        # creates turning radius object
        TR = TurningRadius()

        while not rospy.is_shutdown():
            mainfun(TR)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
