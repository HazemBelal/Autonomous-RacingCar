#!/usr/bin/env python

from fcntl import F_GETLEASE
from os import path
from re import U
from types import LambdaType
from unittest import skip
import numpy as np
import math
from numpy.core.fromnumeric import shape
from numpy.lib.function_base import blackman
import rospy
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import Map
from aam_common_msgs.msg import ConeDetections
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
import ros_numpy
import time 
from nav_msgs.msg import Odometry
import tf 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header, String, ColorRGBA
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion







def subscribers_and_publishers():
   
     global car_pose_pub,global_map_pub,lm_x,lm_y,lm_c,control_steering_angle,control_velocity,angular_vel,Vx,Vy,odomx,odomy, roll , pitch , yaw , current_yaw , RFID

     rospy.init_node("ekf_yasser", anonymous = True)

     rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,cones_callback)
     rospy.Subscriber("/robot_control/command",AckermannDriveStamped,control_callback)
     rospy.Subscriber("/sensor_imu_hector",Imu,imu_callback)
     rospy.Subscriber("/ground_truth/state",Odometry,odom_callback)
     car_pose_pub = rospy.Publisher("/yasser_pose", Path, queue_size=0)
     global_map_pub = rospy.Publisher("/yasser_map",MarkerArray, queue_size=0)
     lm_x = []
     lm_y = []
     lm_c = []
     control_velocity = 0
     control_steering_angle =0
     angular_vel = 0
     RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])




def cones_callback(cones_msgs):
       global lm_x , lm_y , lm_c , RFID
       lm_c = []
       for cone in cones_msgs.cone_detections:
         RFID=np.hstack([cone.position.x,cone.position.y])
         lm_c.append(cone.color.data)
        

    
def control_callback(control_msg):
      global car_pose_pub,global_map_pub,lm_x,lm_y,lm_c,control_steering_angle,control_velocity,angular_vel,Vx,Vy,odomx,odomy, roll , pitch , yaw , current_yaw

      control_steering_angle = control_msg.drive.steering_angle
      control_velocity = control_msg.drive.speed




def imu_callback(imu_msg):
     global anglular_vel
     angular_vel = imu_msg.angular_velocity.z



def odom_callback(odom_msg):
    global car_pose_pub,global_map_pub,lm_x,lm_y,lm_c,control_steering_angle,control_velocity,angular_vel,Vx,Vy,odomx,odomy, roll , pitch , yaw , current_yaw , pose
    pose = PoseStamped()
    pose.header = odom_msg.header
    Vx = odom_msg.twist.twist.linear.x
    Vy = odom_msg.twist.twist.linear.y
    odomx = odom_msg.pose.pose.position.x
    odomy = odom_msg.pose.pose.position.y 
    odom_orientation = odom_msg.pose.pose.orientation
    orientation_list = [odom_orientation.x, odom_orientation.y, odom_orientation.z,odom_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_yaw = yaw
     


# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2 # Change in covariance

#  Simulation parameter
Qsim = np.diag([0.2, np.deg2rad(1.0)])**2  # Sensor Noise
Rsim = np.diag([1.0, np.deg2rad(10.0)])**2 # Process Noise

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

def ekf_slam(xEst, PEst, u, z):
    """
    Performs an iteration of EKF SLAM from the available information.

    :param xEst: the belief in last position
    :param PEst: the uncertainty in last position
    :param u:    the control function applied to the last position
    :param z:    measurements at this step
    :returns:    the next estimated position and associated covariance
    """
    S = STATE_SIZE

    # Predict
    xEst, PEst, G, Fx = predict(xEst, PEst, u)
    initP = np.eye(2)

    # Update
    xEst, PEst = update(xEst, PEst, u, z, initP)

    return xEst, PEst

def predict(xEst, PEst, u):
    """
    Performs the prediction step of EKF SLAM

    :param xEst: nx1 state vector
    :param PEst: nxn covariance matrix
    :param u:    2x1 control vector
    :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
    """
    S = STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], u)
    xEst[0:S] = motion_model(xEst[0:S], u)
    # Fx is an an identity matrix of size (STATE_SIZE)
    # sigma = G*sigma*G.T + Noise
    #PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx
    PEst[0:S, 0:S] = np.dot( G.T , np.dot(PEst[0:S, 0:S] , G )) + np.dot( Fx.T , np.dot(Cx , Fx) )

    return xEst, PEst, G, Fx

def motion_model(x, u):
    """
    Computes the motion model based on current state and input function.

    :param x: 3x1 pose estimation
    :param u: 2x1 control input [v; w]
    :returns: the resulting state after the control function is applied
    """
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = np.dot(F,x) + np.dot(B , u)
    return x

def update(xEst, PEst, u, z, initP):
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param u:     2x1 the control function
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    for iz in range(len(z[:, 0])):  # for each observation
        minid = search_correspond_LM_ID(xEst, PEst, z[iz, 0:2]) # associate to a known landmark

        nLM = calc_n_LM(xEst) # number of landmarks we currently know about

        if minid == nLM: # Landmark is a NEW landmark
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug

        lm = get_LM_Pos_from_state(xEst, minid)
        print(lm," this is a cone location")
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

        K = np.dot(np.dot(PEst , H.T) , np.linalg.inv(S)) # Calculate Kalman Gain
        xEst = xEst + np.dot(K , y)
        PEst = np.dot((np.eye(len(xEst)) - np.dot(K , H)) , PEst)

    xEst[2] = pi_2_pi(xEst[2])
    return xEst, PEst

def calc_innovation(lm, xEst, PEst, z, LMid):
    """
    Calculates the innovation based on expected position and landmark position

    :param lm:   landmark position
    :param xEst: estimated position/state
    :param PEst: estimated covariance
    :param z:    read measurements
    :param LMid: landmark id
    :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
    """
    delta = lm - xEst[0:2]
    q = np.dot(delta.T , delta)[0, 0]
    zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
    # zp is the expected measurement based on xEst and the expected landmark position

    y = (z - zp).T # y = innovation
    y[1] = pi_2_pi(y[1])

    H = jacobH(q, delta, xEst, LMid + 1)
    S = np.dot(np.dot(H , PEst) , H.T) + Cx[0:2, 0:2]

    return y, S, H

def jacobH(q, delta, x, i):
    """
    Calculates the jacobian of the measurement function

    :param q:     the range from the system pose to the landmark
    :param delta: the difference between a landmark position and the estimated system position
    :param x:     the state, including the estimated system position
    :param i:     landmark id + 1
    :returns:     the jacobian H
    """
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = np.dot(G , F)

    return H

def observation(xTrue, xd, u, RFID):
    """
    :param xTrue: the true pose of the system
    :param xd:    the current noisy estimate of the system
    :param u:     the current control input
    :param RFID:  the true position of the landmarks

    :returns:     Computes the true position, observations, dead reckoning (noisy) position,
                  and noisy control function
    """
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])): # Test all beacons, only add the ones we can see (within MAX_RANGE)

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.array([dn, anglen, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * Rsim[0, 0],
        u[1, 0] + np.random.randn() * Rsim[1, 1]]]).T

    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud

def calc_n_LM(x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    """
    Calculates the jacobian of motion model.

    :param x: The state, including the estimated position of the system
    :param u: The control function
    :returns: G:  Jacobian
              Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """

    # [eye(3) [0 x y; 0 x y; 0 x y]]
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_LM(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]],dtype=object)

    G = np.eye(STATE_SIZE) + np.dot(np.dot(Fx.T , jF) ,Fx)
    if calc_n_LM(x) > 0:
        print(Fx.shape)
    return G, Fx,

def calc_LM_Pos(x, z):
    """
    Calculates the pose in the world coordinate frame of a landmark at the given measurement.

    :param x: [x; y; theta]
    :param z: [range; bearing]
    :returns: [x; y] for given measurement
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp


def get_LM_Pos_from_state(x, ind):
    """
    Returns the position of a given landmark

    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_LM_ID(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance.

    If this landmark is at least M_DIST_TH units away from all known landmarks,
    it is a NEW landmark.

    :param xAug: The estimated state
    :param PAug: The estimated covariance
    :param zi:   the read measurements of specific landmark
    :returns:    landmark id
    """

    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        lm = get_LM_Pos_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(np.dot(np.dot(y.T , np.linalg.inv(S)) , y))

    mdist.append(M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid

def calc_input():
    v = control_velocity  # [m/s]
    yawrate = control_steering_angle  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main():
    print("STARTING...")
    global time
    time=0

    # RFID positions [x, y]
     
    subscribers_and_publishers()
    
   
    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hRFID = RFID

    time = rospy.get_rostime().to_sec()

    u = calc_input()

    xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

    xEst, PEst = ekf_slam(xEst, PEst, ud, z)

    x_state = xEst[0:STATE_SIZE]
    print(x_state[0])
    print(x_state[1])
    print(x_state[2])


    path_publish(x_state)
    # store data history
    hxEst = np.hstack((hxEst, x_state))
    hxDR = np.hstack((hxDR, xDR))
    hxTrue = np.hstack((hxTrue, xTrue))
    #hRFID = np.hstack((hRFID, RFID))

def path_publish(x_state):
    path = Path()
    pose = PoseStamped()
    pose.pose.position.x = x_state[0]
    pose.pose.position.y = x_state[1]
    pose.pose.orientation.w = x_state[2]
    path.header.frame_id = "base_footprint"
    path.poses.append(pose)
    car_pose_pub.publish(path)
                         
def publish_landmarks():
    cones = MarkerArray()


if __name__ == '__main__':
  try:
    main()
    while True:
      main()
  except rospy.ROSInterruptException:
    pass
  rospy.spin()
    