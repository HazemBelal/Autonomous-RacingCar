#!/usr/bin/env python3

#ay habd fel 3bd
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

import time 
import matplotlib.pyplot as plt 
from nav_msgs.msg import Odometry
import tf 
import csv
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header, String, ColorRGBA
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray


class EKF():
  
    def __init__(self, namespace='data_gathering'):

        # Initialization of variables
        self.control_steering_angle = 0
        self.control_velocity = 0
        self.angular_vel = 0
        self.Vx = 0
        self.Vy = 0
        self.heading = 0
        self.current_time=0
        # Initialize cone attributes
        self.cones_x = []  # Initialize cones_x
        self.cones_y = []  # Initialize cones_y
        self.cones_color = []  # Initialize cones_color

        # Other initializations
        self.global_map = {
            'xEst': None,
            'pEst': None,
            'mapx': [],
            'mapy': [],
            'mapc': []
        }

        # Time step measurement start
        self.start_time = rospy.get_rostime().to_sec()

        self.state_size = 3  # State size
        self.LM_size = 2
        self.xEst = np.zeros((self.state_size, 1))
        self.pEst = np.zeros((self.state_size, self.state_size ))
        self.x = np.zeros((self.state_size + self.LM_size * 5, 1))


        # Initialize other variables

        self.Xcones_filterd = []
        self.Ycones_filterd = []
        self.Ccones_filterd = []

        self.current_yaw = 0
        self.prev_yaw = 0
        self.delta_yaw = 0

        # Car pose predicted and logged for visualization
        self.car_pose_x = []
        self.car_pose_y = []
        self.car_pose_yaw = []


        self.MAX_RANGE = 8
        self.xEst[2] = -math.pi / 2
        self.prev_time=0
        self.t1=0

        # Threshold of Mahalanobis distance for data association.
        


        rospy.init_node("ekf_slam", anonymous=True)
        rospy.Subscriber("converted_pc", MarkerArray, callback=self.cones_callback)
        rospy.Subscriber("/vel_ekf", Float32MultiArray, self.odom_callback)

        self.car_pose_pub = rospy.Publisher("/car_pose", Path, queue_size=5)
        self.global_map_pub = rospy.Publisher("/global_map", MarkerArray, queue_size=5)
        self.global_map_pub_1time = rospy.Publisher("/global_map_1time", MarkerArray, queue_size=5)
        
        self.rate = rospy.Rate(10)

        self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])

        self.M_DIST_TH = 1
        
        self.path_msg = Path()
        self.marker_array= MarkerArray()
        
        self.u = np.zeros((3, 1))
        
         # Process initial uncertainty
        self.Q = np.array([[0.3, 0.0, 0.0],
                           [0.0, 0.3, 0.0],
                           [0.0, 0.0, 0.1]])  # 0.1745 to make 10deg cov

      
   
    def odom_callback(self, vel_ekf):
      self.u[0] = vel_ekf.data[0]
      self.u[1] = vel_ekf.data[1]
      self.u[2] = vel_ekf.data[2]

      #rospy.loginfo(f"Received Velocity: Vx={self.u[0]}, Vy={self.u[1]}, Angular Velocity={self.u[2]}")

		 
    def cones_callback (self, marker_array):
       
       if not marker_array or not marker_array.markers:
        print("No markers received.")
        return
      
       #print(f"Received {len(marker_array.markers)} markers")

       self.obs = np.empty((0,2))
       initP = np.eye(2)
      

       for marker in marker_array.markers:
          
          #print(f"Processing marker: x={marker.pose.position.x}, y={marker.pose.position.y}")
          d_coneR = math.sqrt((marker.pose.position.x)**2 + (marker.pose.position.y)**2)
          theta_coneR = math.atan2(marker.pose.position.y, marker.pose.position.x)
          self.obs = np.append(self.obs, [(d_coneR, theta_coneR)], axis = 0)

          self.cones_x.append(marker.pose.position.x)
          self.cones_y.append(marker.pose.position.y)

       if not self.cones_x:
        print("No cones detected, skipping EKF update.")
        return

       z = self.get_landmarks()
      
       if z.size == 0:
         print("no lmks detected")

       self.xEst, self.pEst = self.ekf_slam(self.pEst, z, initP)
  
       #print(f"print x of cone: {len(self.xEst)}")
       #print(f"Cones detected: {len(self.cones_x)}")
 
       return self.cones_x, self.cones_y
    

    def viz_path(self): 
      
       pose = PoseStamped()
       self.path_msg.header.frame_id = "map"
       self.path_msg.header.stamp = rospy.Time.now()

       pose.pose.position.x = float(self.xEst[0])  # X coordinate
       pose.pose.position.y = float(self.xEst[1]) # Y coordinate
       pose.pose.position.z = 0

       pose.pose.orientation.x = 0
       pose.pose.orientation.y = 0
       pose.pose.orientation.z = 0
       pose.pose.orientation.w = 1

       #rospy.loginfo(f"State in path={self.xEst}")
       self.path_msg.poses.append(pose)
       #rospy.loginfo(f"Printed path: path={self.path_msg}")

       self.car_pose_pub.publish(self.path_msg)



    def viualise_global_map(self, min_id, xEst) :
      lm = self.get_landmark_position_from_state(min_id, xEst)
      cone_msg = Marker()
        
      cone_msg.header.frame_id = 'map'
      cone_msg.header.stamp = rospy.Time.now()
      cone_msg.type = Marker.SPHERE

     # print(f'lm shape= {lm.shape}')
     # print (f'lm = {lm}')

      cone_msg.pose.position.x = float(lm[0])
      cone_msg.pose.position.y = float(lm[1])
      cone_msg.pose.position.z = 0

      cone_msg.pose.orientation.x = 0
      cone_msg.pose.orientation.y = 0
      cone_msg.pose.orientation.z = 0
      cone_msg.pose.orientation.w = 1

      cone_msg.scale.x = 1 
      cone_msg.scale.y = 1
      cone_msg.scale.z = 1
      
      cone_msg.lifetime = rospy.Duration(0)
      cone_msg.action = Marker.ADD

      self.marker_array.markers.append(cone_msg)

      self.global_map_pub.publish(self.marker_array)
     
      
    def ekf_slam(self, pEst, z, initP) :

      # get landmark EKF format from cones positions
      z = self.get_landmarks()

      self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])
      #rospy.loginfo(f"State before prediction: xEst={self.xEst.flatten()}")

      ''''
      rospy.loginfo(f"Covariance before prediction: pEst={pEst}")
      rospy.loginfo(f"Initial pEst: {pEst.shape}, Initial z: {z.shape}")
      rospy.loginfo(f"n_lm shape: {self.calc_n_lm(self.xEst)}")
      '''

      #if z.shape[0] == 0:
       # rospy.logwarn("No landmarks detected or data is missing.")

      self.xEst, self.pEst = self.predict (self.xEst, pEst, z, initP)
      #rospy.loginfo(f'State after prediction: {self.xEst}')
      
      initP = np.eye(2)
      
      return self.xEst, self.pEst
               

    def get_landmarks(self):

      z = np.zeros((0, 3))

      for i in range(len(self.cones_x)):

         x = self.cones_x[i]
         y = self.cones_y[i]
         d = math.sqrt(x**2 + y**2)
         angle = self.pi_2_pi(math.atan2(y, x)+math.pi/2)
      
         if d <= self.MAX_RANGE:
            zi = np.array([d, angle, i]) 
            z = np.vstack((z, zi))
   
      return z
    
         
    def pi_2_pi(self, angle):
      return (angle + math.pi) % (2 * math.pi) - math.pi
    
   
    def predict (self, xEst, pEst, z, initP):
      #print(f"Previous covariance: {pEst}")      
      G, Fx = self.jacob_motion (xEst, z)
      '''''
      print(f"Fx.T shape: {Fx.T.shape}")
      print(f"Fx shape: {Fx.shape}")
      print(f"G shape: {G.shape}")
      print(f"G.T shape: {G.T.shape}")
      print(f"pEst shape: {pEst.shape}")
      print(f"cx shape: {self.Cx.shape}")
      print(f"xEst length: {len(xEst)}, pEst shape: {pEst.shape}")
      print(f"G shape: {G.shape}, Fx.T shape: {Fx.T.shape}, self.Cx shape: {self.Cx.shape}")
      '''
      
      xEst[0:self.state_size] = self.motion_model(xEst[0:self.state_size])
      #rospy.loginfo(f"Motion model inputs: u={self.u.flatten()}")
     # print (f'pEst : {pEst}')
      pEst =  G.T @ pEst @ G + Fx.T @ self.Cx @ Fx
   #   print(f"Updated covariance: {pEst}")
      #print(f"state: {xEst}")
      #print("length of Xest:", len(xEst))
      
      self.update(xEst, pEst, z, initP)
      
      return xEst, pEst 
            
         
    def jacob_motion (self, x, z):
        # rospy.loginfo(f"xEst shape: {x.shape}")
        nLM = self.calc_n_lm (x)
        Fx = np.hstack(( np.eye(self.state_size), np.zeros((self.state_size, self.LM_size * nLM)) ))
        if self.t1==1:
          self.current_time=rospy.get_rostime().to_sec()
          t=self.current_time-self.prev_time
          #print (f'x shape : {x.shape}')
      
          jF = np.array([[0.0, 0.0, - 0.025 * self.u[0] * math.sin(x[2, 0])],
                         [0.0, 0.0, 0.025 * self.u[0] * math.cos(x[2, 0])],
                         [0.0, 0.0, 0.0]],dtype=object)
          self.prev_time=self.current_time

        else :
          self.t1=1
          jF = np.array([[0.0, 0.0, - 0.025 * self.u[0] * math.sin(x[2, 0])],
                         [0.0, 0.0, 0.025 * self.u[0] * math.cos(x[2, 0])],
                         [0.0, 0.0, 0.0]],dtype=object)
          self.prev_time=rospy.get_rostime().to_sec()
        #print(f"Fx.T shape: {Fx.T.shape}")
        #print(f"Fx shape: {Fx.shape}")
        #print(f"jF shape: {jF.shape}")
        G = np.eye(self.state_size + self.LM_size * nLM) + Fx.T @jF @ Fx
      # rospy.loginfo(f"G shape: {G.shape}")
        #if self.calc_n_lm(x) > 0:
        #  print(f'fx shape{Fx.shape}')
 
        return G, Fx
    
            
    def calc_n_lm(self, x):
  
      n = int((len(x) - self.state_size) / self.LM_size)
     # print(f"len(x): {len(x)}, self.state_size: {self.state_size}, self.LM_size: {self.LM_size}, calculated n: {n}")
      return n   
            
    def motion_model(self, x):

      if self.t1==1:

        self.current_time=rospy.get_rostime().to_sec()
        t=self.current_time-self.prev_time
        print (f'previous {self.prev_time}')
        print (f'current {self.current_time}')

        Vx = self.u[0]
        Vy = self.u[1]

        x[0] = x[0] + (Vx) * t
        x[1] = x[1] + (Vy) * t
        x[2] = self.u[2]
        x[2] = self.pi_2_pi(x[2])
        self.prev_time=self.current_time
        print(t)
      else:
        Vx = self.u[0]
        Vy = self.u[1]
        self.t1=1
        x[0] = x[0] + (Vx) * 0.025
        x[1] = x[1] + (Vy) * 0.025
        x[2] = self.u[2]
        x[2] = self.pi_2_pi(x[2])
        self.prev_time=rospy.get_rostime().to_sec()
      
      self.x = x

      #print(f"Before Update: xEst={self.xEst.flatten()}")
      return x   
  
         
    def update(self, xEst, pEst, z, initP): 
     # rospy.loginfo(f"Observation z: {z}")
      #rospy.loginfo("Running update step")
      #print (f'Xest before update : {xEst}')

      for iz in range(len(z[:, 0])):  # for each observation
        #rospy.loginfo(f"Processing observation {iz}")
        min_id = self.search_correspond_landmark_id(xEst, pEst, z[iz,0:2])
        #print(f'min_id = {min_id}, n.lm = {self.calc_n_lm(xEst)}')
        nLM = self.calc_n_lm(xEst)

        if min_id == nLM and min_id != 0:
          #print("Adding new landmark...")
          new_landmark = self.calc_landmark_position(xEst, z[iz, :])
  
          xAug = np.vstack((xEst, new_landmark))
          PAug = np.vstack((np.hstack((pEst, np.zeros((len(xEst), self.LM_size)))),
                            np.hstack((np.zeros((self.LM_size, len(xEst))), initP))))

          #rospy.logdebug(f"pEst after augmentation: {PAug.shape}")
          self.xEst = xAug
          self.pEst = PAug
          #rospy.loginfo(f"Landmark added at position: {new_landmark.T}")


        # Calculate the landmark position from state using min_id
        lm = self.get_landmark_position_from_state(min_id, xEst) 
        
        # Compute innovation
        y, S, H = self.calc_innovation(lm, xEst, pEst, z[iz, 0:2], min_id)
        #rospy.logdebug(f"Innovation y: {y}, Covariance S: {S}, H: {H}")

        # Calculate Kalman Gain
        K = (pEst @ H.T) @ np.linalg.inv(S)
       # rospy.loginfo(f"Kalman Gain K: {K}")
        # Update state and covariance
        self.xEst = xEst + (K @ y)
        self.pEst = (np.eye(len(xEst)) - (K @ H)) @ pEst

     # for i in range(self.calc_n_lm(xEst)):   
       # rospy.loginfo(f"Landmark {i} position: x={xEst[self.state_size + i * 2, 0]}, "
                   #     f"y={xEst[self.state_size + i * 2 + 1, 0]}") 
    
      self.viz_path()
      self.viualise_global_map(min_id, xEst) 

      xEst[2] = self.pi_2_pi(xEst[2]) 
      #rospy.loginfo(f"Final xEst: {xEst.T}")

      return self.xEst, self.pEst
         


    def search_correspond_landmark_id(self, xAug, PAug, iz):

     # print(f"xAug: {xAug}")
      #print(f"PAug shape: {PAug.shape}")
      #print(f"Measurement (iz): {iz}")
      
      min_dist = []
      nLM = self.calc_n_lm(xAug)
      print(f"Number of landmarks in the state: {nLM}")
      for i in range(nLM):
         lm = self.get_landmark_position_from_state(xAug, i)
         print(f"lm shape: {lm.shape}")
         y, S, H = self.calc_innovation(lm, xAug, PAug, iz, i)
         print(f"y shape = {y.shape}, S shape = {S.shape}, H shape = {H.shape}")
         min_dist.append(y.T @ np.linalg.inv(S) @ y) 
      #print(f"New landmark threshold (M_DIST_TH): {self.M_DIST_TH}")
      min_dist.append(self.M_DIST_TH) 
      #print(f"Mahalanobis distances: {min_dist}")
      min_id = min_dist.index(min(min_dist))
      #print(f"Selected landmark ID: {min_id} (new landmark if {min_id == len(min_dist) - 1})")

      return min_id


    def get_landmark_position_from_state(self, ind, x):
      ''''
      print(f"x: {x}")
      print(f"self.state_size: {self.state_size}")
      print(f"self.LM_size: {self.LM_size}")
      print(f"Slicing indices: {self.state_size + self.LM_size * ind} to {self.state_size + self.LM_size * (ind + 1)}")
      '''
      
      self.lm = x[self.state_size + self.LM_size * ind: self.state_size + self.LM_size * (ind + 1), :]
     # print(f"Extracted lm: {lm}")
     # print(f"x size: {x.shape}")
      #print(f"Landmark at index {ind}: {self.lm}")
      #print(f"Retrieved lm: {self.lm}")
     # print(f'lm : {lm.shape}')
      #print (f'lm in function : {lm}')
      return self.lm
            
            
            
    def calc_innovation(self, lm, xEst, pEst, z, LMid):

      if lm.shape != (2, 1):
        raise ValueError(f"Invalid shape for lm: {lm.shape}")
      if xEst[0:2].shape != (2, 1):
        raise ValueError(f"Invalid shape for xEst[0:2]: {xEst[0:2].shape}")

      #print(f"Landmark (lm) shape: {lm.shape}, value: {lm}")
      #print(f"lm: {lm}, xEst: {xEst}")
      #print(f"xEst shape: {xEst.shape}, pEst shape: {pEst.shape}")
      #print(f"z shape: {z.shape}, z[iz]: {z if z.shape[1] > 2 else 'Invalid'}")

      delta = lm - xEst[0:2]
      q = (delta.T @ delta)[0, 0]
      z_angle = math.atan2(delta[1, 0], delta[0, 0])  - xEst[2, 0]
      zp = np.array([[math.sqrt(q), self.pi_2_pi(z_angle)]])

      y = (z - zp).T   # 2*1
      y[1] = self.pi_2_pi(y[1])

      H = self.jacob_h(q, delta, xEst, LMid + 1)  # 2 * n    n: 3 + 2*nLM
      pEst = np.array(pEst, dtype=np.float64)
      S = H @ pEst @ H.T + self.Cx[0:2, 0:2] # 2*2

      return y, S, H
            
         
    def jacob_h(self, q, delta, x, i):

      sq = math.sqrt(q)
      G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                    [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])                  # 2*5

      G = G / q
      nLM = self.calc_n_lm(x)
      F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
      F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
               np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
      
      if F1.shape[1] < F2.shape[1]:
        padding = np.zeros((F1.shape[0], F2.shape[1] - F1.shape[1]))
        F1 = np.hstack((F1, padding))

      F = np.vstack((F1, F2))   # 5* (3+nLm*2)
      H = np.dot( G , F )  #2* (3+nLm*2)
      
      return H
            
    def calc_landmark_position(self, z, x):
      print(75)
      print(f"z shape: {z.shape}, x shape: {x.shape}")
      print(f"z: {z}, x: {x}")
 
      zp = np.zeros((2, 1))

      zp[0, 0] = x[0] +( z[0, 0] * math.cos(x[2] + z[1, 0] ) )
      zp[1, 0] = x[1] +( z[0, 0] * math.sin(x[2] + z[1, 0] ) )

      return zp

		    
if __name__ == '__main__':

   rospy.init_node("ekf_slam", anonymous=True)
   try:
      ekf_instance = EKF ()
   except rospy.ROSInterruptException:
      pass
   rospy.spin() 