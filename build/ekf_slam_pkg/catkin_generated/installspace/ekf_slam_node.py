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
        rospy.Subscriber("/camera_cones_marker", MarkerArray, callback=self.cones_callback)
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
       rospy.loginfo(f"[cones_callback] called with {len(marker_array.markers)} markers at t={rospy.get_rostime().to_sec():.3f}")
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

       rospy.loginfo(f"[viz_path] publishing Path with {len(self.path_msg.poses)} poses")
       self.car_pose_pub.publish(self.path_msg)





    def viualise_global_map(self, xEst):
        """
        Publish all landmarks in xEst as a MarkerArray.
        xEst: full state vector [x,y,θ, lm1_x, lm1_y, lm2_x, lm2_y, ...]
        """
        marker_array = MarkerArray()
        nLM = self.calc_n_lm(xEst)

        for i in range(nLM):
            lm = self.get_landmark_position_from_state(i, xEst)  # (2,1)

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'cones'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            # position
            m.pose.position.x = float(lm[0, 0])
            m.pose.position.y = float(lm[1, 0])
            m.pose.position.z = 0

            # orientation
            m.pose.orientation.w = 1.0

            # size
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2

            # color
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0

            marker_array.markers.append(m)

        self.global_map_pub.publish(marker_array)
        rospy.loginfo(f"[viualise_global_map] published {nLM} markers")

      
    def ekf_slam(self, pEst, z, initP) :
      rospy.loginfo(f"[ekf_slam] begin – pEst.shape={pEst.shape}, z.shape={z.shape}")

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
    
   
    def predict(self, xEst, pEst, z, initP):
      rospy.loginfo(f"[predict] begin – xEst.len={len(xEst)}, pEst.shape={pEst.shape}")

      # compute dt exactly once
      now = rospy.get_rostime().to_sec()
      if self.prev_time is None:
          dt = 0.025
      else:
          dt = now - self.prev_time
      self.prev_time = now

      # build motion‐model Jacobian and state‐selection matrix
      G, Fx = self.jacob_motion(xEst, dt)

      # apply the motion model
      xEst[0:self.state_size] = self.motion_model(xEst[0:self.state_size], dt)

      # predict covariance
      pEst = G.T @ pEst @ G + Fx.T @ self.Cx @ Fx

      # EKF update
      xEst, pEst = self.update(xEst, pEst, z, initP)

      # force numeric dtype
      xEst = np.array(xEst, dtype=np.float64)
      pEst = np.array(pEst, dtype=np.float64)
      self.xEst, self.pEst = xEst, pEst

      return xEst, pEst


    def jacob_motion(self, x, dt):
      # ensure x is float64
      x = np.array(x, dtype=np.float64)
      nLM = self.calc_n_lm(x)

      # selection matrix for pose block
      Fx = np.hstack((
          np.eye(self.state_size),
          np.zeros((self.state_size, self.LM_size * nLM))
      ))

      # build 3×3 local Jacobian
      v   = float(self.u[0])
      theta = float(x[2, 0])
      jF = np.array([
          [0.0, 0.0, -dt * v * math.sin(theta)],
          [0.0, 0.0,  dt * v * math.cos(theta)],
          [0.0, 0.0,  0.0]
      ], dtype=float)

      # expand to full state
      G = np.eye(self.state_size + 2 * nLM) + Fx.T @ jF @ Fx

      return G, Fx


    
            
    def calc_n_lm(self, x):
  
      n = int((len(x) - self.state_size) / self.LM_size)
     # print(f"len(x): {len(x)}, self.state_size: {self.state_size}, self.LM_size: {self.LM_size}, calculated n: {n}")
      return n   
            
    def motion_model(self, x, dt):
        # simple constant‐velocity model in robot frame
        Vx = float(self.u[0])
        Vy = float(self.u[1])
        x_pos, y_pos, yaw = float(x[0]), float(x[1]), float(x[2])

        # integrate
        x_pos += Vx * dt
        y_pos += Vy * dt
        yaw   = self.pi_2_pi(float(self.u[2]))  # assume u[2] already instantaneous heading

        # write back
        x[0], x[1], x[2] = x_pos, y_pos, yaw
        return x
 
  
         
    def update(self, xEst, pEst, z, initP):
      rospy.loginfo(f"[update] begin – #obs={z.shape[0]}, state_len={len(xEst)}")
      last_min_id = None

      # EKF-SLAM update loop
      for iz in range(z.shape[0]):
          min_id = self.search_correspond_landmark_id(xEst, pEst, z[iz, 0:2])
          last_min_id = min_id
          nLM = self.calc_n_lm(xEst)

          if min_id == nLM:
              # new landmark
              new_lm = self.calc_landmark_position(xEst, z[iz, :])
              xEst = np.vstack((xEst, new_lm))

              PA = np.hstack((pEst, np.zeros((pEst.shape[0], 2))))
              PB = np.hstack((np.zeros((2, pEst.shape[1])), initP))
              pEst = np.vstack((PA, PB))

              self.xEst, self.pEst = xEst, pEst
              continue

          # existing landmark update
          lm = self.get_landmark_position_from_state(min_id, xEst)
          y, S, H = self.calc_innovation(lm, xEst, pEst, z[iz, 0:2], min_id)
          K = (pEst @ H.T) @ np.linalg.inv(S)

          xEst = xEst + (K @ y)
          pEst = (np.eye(len(xEst)) - (K @ H)) @ pEst

          self.xEst, self.pEst = xEst, pEst

      # After all observations, publish path and full map
      self.viz_path()
      self.viualise_global_map(xEst)

      # normalize heading
      xEst[2] = self.pi_2_pi(xEst[2])

      rospy.loginfo(f"[update] DONE – published path & map")
      return self.xEst, self.pEst


    def search_correspond_landmark_id(self, xAug, PAug, iz):

     # print(f"xAug: {xAug}")
      #print(f"PAug shape: {PAug.shape}")
      #print(f"Measurement (iz): {iz}")
      
      min_dist = []
      nLM = self.calc_n_lm(xAug)
      print(f"Number of landmarks in the state: {nLM}")
      for i in range(nLM):
         lm = self.get_landmark_position_from_state(i, xAug)
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
        """
        Compute the EKF‐SLAM measurement Jacobian H for the i-th landmark.

        Args:
          q     (float): predicted squared range to the landmark
          delta (array-like): relative vector [dx, dy] (any shape)
          x     (ndarray): full state vector [x,y,θ,lm1_x,lm1_y,…]
          i     (int): 1-based index of the landmark in the state

        Returns:
          H (ndarray): 2×(3 + 2*nLM) Jacobian of the measurement model
        """
        # --- DEBUG: inspect delta before flattening ---
        rospy.loginfo(f"[jacob_h] raw delta: {delta} (type {type(delta)}, shape {getattr(delta, 'shape', None)})")

        # 1) Flatten delta into two floats
        d = np.array(delta).reshape(-1)
        rospy.loginfo(f"[jacob_h] flattened d: {d} (types: {type(d[0])}, {type(d[1])})")
        dx = float(d[0])
        dy = float(d[1])
        rospy.loginfo(f"[jacob_h] dx={dx}, dy={dy}")

        # 2) Build the 2×5 G matrix with only scalar entries
        sq = math.sqrt(q)
        # intermediate print
        rospy.loginfo(f"[jacob_h] q={q}, sqrt(q)={sq}")
        G = np.array([
            [-sq * dx,    -sq * dy,    0.0,  sq * dx,    sq * dy],
            [    dy,         -dx,      -q,   -dy,         dx   ]
        ], dtype=float) * (1.0 / q)
        rospy.loginfo(f"[jacob_h] G matrix:\n{G}")

        # 3) Build the selection matrix F of size (5) × (3 + 2*nLM)
        nLM = self.calc_n_lm(x)
        rospy.loginfo(f"[jacob_h] nLM={nLM}, full state length={len(x)}")

        # Top: identity for robot pose, zeros for all landmarks
        F1 = np.hstack((np.eye(3), np.zeros((3, 2*nLM))))
        # Bottom: zeros for robot, zeros up to (i-1) landmarks, I2 for this lm, zeros for rest
        before = np.zeros((2, 2*(i-1)))
        after  = np.zeros((2, 2*nLM - 2*(i-1) - 2))
        F2 = np.hstack((np.zeros((2,3)), before, np.eye(2), after))
        F  = np.vstack((F1, F2))
        rospy.loginfo(f"[jacob_h] F matrix shape: {F.shape}")

        # 4) Compute full Jacobian
        H = G.dot(F)
        rospy.loginfo(f"[jacob_h] H matrix shape: {H.shape}")
        return H      
    

    def calc_landmark_position(self, xEst, zi):
        """
        Compute the world‐frame position of a new landmark.

        Args:
          xEst (ndarray 3×1): current robot pose [x; y; θ]
          zi   (array-like of length ≥2): single observation [range, bearing, ...]
        Returns:
          zp   (ndarray 2×1): landmark position [lm_x; lm_y]
        """
        # Unpack robot pose
        x = float(xEst[0, 0])
        y = float(xEst[1, 0])
        yaw = float(xEst[2, 0])

        # Unpack measurement
        r     = float(zi[0])
        theta = float(zi[1])

        # Compute landmark coordinates in world frame
        lm_x = x + r * math.cos(yaw + theta)
        lm_y = y + r * math.sin(yaw + theta)

        return np.array([[lm_x], [lm_y]])

		    
if __name__ == '__main__':

   rospy.init_node("ekf_slam", anonymous=True)
   try:
      ekf_instance = EKF ()
   except rospy.ROSInterruptException:
      pass
   rospy.spin() 