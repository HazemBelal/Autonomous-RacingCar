#!/usr/bin/env python
import csv
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
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray



class callbacks_data:
  def __init__(self): 
    "This is the constructor function where every object of the class will get the same but can change values"
    rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,self.cones_cb)                                   #subscribes to lidar_detections
    rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,callback=self.cones_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_cb)                               #subscribes to control topic
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_cb)                                                         #subscribes to IMU sensor
    #rospy.Subscriber("/ground_truth/state_raw",Odometry,self.odom_cb)    

    rospy.Subscriber("/vel_ekf",Float32MultiArray,self.odom_callback)
                                       #subscribes to Odometry topic 
    self.registered_map = False
    self.control_angle = 0
    self.control_velocity = 0
    self.landmark_x=[]
    self.landmark_y=[]
    self.landmark_color=[]
    #self.map_x = []
    #self.map_y = []
    #self.map_color = []

    self.Velocity_x = 0
    self.Velocity_y = 0
    self.position_x = 0
    self.position_y = 0
    self.odom_orientation = 0
    self.imu_header = 'odom'
    self.angular_velocity = 0
    self.Vx = 0
    self.Vy = 0
    self.yaw = 0
    self.tf_listener = tf.TransformListener()
    self.tf_broadcaster = tf.TransformBroadcaster()   
    self.tf_br = tf.TransformBroadcaster()
    # the name of the map coordinate frame
    self.map_frame = "map"
    # the name of the odometry coordinate frame
    self.odom_frame = "odom"
    # the frame of the robot base
    self.base_frame = "base_link"



  def cones_cb(self,cones_data):

   "We will only need the location (x,y) and color in each and new itearation"

   self.landmark_x=[]
   self.landmark_y=[]
   self.landmark_color=[]


   for i in cones_data.cone_detections:
     self.landmark_x.append(i.position.x)
     self.landmark_y.append(i.position.y)
     self.landmark_color.append(i.color.data)

  
  def cones_callback(self,cone_detections):
    
    self.landmark_x = []
    self.landmark_y = []
    self.landmark_color = []

    for cone in cone_detections.markers:
      
      self.landmark_x.append(cone.pose.position.x+1.5)
      self.landmark_y.append(cone.pose.position.y)

      r=cone.color.r
      g=cone.color.g
      b=cone.color.b
      color=""
      if r == 255 and g == 255 and b == 0 :
            color="yellow_cone"
      elif r == 0 and g == 0 and b == 255 :
            color="blue_cone"
      elif r == 255 and g == 0 and b == 0 :
            color="orange_cone"
      self.landmark_color.append(color)


  def control_cb(self,control_data):                        #For control model

   self.control_angle = control_data.drive.steering_angle
   self.control_velocity = control_data.drive.speed


      





  def imu_cb(self,imu_data):

   self.angular_velocity = imu_data.angular_velocity.z
   self.imu_header=imu_data.header.frame_id

  def odom_callback(self,odom_msg):
     self.Vx = odom_msg.data[0]
     self.Vy = odom_msg.data[1]

    #self.Vx = Vx * math.cos() + Vy * math.sin()
    #self.Vy = -Vx * math.sin() + Vy* math.cos() 
     self.heading = odom_msg.data[2]
     self.yaw = odom_msg.data[3]
    # print(self.Vx)



def publishing_cones2(data):

   " we will only need positions ,header ,scales , and color"
   map_marker2=Marker()
   map3=MarkerArray()
   x=0
   print(data,'dataaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
   saving2(data)
   











def publishing_cones(data):

   " we will only need positions ,header ,scales , and color"
   
 
   map_marker=Marker()
   
   x=0
   map_x=[]
   map_y=[]
   global map2

   for i in range(calc_n_LM(data)):

     map_x.append(data[STATE_SIZE + i * 2,0])
     map_y.append(data[STATE_SIZE+ i * 2 + 1,0])
    
 #print("in map viz",len(mapx))
   while x < len(map_x):
    
        map_marker.header.frame_id = "map"
        map_marker.ADD
        map_marker.SPHERE
        map_marker.pose.position.x = map_x[x]
        map_marker.pose.position.y = map_y[x]
        map_marker.pose.position.z = 0
        map_marker.pose.orientation.w = 1
        map_marker.scale.x = 1
        map_marker.scale.y = 1
        map_marker.scale.z = 1
        map_marker.color.a = 1
        map_marker.id +=1
        map_marker.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
        map_marker.type = Marker.MESH_RESOURCE
        map_marker.mesh_use_embedded_materials = True
        map2.markers.append(map_marker)
        x+=1
  

    #print("published cone ",x," number ",calc_n_LM(data))
   pub2.publish(map2) 
   saving(map_x,map_y)
    
 
def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi



def calc_input(data):
    v = data.control_velocity  # [m/s]
    yawrate = data.angular_velocity  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u




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
        mdist.append(math.sqrt(np.dot(np.dot(y.T , np.linalg.inv(S)) , y)))

    mdist.append(M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid





def get_LM_Pos_from_state(x, ind):
    """
    Returns the position of a given landmark

    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm







def calc_LM_Pos(x, z,id):
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











def jacob_motion(x, call_backs_data):
    """
    Calculates the jacobian of motion model.

    :param x: The state, including the estimated position of the system
    :param u: The control function
    :returns: G:  Jacobian
              Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """

    # [eye(3) [0 x y; 0 x y; 0 x y]]
    v=math.sqrt((call_backs_data.Vx**2)+(call_backs_data.Vy**2))
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_LM(x)))))
    global dt 

    jF = np.array([[0.0, 0.0, -dt * v * math.sin(x[2, 0])],
                   [0.0, 0.0, dt * v * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]],dtype=object)

    G = np.eye(STATE_SIZE) + np.dot(np.dot(Fx.T , jF) ,Fx)
    return G, Fx









def calc_n_LM(x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n









def observation(Call_backs_data):
    """
    :param xTrue: the true pose of the system
    :param xd:    the current noisy estimate of the system
    :param u:     the current control input
    :param RFID:  the true position of the landmarks

    :returns:     Computes the true position, observations, dead reckoning (noisy) position,
                  and noisy control function
    """
    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(Call_backs_data.landmark_x)-1): # Test all beacons, only add the ones we can see (within MAX_RANGE)

        dx = Call_backs_data.landmark_x[i]
        dy = Call_backs_data.landmark_y[i]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx))
        if d <= MAX_RANGE:
            dn = d  # add noise
            anglen = angle   # add noise
            zi = np.array([dn, anglen, i])
            z = np.vstack((z, zi))
    return  z





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
    #publishing_cones(lm,LMid)
    y = (z - zp).T # y = innovation
    y[1] = pi_2_pi(y[1])

    H = jacobH(q, delta, xEst, LMid + 1)
    S = np.dot(np.dot(H , PEst) , H.T) + Cx[0:2,0:2]

    return y, S, H


def broadcast_last_transform(call_backs_data):

  """ Make sure that we are always broadcasting the last map
      to odom transformation.  This is necessary so things like
      move_base can work properly. """
###################################################### translation and rotation gaieen mnen 
  call_backs_data.tf_broadcaster.sendTransform(call_backs_data.translation,
                                    call_backs_data.rotation,
                                    rospy.get_rostime(),
                                    call_backs_data.base_frame,
                                    call_backs_data.map_frame)

  
def fix_map_to_odom_transform(call_backs_data,xEst):
  """ This method constantly updates the offset of the map and
      odometry coordinate systems based on the latest results from
      the localizer """

  (call_backs_data.translation, call_backs_data.rotation) = convert_pose_inverse_transform(xEst[0],xEst[1],xEst[2])    #small track +math.pi/2

  p = PoseStamped(pose= convert_translation_rotation_to_pose(call_backs_data.translation,call_backs_data.rotation),
                      header=Header(frame_id=call_backs_data.base_frame))
  
  call_backs_data.odom_to_map = call_backs_data.tf_listener.transformPose(call_backs_data.base_frame, p)


def convert_translation_rotation_to_pose(translation, rotation):
  """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
  return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))


def convert_pose_inverse_transform(poseX,poseY,theta):
  """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
  translation = np.zeros((4,1))
  translation[0] = poseX
  translation[1] = poseY
  translation[2] = 0.0
  translation[3] = 1.0
  


  rotation = np.transpose(rotation_matrix(0, [0,0,1]))
  transformed_translation = rotation.dot(translation)

  translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
  rotation = quaternion_from_matrix(rotation)
  rotation = tf.transformations.quaternion_from_euler(0.0, 0.0 ,theta)
  

  return (translation, rotation)



def motion_model(x, call_backs_data):
    """
    Computes the motion model based on current state and input function.

    :param x: 3x1 pose estimation
    :param u: 2x1 control input [v; w]
    :returns: the resulting state after the control function is applied
    """
    global dt
    x[0] = x[0] + (call_backs_data.Vx)*dt   #  phi-math.pi/2  small track
    x[1] = x[1] + (call_backs_data.Vy)*dt
    #x[2] = x[2] + phi
    x[2]= call_backs_data.yaw
    return x




def ekf_slam(xEst, PEst,z,call_backs_data):
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
    xEst, PEst, G, Fx = predict(xEst, PEst,call_backs_data)
    initP = np.eye(2)

    # Update
    xEst, PEst = update(xEst, PEst,z, initP)

    return xEst, PEst









def predict(xEst, PEst,call_backs_data):
    """
    Performs the prediction step of EKF SLAM

    :param xEst: nx1 state vector
    :param PEst: nxn covariance matrix
    :param u:    2x1 control vector
    :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
    """
    S = STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], call_backs_data)
    xEst[0:S] = motion_model(xEst[0:S],call_backs_data)

    #else:
     #xEst=xEst

    # Fx is an an identity matrix of size (STATE_SIZE)
    # sigma = G*sigma*G.T + Noise
    #PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx
    PEst[0:S, 0:S] = np.dot( G.T , np.dot(PEst[0:S, 0:S] , G )) + np.dot( Fx.T , np.dot(Cx , Fx) )

    return xEst, PEst, G, Fx












def update(xEst, PEst,z, initP):
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
            #print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz, :],minid)))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        vlm=calc_LM_Pos(xEst, z[iz, :],minid)
        saving2(vlm)
        lm = get_LM_Pos_from_state(xEst, minid)
         #print(lm," this is a cone location")
        #publishing_cones(lm)
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

        K = np.dot(np.dot(PEst , H.T) , np.linalg.inv(S)) # Calculate Kalman Gain
        xEst = xEst + np.dot(K , y)
        PEst = np.dot((np.eye(len(xEst)) - np.dot(K , H)) , PEst)
  
        
    xEst[2] = pi_2_pi(xEst[2])
   
    print("size = ",calc_n_LM(xEst))

    publishing_cones(xEst)
    return xEst, PEst




def saving(x,y):
  #print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
  fulldata=[[],[]]
  for i in range(len(x)):
    mydata=[x[i],y[i]]
    fulldata.append(mydata)
    i+=1
  filename='/home/yasser/map.csv'
  with open(filename, 'w') as csvfile: 
    csvwriter = csv.writer(csvfile) 
    csvwriter.writerows(fulldata)

def saving2(x):
  #print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
  fulldata=[[],[]]

  for i in range(len(x)):
    mydata=[x[0,0],x[1,0]]
    fulldata.append(mydata)
    i+=1
  filename='/home/yasser/map_centroid.csv'
  with open(filename, 'w') as csvfile: 
    csvwriter = csv.writer(csvfile) 
    csvwriter.writerows(fulldata)

def savingP(x):
  #print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
  fulldata=[[],[],[]]
  print((len(x)/2))
  for i in range(int(len(x)/2)):
    mydata=[x[i,0],x[i+1,0],x[i+2,0]]
    fulldata.append(mydata)
    i+=1
  filename='/home/yasser/path.csv'
  with open(filename, 'w') as csvfile: 
    csvwriter = csv.writer(csvfile) 
    csvwriter.writerows(fulldata)



def main(call_backs_data):
    "This is the main function where everything takes place"

    global start_time
    start_time = rospy.get_rostime().to_sec()-0.000001
    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)
    # history
    hxEst = xEst
    while not rospy.is_shutdown():
     global  dt
     dt= rospy.get_rostime().to_sec() - start_time
     start_time = rospy.get_rostime().to_sec()

     #u = calc_input(call_backs_data)

     z= observation(call_backs_data)
     xEst, PEst = ekf_slam(xEst, PEst, z,call_backs_data)
     fix_map_to_odom_transform(call_backs_data,xEst)
     broadcast_last_transform(call_backs_data)


     print(xEst,"batngan")
     x_state = xEst[0:STATE_SIZE]
     publish_path(xEst[:3,0])
     #saving(xEst)

    
    #publishing_cones(z)

    # store data history
     hxEst_x = []
     hxEst_y = []
     hxEst_w = []
     hxEst_x.append(x_state[0])
     hxEst_y.append(x_state[1])
     hxEst_w.append(x_state[2])

def publish_path(data_x):
  poses= PoseStamped()
  poses.header.frame_id="map"
  poses.pose.position.x=data_x[0]
  poses.pose.position.y=data_x[1]
  poses.pose.orientation.w=data_x[2]
  global oath
  oath.poses.append(poses)
  pub.publish(oath)
  print("path published")

if __name__ == '__main__':
  rospy.init_node("final_ekf")

  global oath  ,dt , id ,map2
  id =0
  map2=MarkerArray()
  oath = Path()  
  # EKF state covariance
  Cx = np.diag([0.25,0.25, np.deg2rad(10.0)]) ** 2
  r= np.diag([1.0, 1.0])**2 # Process Noise
  map_x2=[]
  map_y2=[]
  
  pub=rospy.Publisher("/yassers_path",Path,queue_size=1)
  pub2=rospy.Publisher("/global_map_yasser",MarkerArray, queue_size=0)
  pub3=rospy.Publisher("/global_map_centroid",MarkerArray, queue_size=0)

  oath.header.frame_id="map"
  MAX_RANGE = 6.0  # maximum observation range
  M_DIST_TH = 1.0  # Threshold of Mahalanobis distance for data association.
  STATE_SIZE = 3  # State size [x,y,yaw]
  LM_SIZE = 2  # LM state size [x,y]
  call_backs_data=callbacks_data()
  try:
      main(call_backs_data)
  except rospy.ROSInterruptException:
      pass
  