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
#import ros_numpy
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion,PoseWithCovariance
from std_msgs.msg import Float32MultiArray
import statistics

class Sensors():
  def __init__(self, namespace='data_gathering'):

    rospy.init_node("ekf_slam", anonymous = True)

    #rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,callback=self.cones_callback)
    rospy.Subscriber("/camera_cones_marker",MarkerArray,callback=self.cones_callback)
    #rospy.Subscriber("/cam_lidar_fusion_cones",MarkerArray, self.cones_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
    #rospy.Subscriber("/ground_truth/state",Odometry,self.odom_callback)
    rospy.Subscriber("/vel_ekf",Float32MultiArray,self.odom_callback)

    

    self.car_pose_pub = rospy.Publisher("/car_pose", Path, queue_size=5)
    self.global_map_pub = rospy.Publisher("/global_map",MarkerArray, queue_size=5)
    self.global_map_covariance = rospy.Publisher("/global_map_covariance",PoseWithCovarianceStamped, queue_size=5)
    self.global_map_pub_1time = rospy.Publisher("/global_map_1time",MarkerArray, queue_size=5)
  

    
    # input control
    # control_velocity : velocity Command
    # control_steering_angle : steering angle command
    self.control_velocity = 0
    self.control_steering_angle = 0
    
    #Covariance initialization 
    self.covariance_x=[]
    self.covariance_y=[]
    self.covariance_counter=0


    # measurements 
    # Vx : linear  Velocity in the X direction
    # VY : linear  Velocity in the Y direction
    # angular_vel : Vehicle angular velocity around z axis (yaw rate)
    self.Vx = 0
    self.Vy = 0
    self.heading =0
    self.angular_vel = 0
    self.yaw = 0

    # EKF parameters
    # STATE_SIZE : (X,Y,Yaw)
    # xEst : state estimate
    # pEst : Covariance
    self.STATE_SIZE = 3
    self.xEst = np.zeros((self.STATE_SIZE, 1)) #3*1>> will make all the values=0
    self.pEst = np.eye(self.STATE_SIZE)    #identity matrix
    
    #self.xEst[2]= -math.pi/2   #small track ,big track, acceleration 0degs   skidpad 90deg
    
    #process initial uncertanity"used only once"
    self.Q=np.array([[0.3,0.0,0.0],
                    [0.0,0.3,0.0],
                    [0.0,0.0,0.1]])  # 0.1745 to make 10deg cov

    # time step measurement Start
    self.start_time = rospy.get_rostime().to_sec()

    # delta yaw calculation 
    # prev_yaw : previous yaw
    # delta_yaw : delta yaw
    self.current_yaw = 0
    self.prev_yaw = 0
    self.delta_yaw = 0

    # car pose predicted and logged for visualization
    # car_pose_x : car pose X
    # car_pose_y : car pose Y
    self.car_pose_x = []
    self.car_pose_y = []
    self.car_pose_yaw = []


    # global map constructed and logged for visualization
    # global_map_X : landmarks X
    # global_map_Y : landmarks Y
    self.mapx = []
    self.mapy = []
    self.mapc = []



    self.cones_x = []
    self.cones_y = []
    self.cones_color = []



    # tf broadcaster
    self.tf_br = tf.TransformBroadcaster()

    # the name of the map coordinate frame
    self.map_frame = "map"
    # the name of the odometry coordinate frame
    self.odom_frame = "odom"
    # the frame of the robot base
    self.base_frame = "base_link"
    self.zed_frame = "zed2i"
     

    # enable listening for and broadcasting coordinate transforms
    self.tf_listener = tf.TransformListener()
    self.tf_broadcaster = tf.TransformBroadcaster()   

    self.loop_closure_flag = False
    self.started_motion = False
    #self.map_pub_flag = False
    self.sent_before = False

    self.global_map_msg = MarkerArray()

    self.id =0

    #filtered cones
    self.Xcones_filterd = []
    self.Ycones_filterd = []
    self.Ccones_filterd = []

  #prepare the cones in self.Xcones_filterd   ,self.Ycones_filterd   ,self.Ccones_filterd  
  def cones_callback(self,cone_detections):
    print("cooooonee")
    
    self.cones_x = []
    self.cones_y = []
    self.cones_color = []

    for cone in cone_detections.markers:
      
      self.cones_x.append(cone.pose.position.x)#+1.7)
      self.cones_y.append(cone.pose.position.y)#-0.2)

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
      self.cones_color.append(color)


    
    i = 0

    Xcones_filterd_temp = []
    Ycones_filterd_temp = []
    Ccones_filterd_temp = []


    while i <= len(self.cones_x)-1:
      xcomp = self.cones_x[i]
      ycomp = self.cones_y[i]
      Ccomp = self.cones_color[i]
      
      d_comp = math.sqrt((xcomp)**2 + (ycomp)**2)
      theta_comp = math.atan2(ycomp,xcomp)

      j = i+1

      while j<= len(self.cones_x)-1:
        

        xiter = self.cones_x[j]
        yiter = self.cones_y[j]

        d_iter = math.sqrt((xiter)**2 + (yiter)**2)
        theta_iter = math.atan2(yiter,xiter)
        
        if ( abs(d_iter-d_comp)<0.25 ) and  (abs(theta_comp-theta_iter)<0.01):
          j+=1
          continue

        else :
          Xcones_filterd_temp.append(xcomp)
          Ycones_filterd_temp.append(ycomp)
          Ccones_filterd_temp.append(Ccomp)
          break

      i+=1
    
    self.Xcones_filterd = self.cones_x
    self.Ycones_filterd = self.cones_y
    self.Ccones_filterd = self.cones_color
  '''
  def cones_callback(self,cone_detections):
    
    self.cones_x = []
    self.cones_y = []
    self.cones_color = []
    print("cone detection")

    for cone in cone_detections.cone_detections:
      
      self.cones_x.append(cone.position.x)
      self.cones_y.append(cone.position.y)
      self.cones_color.append(cone.color.data)


    
    i = 0

    Xcones_filterd_temp = []
    Ycones_filterd_temp = []
    Ccones_filterd_temp = []


    while i <= len(self.cones_x)-1:
      xcomp = self.cones_x[i]
      ycomp = self.cones_y[i]
      Ccomp = self.cones_color[i]

      d_comp = math.sqrt((xcomp)**2 + (ycomp)**2)
      theta_comp = math.atan2(ycomp,xcomp)

      j = i+1

      while j<= len(self.cones_x)-1:
        

        xiter = self.cones_x[j]
        yiter = self.cones_y[j]

        d_iter = math.sqrt((xiter)**2 + (yiter)**2)
        theta_iter = math.atan2(yiter,xiter)
        
        if ( abs(d_iter-d_comp)<0.25 ) and  (abs(theta_comp-theta_iter)<0.01):
          j+=1
          continue

        else :
          Xcones_filterd_temp.append(xcomp)
          Ycones_filterd_temp.append(ycomp)
          Ccones_filterd_temp.append(Ccomp)
          break

      i+=1
    
    self.Xcones_filterd = Xcones_filterd_temp
    self.Ycones_filterd = Ycones_filterd_temp
    self.Ccones_filterd = Ccones_filterd_temp
  '''
  #prepare   self.control_steering_angle    self.control_velocity   
  def control_callback(self,control_msg):
    self.control_steering_angle = control_msg.drive.steering_angle
    self.control_velocity = control_msg.drive.speed


  #prepare    self.angular_vel
  def imu_callback(self,imu_msg):
    angular_vel = imu_msg.angular_velocity.z
    self.angular_vel = angular_vel
    

  #prepare    self.Vx   , self.Vy  ,heading, self.current_yaw
  def odom_callback(self,odom_msg):
    self.Vx = odom_msg.data[0]
    self.Vy = odom_msg.data[1]

    #self.Vx = Vx * math.cos() + Vy * math.sin()
    #self.Vy = -Vx * math.sin() + Vy* math.cos() 
    self.heading = odom_msg.data[3]
    self.yaw = odom_msg.data[2]
    #print("yawwww",self.yaw)
    # print(self.Vx)
    # print(self.Vy)
    '''self.odomx = odom_msg.pose.pose.position.x
    self.odomy = odom_msg.pose.pose.position.y 
    self.odom_orientation = odom_msg.pose.pose.orientation
    orientation_list = [self.odom_orientation.x, self.odom_orientation.y, self.odom_orientation.z, self.odom_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    self.current_yaw = yaw'''







class Ekf():


  def __init__(self,xEst,pEst,mapc ,cones_x, cones_y,cones_c,delta_yaw,prev_yaw,angular_vel,control_steering_angle,control_velocity,Vx,Vy,start_time,Q,loop_closure_flag,heading,yaw):
    

    self.xEst = xEst
    self.pEst = pEst
    self.mapc = mapc

    self.cones_x = cones_x
    self.cones_y = cones_y
    self.cones_c = cones_c

    self.delta_yaw = delta_yaw
    self.prev_yaw = prev_yaw
    self.angular_vel = angular_vel

    self.control_steering_angle = control_steering_angle
    self.control_velocity = control_velocity

    self.u = np.array([[math.sqrt(Vx**2+Vy**2), angular_vel]]).T

    self.Vx = Vx
    self.Vy = Vy
    self.heading = heading
    self.yaw = yaw
    
    self.dt = rospy.get_rostime().to_sec() - start_time
    self.Q = Q

    self.state_size = 3
    self.LM_SIZE = 2

    # EKF state covariance
    self.Cx = np.diag([0.25,0.25, np.deg2rad(5)])

    # maximum observation range
    self.MAX_RANGE = 8

    # Threshold of Mahalanobis distance for data association.
    self.M_DIST_TH = 1

    self.mapx = []
    self.mapy = []

    self.loop_closure_flag = loop_closure_flag





def check_motion(global_map ):
  x = float(global_map.xEst[0])
  y = float(global_map.xEst[1])
  d=math.sqrt(x**2+y**2)
  if d > 5 and (global_map.started_motion == False):
    global_map.started_motion = True


def check_closure2(global_map ):
  x = float(global_map.xEst[0])
  y = float(global_map.xEst[1])
  d=math.sqrt(x**2+y**2)
  if d < 2.5 :
    global_map.loop_closure_flag = True
    viualise_global_map(global_map)

'''
def file_saved(global_map ):
  i=0
  f= open(r'/home/derorca/data.txt', 'w')
  f.write("X positions \n")
  while i<len(global_map.car_pose_x):
    x= global_map.car_pose_x[i]
    f.write( str(x)+"\n")
    i= i+1

  f.write("Y positions\n")
  i=0
  while i<len(global_map.car_pose_y):
    y= global_map.car_pose_y[i] 
    f.write(str(y)+"\n")
    i= i+1

  f.close()
'''
'''
def save_map(global_map ):
  i=0
  f= open(r'/home/derorca/map.txt', 'w')
  f.write("X positions \n")
  while i<len(global_map.mapx):
    x= global_map.mapx[i]
    f.write( str(x)+"\n")
    i= i+1

  f.write("Y positions\n")
  i=0
  while i<len(global_map.mapy):
    y= global_map.mapy[i] 
    f.write(str(y)+"\n")
    i= i+1

  f.close()
'''


def csv_map(global_map):
  #print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
  fulldata=[[],[],[]]
  for i in range(len(global_map.mapx)):
   mydata=[global_map.mapx[i]-15,global_map.mapy[i]+14.4,global_map.mapc[i]]
   fulldata.append(mydata)
    
  filename='map_code_transformed_small_track.csv'
  with open(filename, 'w') as csvfile: 
     csvwriter = csv.writer(csvfile) 
     csvwriter.writerows(fulldata)


'''
def variance(global_map):
     # Number of observations
      n = len(global_map.mapx)
     # Mean of the data
      i=0
      while i <= n:
        global_map.sum_x[i]= global_map.sum_x[i] + global_map.mapx[i]
        global_map.sum_y[i]= global_map.sum_y[i] + global_map.mapy[i]
        global_map.mean_x[i] = global_map.sum_x[i] / global_map.covariance_counter
        global_map.mean_y[i] = global_map.sum_y[i] / global_map.covariance_counter
        # Square deviations
        deviations_x = [(x - global_map.mean_x) ** 2 for x in global_map.mapx[i]]
        deviations_y = [(y - global_map.mean_y) ** 2 for y in global_map.mapy[i]]
    # Variance
      variance = sum(deviations) / n
      global_map.covariance_counter +=1
    #global_map.covariance=[]
'''

def viz_path(global_map ):
  path_msg = Path()
  path_msg.header.frame_id = 'map'
  
  i = 0

  


  while i < len(global_map.car_pose_x):
    #print("vis path")
    pose_msg = PoseStamped()

    pose_msg.pose.position.x = global_map.car_pose_x[i]
    pose_msg.pose.position.y = global_map.car_pose_y[i]
    pose_msg.pose.orientation.w = global_map.car_pose_yaw[i]
    path_msg.header.frame_id = 'map'
    path_msg.poses.append(pose_msg)



    i+=1
    

  
  global_map.car_pose_pub.publish(path_msg)


def  viualise_global_map(global_map):
  cone_msg = Marker()
 
  c = 0
  id = global_map.id
  #print("in map viz",len(global_map.mapx))
  while c < len(global_map.mapx):
    
    
    x_cone = global_map.mapx[c]
    y_cone = global_map.mapy[c]
    c_cone = global_map.mapc[c]

    #global_map.covariance.pose.pose.position.x=x_cone
    ##global_map.covariance.pose.pose.position.y=y_cone
    #global_map.covariance.pose.pose.orientation.w=1
    #global_map.covariance.pose.covariance=[1,2,3,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    #global_map.global_map_covariance.publish(global_map.covariance)


    c +=1

    cone_msg.header.frame_id ="map"
    cone_msg.ADD
    cone_msg.SPHERE
    cone_msg.pose.position.x = x_cone
    cone_msg.pose.position.y = y_cone
    cone_msg.pose.position.z = 0
    cone_msg.pose.orientation.w = 1
    cone_msg.scale.x = 0.1
    cone_msg.scale.y = 0.1
    cone_msg.scale.z = 0.1
    cone_msg.color.a = 1
    cone_msg.id =id 
    id+=1
    if c_cone == "blue_cone":
      cone_msg.color.r = 0
      cone_msg.color.g = 0
      cone_msg.color.b = 255
      #cone_msg.mesh_resource = "package://aamfsd_description/meshes/cone_yellow.dae"
    else :
      cone_msg.color.r = 255
      cone_msg.color.g = 255
      cone_msg.color.b = 0
      #cone_msg.mesh_resource = "package://aamfsd_description/meshes/cone_yellow.dae"
    #cone_msg.type = Marker.MESH_RESOURCE
    #cone_msg.mesh_resource = "/home/abdelkader/aamfsd_2024/src/aam_cars/aamfsd_description/meshes/cone_blue.dae"
    #cone_msg.mesh_use_embedded_materials = True



    
    global_map.global_map_msg.markers.append(cone_msg)
  if global_map.loop_closure_flag and not global_map.sent_before:
    global_map.global_map_pub_1time.publish(global_map.global_map_msg)
    global_map.sent_before = True
    print("loop closure detected")
    print("map is sent here once el mafrod ya3ni")
  
  #global_map.id=id
  #global_map.global_map_covariance.publish(global_map.covariance)
  global_map.global_map_pub.publish(global_map.global_map_msg)




def broadcast_last_transform(global_map ):

  """ Make sure that we are always broadcasting the last map
      to odom transformation.  This is necessary so things like
      move_base can work properly. """
###################################################### translation and rotation gaieen mnen 
  global_map.tf_broadcaster.sendTransform(global_map.translation,
                                    global_map.rotation,
                                    rospy.get_rostime(),
                                    global_map.base_frame,
                                    global_map.map_frame)

  
def fix_map_to_odom_transform(global_map ):
  """ This method constantly updates the offset of the map and
      odometry coordinate systems based on the latest results from
      the localizer """

  (global_map.translation, global_map.rotation) = convert_pose_inverse_transform(global_map.xEst[0],global_map.xEst[1],global_map.xEst[2])    #small track +math.pi/2

  p = PoseStamped(pose= convert_translation_rotation_to_pose(global_map.translation,global_map.rotation),
                      header=Header(frame_id=global_map.base_frame))
  
  Data_type = object
 
# Now we fix the error
  global_map.odom_to_map = global_map.tf_listener.transformPose(global_map.base_frame, p,dtype=Data_type)


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



def ekf_slam(SLAM ):
  # xEst : Estimated State
  # PEst : The uncertainty in last position
  # u : control input
  # z : landmarks measurements at this step
  # S : state size

  S = 3
  
  # get landmark EKF format from cones positions
  z ,colors = get_landmarks(SLAM)


  ## Prediction ##
  predict(SLAM)


  initP = np.eye(2)

  ## Update ##
  #if SLAM.loop_closure_flag == False:
  update(z ,colors, initP ,SLAM )
  
  
  for i in range(calc_n_lm(SLAM.xEst,SLAM)):

    SLAM.mapx.append(SLAM.xEst[SLAM.state_size + i * 2,0])
    SLAM.mapy.append(SLAM.xEst[SLAM.state_size + i * 2 + 1,0])



  return SLAM.xEst ,SLAM.pEst , SLAM.mapx , SLAM.mapy ,SLAM.mapc


def get_landmarks(SLAM ):

  z = np.zeros((0, 3))
  colors =[]

  for i in range(len(SLAM.cones_x)):
    x = SLAM.cones_x[i]
    y = SLAM.cones_y[i]
    c = SLAM.cones_c[i]
    d = math.sqrt(x**2 + y**2)
    angle = pi_2_pi(math.atan2(y, x))  #small track +math.pi/2


    if d <= SLAM.MAX_RANGE:
      zi = np.array([d, angle, i])
      z = np.vstack((z, zi))
      colors.append(c)

  return z , colors


def pi_2_pi(angle):
  return (angle + math.pi) % (2 * math.pi) - math.pi


def predict(SLAM ):
  # xEst : Estimated State
  # u : control input
  S = SLAM.state_size
  G , Fx = jacob_motion( SLAM )

  SLAM.xEst[0:S] = motion_model(SLAM.xEst[0:S],SLAM)

  SLAM.pEst[0:S, 0:S] = np.dot( G.T , np.dot( SLAM.pEst[0:S, 0:S] , G )) + np.dot( Fx.T , np.dot(SLAM.Cx , Fx) )


def jacob_motion(SLAM ):

  """
  Calculates the jacobian of motion model. 
  Inputs:
  => x: The state, including the estimated position of the system
  => u: The control function
  Outputs: 
  => G:  Jacobian
  => Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
  """

  Fx = np.hstack(( np.eye(SLAM.state_size), np.zeros((SLAM.state_size,  SLAM.LM_SIZE * calc_n_lm(SLAM.xEst[0:3],SLAM))) ))


  gama = SLAM.angular_vel * SLAM.dt

  G = np.array([[1, 0.0,(SLAM.Vx*math.sin(SLAM.xEst[2]-gama)-SLAM.Vy*math.cos(SLAM.xEst[2]-gama))*SLAM.dt],
                  [0.0, 1,(SLAM.Vx*math.cos(SLAM.xEst[2]-gama)-SLAM.Vy*math.sin(SLAM.xEst[2]-gama))*SLAM.dt ],
                  [0.0, 0.0, SLAM.dt]], dtype=float)


  return G, Fx


def calc_n_lm(x,SLAM):
  n = int((len(x) - SLAM.state_size) / SLAM.LM_SIZE)
  return n


def motion_model(x, SLAM ):

  phi = SLAM.angular_vel * SLAM.dt
  x[0] = x[0] + (SLAM.Vx)*SLAM.dt   #  phi-math.pi/2  small track
  x[1] = x[1] + (SLAM.Vy)*SLAM.dt
  #x[2] = x[2] + phi
  x[2] = SLAM.yaw
  #x[2] = pi_2_pi(x[2])
  print("this is the yaw",x[2]*180/math.pi ,"this is the phi", phi)

  return x


def motion_model2(x, SLAM ):
    car_length = 1535
    l_r = car_length / 2

    v_resultant = math.sqrt(SLAM.Vx**2 + SLAM.Vy**2)
    #yaw_from_u = self.u[1,0]
    yaw_old = x[2]
    
    #??
    #sigma = yaw_from_u - yaw_old 
    sigma = SLAM.heading
	  #L is the car length,,,,sigma is the steering_angle
    
    if sigma != 0:
      S = car_length / math.tan(abs(sigma))
    else:
      S = np.iinfo(np.intc).max
    #beta is the slip angle ,, L_r is the distance from the back wheel to the center of gravity
    beta =  np.arctan(l_r * math.tan(sigma)/car_length)
    
    #R is the distance from the center of gravity to the center of the center of rotation
    R = S / math.cos(beta)
    
    yaw_rate = v_resultant / R
    
    x_rate = v_resultant * math.cos(beta + yaw_old)
    y_rate = v_resultant * math.sin(beta + yaw_old)
    
    x[0] =x[0] + x_rate* SLAM.dt
    x[1] =x[1] + y_rate* SLAM.dt
    x[2] =x[2] + yaw_rate* SLAM.dt
    x[2] = pi_2_pi(x[2])

    return x


def update(z , colors, initP, SLAM ):
  """
  Performs the update step of EKF SLAM
  
  :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
  :param PEst:  nxn the predicted covariance
  :param u:     2x1 the control function 
  :param z:     the measurements read at new position
  :param initP: 2x2 an identity matrix acting as the initial covariance
  :returns:     the updated state and covariance for the system
  """
  #print("map size",self.calc_n_lm(xEst))
  #print("number of land marks observed",len(z[:, 0]))

  state_size = calc_n_lm(SLAM.xEst,SLAM)*2+3
  observation_size = 2*len(z[:, 0])
  H=np.zeros((observation_size, state_size ))

  dz = np.zeros((2*len(z[:, 0]), 1))

  Rcx = np.zeros((2*len(z[:, 0]),2*len(z[:, 0])))
  i=0
  for iz in range(len(z[:, 0])):  # for each observation

    min_id = search_correspond_landmark_id(SLAM.xEst, SLAM.pEst, z[iz, 0:2] ,SLAM) # associate to a known landmark

    nLM = calc_n_lm(SLAM.xEst,SLAM)

    if min_id == nLM:
      # Extend state and covariance matrix
      xAug = np.vstack((SLAM.xEst, calc_landmark_position(SLAM.xEst, z[iz, :])))
      PAug = np.vstack((np.hstack((SLAM.pEst, np.zeros((len(SLAM.xEst), SLAM.LM_SIZE)))),
                        np.hstack((np.zeros((SLAM.LM_SIZE, len(SLAM.xEst))), initP))))
      SLAM.xEst = xAug
      SLAM.pEst = PAug
      SLAM.mapc.append(colors[i])

      H = np.hstack((H,np.zeros((observation_size,2))))

    lm = get_landmark_position_from_state(SLAM.xEst, min_id ,SLAM)

    dz, H = calc_innovation2(lm, SLAM.xEst, z[iz, 0:2], min_id, iz,H,dz)

    Rcx[iz*2, iz*2] = 0.2
    Rcx[iz*2+1, iz*2+1] = np.deg2rad(10.0)

    i+=1

  S = np.dot(np.dot(H,SLAM.pEst),H.T) + Rcx

  K = np.dot(np.dot(SLAM.pEst , H.T) , np.linalg.inv(S))   #  n*n  dot  n*2   dot  2*2   it returns n*2
  # print(np.shape(self.pEst))

  #k_y = np.dot(K , y)
  #k_y[0:3] = 0
  #self.xEst = self.xEst + k_y
  SLAM.xEst = SLAM.xEst + np.dot(K , dz)

  SLAM.pEst = np.dot( ( np.eye(len(SLAM.xEst)) -np.dot(K , H) ),SLAM.pEst)  #3AAAAAAAAAAAAAAAAA
    # self.pEst = np.eye(len(self.xEst)) - np.dot(np.dot(K , H),self.pEst)        

  #print("state after update" ,self.xEst[0:3].T )

  SLAM.xEst[2] = pi_2_pi(SLAM.xEst[2])

  return SLAM.xEst, SLAM.pEst


def search_correspond_landmark_id(xAug, PAug, zi ,SLAM ):
  """
  Landmark association with Mahalanobis distance
  """

  nLM = calc_n_lm(xAug,SLAM)
  
  min_dist = []

  for i in range(nLM):
    lm = get_landmark_position_from_state(xAug, i,SLAM)
    y, S, H = calc_innovation(lm, xAug, PAug, zi, i ,SLAM)
    min_dist.append(np.dot( y.T , np.dot( np.linalg.inv(S) , y ) ) )  #finds the disctance between all the landmarks and this landmark(zi)

  min_dist.append(SLAM.M_DIST_TH)  # new landmark       if the distance is more than the threshold then it is a new lm  if not then

  min_id = min_dist.index(min(min_dist))

  return min_id


def get_landmark_position_from_state( x, ind, SLAM ):
  lm = x[SLAM.state_size + SLAM.LM_SIZE * ind: SLAM.state_size + SLAM.LM_SIZE * (ind + 1), :]

  return lm


def calc_innovation(lm, xEst, PEst, z, LMid ,SLAM ):
  delta = lm - xEst[0:2]
  q = np.dot(delta.T , delta)[0, 0]
  z_angle = math.atan2(delta[1, 0], delta[0, 0])  - xEst[2, 0]
  zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
  y = (z - zp).T   # 2*1
  y[1] = pi_2_pi(y[1])
  H = jacob_h(q, delta, xEst, LMid + 1 ,SLAM)  # 2 * n    n: 3 + 2*nLM

  S = np.dot( H , np.dot(PEst , H.T) ) + SLAM.Cx[0:2, 0:2]  # 2*2

  return y, S, H


def calc_innovation2(lm, xEst, z, LMid , iz, H,dz):
  delta = lm - xEst[0:2]
  q = np.dot(delta.T , delta)[0, 0]
  z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
  #z_angle = math.atan2(delta[1, 0], delta[0, 0])
  #zp = np.array([[math.sqrt(q), self.pi_2_pi(z_angle)]])
  zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
  y = (z - zp).T   # 2*1
  y[1] = pi_2_pi(y[1])

  dz[2*iz : 2*iz+2]= y

  H = jacob_h2(q, delta,H, iz,LMid ) 
    # 2 * n    n: 3 + 2*nLM

  #print(H.shape,PEst.shape , H.T.shape ,self.Cx[0:2, 0:2].shape)
  #S = np.dot( H , np.dot(PEst , H.T) ) + self.Cx[0:2, 0:2]  # 2*2

  return dz, H


def jacob_h(q, delta, x, i, SLAM):
  sq = math.sqrt(q)
  G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])                  # 2*5

  G = G / q
  nLM = calc_n_lm(x,SLAM)
  F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
  F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                  np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

  F = np.vstack((F1, F2))   # 5* (3+nLm*2)

  H = np.dot( G , F )  #2* (3+nLm*2)

  return H


def jacob_h2(q,delta,H,iz,LMid):
  
  sq= math.sqrt(q)
  Hv = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0],
                  [delta[1, 0], - delta[0, 0], - q]])
  Hlm = np.array([[sq * delta[0, 0], sq * delta[1, 0]],
                  [- delta[1, 0], delta[0, 0]]])
  
  Hv=Hv/q
  Hlm=Hlm/q
  H[iz*2: iz*2 + 2 , 0:3] = Hv
  H[iz*2: iz*2 + 2 , LMid * 2 + 3 : LMid * 2 + 5] = Hlm

  return H


def calc_landmark_position(x, z):
  zp = np.zeros((2, 1))

  zp[0, 0] = x[0, 0] +( z[0] * math.cos(x[2, 0] + z[1] ) )
  zp[1, 0] = x[1, 0] +( z[0] * math.sin(x[2, 0] + z[1] ) )

  return zp


def mainFun(global_map ):

  #global_map.delta_yaw = global_map.prev_yaw - global_map.current_yaw #not used

  SLAM = Ekf(global_map.xEst,global_map.pEst,global_map.mapc,global_map.Xcones_filterd,global_map.Ycones_filterd,global_map.Ccones_filterd,global_map.delta_yaw,global_map.prev_yaw,global_map.angular_vel,global_map.control_steering_angle,global_map.control_velocity,global_map.Vx,global_map.Vy,global_map.start_time,global_map.Q,global_map.loop_closure_flag ,global_map.heading ,global_map.yaw )

  global_map.start_time = rospy.get_rostime().to_sec()
  
  global_map.xEst , global_map.pEst , global_map.mapx , global_map.mapy , global_map.mapc = ekf_slam(SLAM)

  #global_map.prev_yaw = global_map.current_yaw #not used

  #print("x is : ",global_map.xEst[0],"   y is: ",global_map.xEst[1])


  global_map.car_pose_x.append(float(global_map.xEst[0]))
  global_map.car_pose_y.append(float(global_map.xEst[1]))
  global_map.car_pose_yaw.append(float(global_map.xEst[2]))


  check_motion(global_map)
  if global_map.started_motion and (global_map.loop_closure_flag ==False):
    check_closure2(global_map)
 
  csv_map(global_map) 

  fix_map_to_odom_transform(global_map)
  broadcast_last_transform(global_map)

  # Visualize the logged path and global map on rviz
  #file_saved(global_map)
  #save_map(global_map)
  print(global_map.xEst)
  viz_path(global_map)
  viualise_global_map(global_map)
  

  #global_map.start_time = rospy.get_rostime().to_sec()
  




if __name__ == '__main__':
  try:
    global_map = Sensors()
    while not rospy.is_shutdown():
      mainFun(global_map)
  except rospy.ROSInterruptException:
    pass
  rospy.spin()
    
