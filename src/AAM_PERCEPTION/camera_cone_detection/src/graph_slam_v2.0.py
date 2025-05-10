#!/usr/bin/env python

#ay habd fel 3bd new algorithm <3
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion,PoseWithCovariance
from std_msgs.msg import Float32MultiArray
import statistics
from tf2_geometry_msgs import PointStamped
import geometry_msgs.msg
import tf2_ros
import copy
import math
import itertools
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import copy
import itertools
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

#####################################################################################################################################################################



class Sensors():
  def __init__(self, namespace='data_gathering'):

    rospy.init_node("Graph_slam", anonymous = True)

    #rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,callback=self.cones_callback)
    rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,callback=self.cones_callback)
    #rospy.Subscriber("/cam_lidar_fusion_cones",MarkerArray, self.cones_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
    #rospy.Subscriber("/ground_truth/state",Odometry,self.odom_callback)
    rospy.Subscriber("/vel_ekf",Float32MultiArray,self.odom_callback)

    

    self.car_pose_pub = rospy.Publisher("/car_pose_graph", Path, queue_size=5)
    self.global_map_pub = rospy.Publisher("/global_map_graph",MarkerArray, queue_size=5)
    self.global_map_covariance = rospy.Publisher("/global_map_covariance",PoseWithCovarianceStamped, queue_size=5)
    self.global_map_pub_1time = rospy.Publisher("/global_map_1time_graph",MarkerArray, queue_size=5)
  
   


    # for transformation
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    # Wait for the transform between the frames to become available
    self.tf_buffer.can_transform("base_link", "odom", rospy.Time(), rospy.Duration(1))
    
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
    self.hz = []
    self.hz_h = []



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
    # the frame of the zed2i
    self.zed_frame = "zed_frame"
     

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
      
      self.cones_x.append(cone.pose.position.x+1.7)
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
    self.heading = odom_msg.data[2]
    self.yaw = odom_msg.data[3]
    # print(self.Vx)
    # print(self.Vy)



#####################################################################################################################################################################

class GRAPH():


  def __init__(self,xEst,pEst,mapc ,cones_x, cones_y,cones_c,delta_yaw,prev_yaw,angular_vel,control_steering_angle,control_velocity,Vx,Vy,start_time,Q,loop_closure_flag,heading,yaw,hz_h):
    

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
    self.STATE_SIZE =3
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
    self.STATE_SIZE = 3  # State size [x,y,yaw]

 # Covariance parameter of Graph Based SLAM
    self.C_SIGMA1 = 0.1
    self.C_SIGMA2 = 0.1
    self.C_SIGMA3 = np.deg2rad(1.0)

    self.MAX_ITR = 20  # Maximum iteration
    self.show_graph_d_time = 20.0  # [s]
    self.show_animation = True
    self.hxTrue = []
    self.hxDR = []
    self.hz = []
    self.d_time = 0.0
    self.init = False
    self.SIM_TIME = 100
    self.time = 0
    self.S=3

#####################################################################################################################################################################

def graph_slam(SLAM):
    


        #SLAM.time += SLAM.dt
        #SLAM.d_time += SLAM.dt
        #u = calc_input()
        #SLAM.hz=hz_h 
        SLAM.xEst[0:SLAM.S] = motion_model(SLAM.xEst[0:SLAM.S],SLAM)
        z = observation(SLAM)
        SLAM.hz.append(z)
        SLAM.xEst = graph_based_slam(SLAM.xEst, SLAM.hz,SLAM)
        print("slamxest",SLAM.xEst,SLAM.hz)
        #SLAM.d_time = 0.0


  
   # for i in range(calc_n_lm(SLAM.xEst,SLAM)):

    #  SLAM.mapx.append(SLAM.xEst[SLAM.state_size + i * 2,0])
     # SLAM.mapy.append(SLAM.xEst[SLAM.state_size + i * 2 + 1,0])
     
        return SLAM.xEst , SLAM.hz#, SLAM.mapx , SLAM.mapy ,SLAM.mapc

#####################################################################################################################################################################


def calc_n_lm(x,SLAM):
  n = int((len(x) - SLAM.state_size) / SLAM.LM_SIZE)
  return n



#####################################################################################################################################################################

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
    #visualise_global_map(global_map)


#####################################################################################################################################################################
#####################################################################################################################################################################


def broadcast_last_transform(global_map ):

  """ Make sure that we are always broadcasting the last map
      to odom transformation.  This is necessary so things like
      move_base can work properly. """
###################################################### translation and rotation gaieen mnen 
  global_map.tf_broadcaster.sendTransform(global_map.translation,
                                    global_map.rotation,
                                    rospy.get_rostime(),
                                    global_map.base_frame,
                                    global_map.odom_frame)

  
def fix_map_to_odom_transform(global_map ):
  """ This method constantly updates the offset of the map and
      odometry coordinate systems based on the latest results from
      the localizer """

  (global_map.translation, global_map.rotation) = convert_pose_inverse_transform(global_map.xEst[0],global_map.xEst[1],global_map.xEst[2])    #small track +math.pi/2

  p = PoseStamped(pose= convert_translation_rotation_to_pose(global_map.translation,global_map.rotation),
                      header=Header(frame_id=global_map.base_frame))
  
  global_map.odom_to_map = global_map.tf_listener.transformPose(global_map.base_frame, p)


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

#####################################################################################################################################################################

class Edge:

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0


def cal_observation_sigma(SLAM):
    sigma = np.zeros((3, 3))
    sigma[0, 0] = SLAM.C_SIGMA1 ** 2
    sigma[1, 1] = SLAM.C_SIGMA2 ** 2
    sigma[2, 2] = SLAM.C_SIGMA3 ** 2

    return sigma


def calc_3d_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = 0

    Rt1 = calc_3d_rotational_matrix(tangle1)
    Rt2 = calc_3d_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma()
    sig2 = cal_observation_sigma()

    #edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)
    edge.omega = np.linalg.inv(np.dot(Rt1 , np.dot( sig1 , Rt1.T)) + np.dot( Rt2 , np.dot( sig2 , Rt2.T)))

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(x_list, z_list):
    edges = []
    cost = 0.0
    z_ids = list(itertools.combinations(range(len(z_list)), 2))

    for (t1, t2) in z_ids:
        x1, y1, yaw1 = x_list[0, t1], x_list[1, t1], x_list[2, t1]
        x2, y2, yaw2 = x_list[0, t2], x_list[1, t2], x_list[2, t2]

        if z_list[t1] is None or z_list[t2] is None:
            continue  # No observation

        for iz1 in range(len(z_list[t1][:, 0])):
            for iz2 in range(len(z_list[t2][:, 0])):
                if z_list[t1][iz1, 3] == z_list[t2][iz2, 3]:
                    d1 = z_list[t1][iz1, 0]
                    angle1, _ = z_list[t1][iz1, 1], z_list[t1][iz1, 2]
                    d2 = z_list[t2][iz2, 0]
                    angle2, _ = z_list[t2][iz2, 1], z_list[t2][iz2, 2]

                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, d2, angle2, t1, t2)

                    edges.append(edge)
                    cost += np.dot(edge.e.T,np.dot( edge.omega , edge.e))[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


def fill_H_and_b(H, b, edge,SLAM):
    A, B = calc_jacobian(edge)

    id1 = edge.id1 *SLAM.STATE_SIZE
    id2 = edge.id2 * SLAM.STATE_SIZE

    H[id1:id1 + SLAM.STATE_SIZE, id1:id1 + SLAM.STATE_SIZE] += np.dot(A.T, np.dot( edge.omega , A))
    H[id1:id1 + SLAM.STATE_SIZE, id2:id2 + SLAM.STATE_SIZE] += np.dot(A.T , np.dot(edge.omega , B))
    H[id2:id2 + SLAM.STATE_SIZE, id1:id1 + SLAM.STATE_SIZE] += np.dot(B.T ,np.dot( edge.omega , A))
    H[id2:id2 + SLAM.STATE_SIZE, id2:id2 + SLAM.STATE_SIZE] += np.dot(B.T , np.dot(edge.omega , B))

    b[id1:id1 + SLAM.STATE_SIZE] += (np.dot(A.T , np.dot(edge.omega , edge.e)))
    b[id2:id2 + SLAM.STATE_SIZE] += (np.dot(B.T , np.dot(edge.omega , edge.e)))


    return H, b

#######################################################################################################################################################################
def graph_based_slam(x_init, hz,SLAM):
    print("start graph based slam")

    z_list = copy.deepcopy(hz)

    x_opt = copy.deepcopy(x_init)
    nt = x_opt.shape[1]
    n = nt * SLAM.STATE_SIZE

    for itr in range(SLAM.MAX_ITR):
        edges = calc_edges(x_opt, z_list)

        H = np.zeros((n, n))
        b = np.zeros((n, 1))

        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        # to fix origin
        H[0:SLAM.STATE_SIZE, 0:SLAM.STATE_SIZE] += np.identity(SLAM.STATE_SIZE)

        dx = - np.dot(np.linalg.inv(H) , b)

        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = (np.dot(dx.T , dx))[0, 0]
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    return x_opt

#######################################################################################################################################################################
def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u
#######################################################################################################################################################################

def observation(SLAM ):

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
  print("before returning observation this is slam",SLAM.cones_x,SLAM.cones_y)

  return z 
#######################################################################################################################################################################

def motion_model(x, SLAM ):

  phi = SLAM.angular_vel * SLAM.dt
  x[0] = x[0] + (SLAM.Vx)*SLAM.dt   #  phi-math.pi/2  small track
  x[1] = x[1] + (SLAM.Vy)*SLAM.dt
  #x[2] = x[2] + phi
  x[2] = SLAM.yaw
  #x[2] = pi_2_pi(x[2])
  print("this is the yaw",x[2]*180/math.pi ,"this is the phi", phi)

  return x


#######################################################################################################################################################################
def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

#######################################################################################################################################################################
def main(global_map):
    print(" starting graph slam!!")
   
    SLAM = GRAPH(global_map.xEst,global_map.pEst,global_map.mapc,global_map.Xcones_filterd,global_map.Ycones_filterd,global_map.Ccones_filterd,global_map.delta_yaw,global_map.prev_yaw,global_map.angular_vel,global_map.control_steering_angle,global_map.control_velocity,global_map.Vx,global_map.Vy,global_map.start_time,global_map.Q,global_map.loop_closure_flag ,global_map.heading ,global_map.yaw,global_map.hz_h)

    global_map.start_time = rospy.get_rostime().to_sec()
  
    global_map.xEst,global_map.hz  = graph_slam(SLAM)
    #, global_map.pEst , global_map.mapx , global_map.mapy , global_map.mapc 
   
  #global_map.prev_yaw = global_map.current_yaw #not used

  #print("x is : ",global_map.xEst[0],"   y is: ",global_map.xEst[1])


    global_map.car_pose_x.append(float(global_map.xEst[0]))
    global_map.car_pose_y.append(float(global_map.xEst[1]))
    global_map.car_pose_yaw.append(float(global_map.xEst[2]))
    global_map.hz_h.append(global_map.hz)


    check_motion(global_map)
    if global_map.started_motion and (global_map.loop_closure_flag ==False):
     check_closure2(global_map)
 
    #csv_map(global_map) 
    #transformation_new_method(global_map)

    fix_map_to_odom_transform(global_map)
    broadcast_last_transform(global_map)

  # Visualize the logged path and global map on rviz
  #file_saved(global_map)
  #save_map(global_map)
  #print(global_map.pEst)
    viz_path(global_map)
    #visualise_global_map(global_map)

  #global_map.start_time = rospy.get_rostime().to_sec()
  




def viz_path(global_map ):
  path_msg = Path()
  path_msg.header.frame_id = 'odom'
  
  i = 0

  


  while i < len(global_map.car_pose_x):
    #print("vis path")
    pose_msg = PoseStamped()

    pose_msg.pose.position.x = global_map.car_pose_x[i]
    pose_msg.pose.position.y = global_map.car_pose_y[i]
    pose_msg.pose.orientation.w = global_map.car_pose_yaw[i]
    path_msg.header.frame_id = 'odom'
    path_msg.poses.append(pose_msg)



    i+=1
    

  
  global_map.car_pose_pub.publish(path_msg)
#######################################################################################################################################################################

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

    cone_msg.header.frame_id = "odom"
    cone_msg.ADD
    cone_msg.SPHERE
    cone_msg.pose.position.x = x_cone
    cone_msg.pose.position.y = y_cone
    cone_msg.pose.position.z = 0
    cone_msg.pose.orientation.w = 1
    cone_msg.scale.x = 1
    cone_msg.scale.y = 1
    cone_msg.scale.z = 1
    cone_msg.color.a = 1
    cone_msg.id =id 
    id+=1
    if c_cone == "blue_cone":
      cone_msg.color.r = 0
      cone_msg.color.g = 0
      cone_msg.color.b = 255
      cone_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
    else :
      cone_msg.color.r = 255
      cone_msg.color.g = 255
      cone_msg.color.b = 0
      cone_msg.mesh_resource = "package://aamfsd_description/meshes/cone_yellow.dae"
    cone_msg.type = Marker.MESH_RESOURCE
    cone_msg.mesh_use_embedded_materials = True



    
    global_map.global_map_msg.markers.append(cone_msg)
  if global_map.loop_closure_flag and not global_map.sent_before:
    global_map.global_map_pub_1time.publish(global_map.global_map_msg)
    global_map.sent_before = True
    print("map is sent here once el mafrod ya3ni")
  
  #global_map.id=id
  #global_map.global_map_covariance.publish(global_map.covariance)
  global_map.global_map_pub.publish(global_map.global_map_msg)

 #######################################################################################################################################################################


if __name__ == '__main__':
  try:
    global_map = Sensors()
    print(" starting graph slam!!")

    while not rospy.is_shutdown():
      main(global_map)
  except rospy.ROSInterruptException:
    pass
  rospy.spin()

#######################################################################################################################################################################