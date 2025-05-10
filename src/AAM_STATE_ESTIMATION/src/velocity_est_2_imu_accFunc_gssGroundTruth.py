#!/usr/bin/env python3

#ay habd fel 3bd
from fcntl import F_GETLEASE
from os import path
from re import U
from types import LambdaType
from unittest import skip
import math
from numpy.core.fromnumeric import shape
from numpy.lib.function_base import blackman
import rospy
#from aam_common_msgs.msg import Cone
#from aam_common_msgs.msg import Map
#from aam_common_msgs.msg import ConeDetections
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
#import time 
import matplotlib.pyplot as plt 
from nav_msgs.msg import Odometry
import tf 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
#from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header, String, ColorRGBA
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
import numpy as np
import random
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class sensor():
   def __init__(self, namespace='data_gathering'):
      rospy.init_node("ekf_velocity", anonymous = True)
      
      self.pub = rospy.Publisher('vel_ekf', Float32MultiArray, queue_size=1)
      
      rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
      rospy.Subscriber("/aamfsd/joint_states",JointState,self.wheel_callback)
      rospy.Subscriber("/ground_truth/state_raw",Odometry,self.odom_callback)
      #rospy.Subscriber("/global_map_1time",MarkerArray,self.map_callback)
      
      self.xEst = np.zeros((2,1))
      self.pEst = np.eye(2)
      self.ax = 0
      self.ay = 0
      self.angular_vel =0

      self.yaw = 0   # spawn angle  acceleration , small , big  yaw =0      skid = 1.57079632679

      self.Vx = 0
      self.Vy = 0
      self.old_Vx =0
      self.old_Vy=0

      self.start_time = rospy.get_rostime().to_sec()
      self.dt=0

      self.truthx =[]
      self.truthy =[]
      self.z=np.array([[self.Vx],[self.Vy]])
      self.xEstHistx=[]
      self.xEstHisty=[]

      self.Front_Left_steering=0
      self.Front_Right_steering=0
      self.Heading = 0

      self.stopFlag=0
      
      self.time=[]

   #def imu_callback(self, msg):
      #self.ax = msg.linear_acceleration.x
      #self.ay = msg.linear_acceleration.y
	    #print("ax = :" ,self.ax)
	    #print("aYYY = :" ,self.ay)
        

   def odom_callback(self,odom_msg):
        self.Vx = odom_msg.twist.twist.linear.x
        self.Vy = odom_msg.twist.twist.linear.y
        self.z=np.array([[self.Vx],[self.Vy]])

   def imu_callback(self, msg):
        self.angular_vel = msg.angular_velocity.z

   def wheel_callback(self,msg):
        # needs to be multiled by the wheel radius   dia = 0.505

        self.Front_Left_steering  = msg.position[4]
        self.Front_Right_steering = msg.position[9]
        self.Heading =  (self.Front_Left_steering + self.Front_Right_steering) /2.0
        #print(self.Heading , "headinggggggg")

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def acc(s):
    if s.dt!=0:
      
      s.ax = (s.Vx- s.old_Vx)/s.dt
      s.ay = (s.Vy- s.old_Vy)/s.dt
      s.old_Vx = s.Vx
      s.old_Vy = s.Vy
      #print("this is prediction  ax = :" ,s.ax)
      #print("this is prediction  aaaYYY = :" ,s.ay)

def prediction(s):
    '''x = s.Vx
    y = s.Vy
    s.xEst[0] = random.gauss(s.Vx , s.Vx * 0.4)
    s.xEst[1] = random.gauss(s.Vy , s.Vy * 0.4)

    s.pEst = s.pEst + np.array([[0.4 , 0],
                                [0 , 0.4]])
    '''
    acc(s)
    s.xEst[0] = s.xEst[0]+s.dt*s.ax
    s.xEst[1] = s.xEst[1]+s.dt*s.ay
    s.yaw += s.dt * s.angular_vel
    s.yaw = pi_2_pi(s.yaw)
    #print(s.yaw , "yawwwww")

    s.pEst = s.pEst + np.array([[0.005 , 0],
                                [0 , 0.005]])
    #print("this is prediction  vx" ,s.xEst[0])
    #print("this is prediction  vyyy" ,s.xEst[1])   

def update(s):
    
    y = s.z -  s.xEst
    S = s.pEst+ np.array([[0.05,0],[0,0.05]])
    K= np.dot(s.pEst, np.linalg.inv(S))
    s.xEst= s.xEst + np.dot(K,y)
    s.pEst = np.dot(np.eye(2)-(K),s.pEst)

    #print("this is update  vx" ,s.xEst[0])
    #print("this is update  vyyy" ,s.xEst[1])
'''
def file_saved1(s ):
  i=0
  f= open(r'/home/sambo/dataEst.txt', 'w')
  f.write("X positions \n")
  while i<len(s.xEstHistx):
    x= s.xEstHistx[i]
    f.write( str(x)+"\n")
    i= i+1

  f.write("Y positions\n")
  i=0
  while i<len(s.xEstHisty):
    y= s.xEstHisty[i] 
    f.write(str(y)+"\n")
    i= i+1

  f.close()

def file_saved2(s ):
  i=0
  f= open(r'/home/sambo/dataTruth.txt', 'w')
  f.write("X positions \n")
  while i<len(s.truthx):
    x= s.truthx[i]
    f.write( str(x)+"\n")
    i= i+1

  f.write("Y positions\n")
  i=0
  while i<len(s.truthy):
    y= s.truthy[i] 
    f.write(str(y)+"\n")
    i= i+1

  f.close()
  
def save_time(s):
  i=0
  f= open(r'/home/sambo/time_tick.txt', 'w')
  while i<len(s.time):
    x = s.time[i]
    f.write(str(x)+"\n")
    i=i+1
  f.close()
'''
def mainfun(s):
 
    s.dt = rospy.get_rostime().to_sec() - s.start_time
    s.start_time = rospy.get_rostime().to_sec()
    prediction(s)
    update(s)

    s.xEstHistx.append(s.xEst[0,0])
    s.xEstHisty.append(s.xEst[1,0])
    s.truthx.append(s.Vx)
    s.truthy.append(s.Vy)
    s.time.append(s.dt)
    #vel = [s.Vx,s.Vy]
    #vel =[s.Vx , s.Vy]
    mean = 0
    std_dev = 0.1  # Adjust this value to control the amount of noise

# Adding Gaussian noise
    noise = np.random.normal(mean, std_dev)
    thing = Float32MultiArray()
    aaa = [s.Vx ,s.Vy,s.Heading,s.yaw]
    '''thing.label[0] = "x"
    thing.size[0] = 1
    thing.stride[0] = s.Vx
    thing2.label[1] = "y"
    thing2.size[1] = 1
    thing2.stride[1] = s.Vy
    arg.layout.dim.append(thing)'''
    thing.data = aaa		

    s.pub.publish(thing)
    
    #if s.Vx>1 or s.Vy>1:
    
    	#file_saved1(s)
    	#file_saved2(s)
    	#save_time(s);
	
    
 

if __name__ == '__main__':
    try:
        
        s = sensor()
        while True: #main functions
            mainfun(s)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()



