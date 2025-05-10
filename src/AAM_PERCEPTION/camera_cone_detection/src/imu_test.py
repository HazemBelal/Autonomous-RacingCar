#!/usr/bin/env python3
from os import write
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from tf.transformations import euler_from_quaternion
import tf
from tf.transformations import euler_from_quaternion
import csv
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import math
class Test:
    def __init__(self):
        rospy.init_node("Imu_test", anonymous=True)
        rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.imu_callback)
        self.pub = rospy.Publisher('imu_vel', Float32MultiArray, queue_size=10)
        self.Vx = 0
        self.Vy = 0
        self.yaw = 0
        self.ax = 0
        self.ay = 0
        self.heading=0
        self.linear_acceleration=0
        self.old_Vx = 0
        self.old_Vy = 0
        self.dt = 0
        self.start_time = rospy.get_rostime().to_sec()
        self.time = []
        self.roll=0
        self.pitch=0
        self.yaw=0


    def imu_callback(self,msg):
        linear_acceleration = msg.linear_acceleration  
        self.ax = linear_acceleration.x
        self.ay= linear_acceleration.y

        '''orientation_quat = msg.orientation
        orientation_euler = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.yaw = orientation_euler[2]
        explicit_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(explicit_quat)'''

    

def vel(T):
    if T.dt != 0 :

        T.Vx = (T.ax-(0.04)) *T.dt+T.old_Vx 
        T.Vy= (T.ay-(-0.04741)) *T.dt+T.old_Vy
        T.old_Vx = T.Vx 
        T.old_Vy = T.Vy    

def mainfun(T):
    T.dt = rospy.get_rostime().to_sec() - T.start_time
    T.start_time = rospy.get_rostime().to_sec()
    T.time.append(T.dt)
    arr = Float32MultiArray()
    T.heading=math.atan2(T.Vy,T.Vx)
    vel(T)
    V = [T.Vx,T.Vy,T.yaw,T.heading]
    print(V)
    arr.data = V
    T.pub.publish(arr)

if __name__ == '__main__':
    try:
        
        T= Test()

        while not rospy.is_shutdown():
            mainfun(T)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
