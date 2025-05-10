#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import math
class Test:
    def __init__(self):
        rospy.init_node("Imu_test", anonymous=True)
        rospy.Subscriber('/zed/zed_node/imu/data', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/imu_vel', Float32MultiArray, queue_size=10)
        self.Vx = 0
        self.Vy = 0
        self.yaw = 0
        self.heading=0


    def imu_callback(msg,self):
        linear_acceleration = msg.linear_acceleration  
        self.Vx = linear_acceleration.x
        self.Vy = linear_acceleration.y

        orientation_quat = msg.orientation
        orientation_euler = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.yaw = orientation_euler[2]

def mainfun(T):
    arr = Float32MultiArray()
    T.heading=math.atan2(T.Vy,T.Vx)
    V = [T.Vx,T.Vy,T.yaw,T.heading]
    print("velocities", V)
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
