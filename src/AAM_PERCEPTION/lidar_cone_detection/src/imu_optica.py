#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from tf.transformations import euler_from_quaternion
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import math
import numpy as np

class ZEDVelocityEstimator:


        def __init__(self):
            rospy.init_node('zed_vel',anonymous =False)
            self.zed_odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.zed_odom_callback)
            rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.imu_callback)
            rospy.Subscriber("/optical_flow", Float32MultiArray, self.optical_flow_callback)

            #self.zed_velocity_pub = rospy.Publisher('/zed/velocity', Odometry, queue_size=10)
            self.pub = rospy.Publisher("/vel_ekf", Float32MultiArray, queue_size=10)
        

            self.zed_odom_prev = None
            self.zed_velocity = Odometry()
            self.Vel_flow = np.array([0, 0])
            self.yaw = 0.0           # Initialize the state vector
            self.state = np.zeros((2,1))
            self.zz = np.zeros((2,1))

            # Initialize the covariance matrix
            self.covariance = np.eye(2)
            self.heading=0
         

        def zed_odom_callback(self, msg):
            """Callback for the /zed/odom topic.

            Computes the velocity of the ZED from successive /zed/odom messages.

            Args:
                msg: A nav_msgs/Odometry message containing the odometry of the ZED.
            """

            # Compute the velocity of the ZED from successive /zed/odom messages.
            if self.zed_odom_prev is not None:
                dt = (msg.header.stamp - self.zed_odom_prev.header.stamp).to_sec()
                dx = msg.pose.pose.position.x - self.zed_odom_prev.pose.pose.position.x
                dy = msg.pose.pose.position.y - self.zed_odom_prev.pose.pose.position.y

                self.zed_velocity.twist.twist.linear.x = dx / dt
                self.zed_velocity.twist.twist.linear.y = dy / dt
                


            # Publish the velocity of the ZED.
            self.zed_velocity.header.stamp = rospy.Time.now()
            self.zed_velocity.header.frame_id = 'base_link'
            #self.zed_velocity_pub.publish(self.zed_velocity)

            # Save the current ZED odometry message for the next callback.
            self.zed_odom_prev = msg

        def optical_flow_callback(self, optical_flow_msg):
            self.Vel_flow = [optical_flow_msg.data[0], optical_flow_msg.data[1]]  
            self.zz[0:2,0] =[[-self.optical_flow_msg.data[1]], [self.optical_flow_msg.data[0]]]

        def imu_callback(self,msg):
        
            orientation_quat = msg.orientation
            orientation_euler = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
            self.yaw = orientation_euler[2]    

            
def predict(z):
    z.state[0]= z.state[0]+z.zed_velocity.twist.twist.linear.x
    z.state[1]= z.state[1]+z.zed_velocity.twist.twist.linear.y

    z.covariance = z.covariance+ np.array([[0.005, 0],
                                          [0, 0.005]])
def update(z):

    y = z.zz - z.state
    S = z.covariance + np.array([[0.1063958, 0], [0, 0.1063958]])
    K = np.dot(z.covariance, np.linalg.inv(S))
    z.state = z.state + np.dot(K, y)
    z.covariance = np.dot(np.eye(2) - (K), z.covariance)

def mainfun(z):
    
    predict(z)
    update(z)
    

    arr = Float32MultiArray()
    z.heading=math.atan2(z.state[0,0],z.state[1,0])
  
    
    V = [z.state[0],z.state[1],z.yaw,z.heading]
    print("velocities", V)
    
    
    arr.data = V
    z.pub.publish(arr)
    #z.ratee.sleep()
           

'''  if __name__ == '__main__':
        rospy.init_node('zed_velocity_estimator')
        zed_velocity_estimator = ZEDVelocityEstimator()
        rospy.spin()#!/usr/bin/env python3'''

if __name__ == '__main__':
    try:

        z=ZEDVelocityEstimator()
        while not rospy.is_shutdown():
            mainfun(z)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
