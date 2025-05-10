#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import rospy
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
velprevx=0
velprevy=0
t_prev=0
firsttime=0
realspeedx=0
realspeedy=0
pub = rospy.Publisher("/vx", Float64, queue_size=1)
pub2 = rospy.Publisher("/vy", Float64, queue_size=1)
pub3 = rospy.Publisher("/errorx", Float64, queue_size=1)
pub4 = rospy.Publisher("/errory", Float64, queue_size=1)

def restore(real):
    global realspeedx,realspeedy
    realspeedx=real.twist.twist.linear.x
    realspeedy=real.twist.twist.linear.y
def callback (acc):
   global velprevx,t_prev,firsttime,velprevy
   
   if(firsttime==0):
       t_prev=rospy.Time.now().to_sec()
       firsttime=1
   else:
    accx=acc.linear_acceleration.x
    accy=acc.linear_acceleration.y    
    orientation_list = [acc.orientation.x, acc.orientation.y, acc.orientation.z, acc.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)
    accxans=accx
    accyans=accy
    t_current= rospy.Time.now().to_sec() 
    currentvelx=velprevx+((accxans)*(t_current-t_prev))
    currentvely=velprevy+((accyans)*(t_current-t_prev))
    t_prev=t_current
    velprevx=currentvelx
    velprevy=currentvely
    vx=Float64()
    vy=Float64()
    errorx=Float64()
    errory=Float64()
    vx.data=-currentvely
    vy.data=currentvelx
    errorx=abs(vx.data-realspeedx)
    errory=abs(vy.data-realspeedy)
    pub3.publish(errorx)
    pub4.publish(errory)
    velprevx=currentvelx
    velprevy=currentvely
    pub.publish(vx)
    pub2.publish(vy)
def main1():
    rospy.init_node('vodom',anonymous =False)
    global rate
    rate=rospy.Rate(15)
    rospy.Subscriber('/sensor_imu_hector',Imu,callback,queue_size=1)
    rospy.Subscriber('/ground_truth/state',Odometry,restore,queue_size=1)
    rospy.spin()
if __name__ == '__main__':
    main1()   
