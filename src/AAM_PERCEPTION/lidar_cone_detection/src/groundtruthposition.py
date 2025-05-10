#!/usr/bin/env python3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import rospy
pub = rospy.Publisher("/xpos", Float64, queue_size=1)
pub3 = rospy.Publisher("/ypos", Float64, queue_size=1)
def restore(real):
    realposex=real.pose.pose.position.x-26.29
    realposey=real.pose.pose.position.y-29.80
    pxavg=Float64()
    pyavg=Float64()
    pxavg.data=realposex
    pyavg.data=realposey
    pub.publish(pxavg)
    pub3.publish(pyavg)
    

def main1():
    rospy.init_node('vodom',anonymous =False)
    rospy.Subscriber('/ground_truth/state',Odometry,restore,queue_size=1)
    rospy.spin()
if __name__ == '__main__':
    main1()
