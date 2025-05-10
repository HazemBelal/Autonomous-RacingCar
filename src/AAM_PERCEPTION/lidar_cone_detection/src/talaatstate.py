#!/usr/bin/env python3
import csv
import math
import string
import matplotlib.pyplot as plt
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.stats import multivariate_normal
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
csv_file_path = "/home/abdelkader/Desktop/aamfsd_2024/src/aam_perception/lidar_cone_detection/src/output.csv"
class printdata:
    def __init__(self):
        self.vxwheel=0
        self.vywheel=0
        self.vxoptical=0
        self.vyoptical=0
        self.vxground=0
        self.vyground=0
        self.keys=[0,0,0]
        rospy.init_node("hsv_node", anonymous = True,disable_signals=True)
        self.commandcontrol_sub = rospy.Subscriber("/vel_ekf",Float32MultiArray, self.control_callback)
        self.subsoptical=rospy.Subscriber("/optical_flow", Float32MultiArray, self.optical_flow_callback)
        rospy.Subscriber('/ground_truth/state',Odometry,self.restore,queue_size=1)
        
    def restore(self,real):
        if self.keys[0] != -1:
            self.keys[0] = 0
            self.vxground=real.twist.twist.linear.x
            self.vyground=real.twist.twist.linear.y
            self.keys[0] = 1
            self.prdara()

    def control_callback (self,controls):
        if self.keys[1] != -1:
            self.keys[1] = 0
            self.vxwheel=controls.data[1]
            self.vywheel=-controls.data[0]
            self.keys[1]=1
            self.prdara()
    def optical_flow_callback(self,controls):
        if self.keys[2] != -1:
            self.keys[2] = 0
            self.vxoptical=controls.data[0]
            self.vyoptical=controls.data[1]
            self.keys[2]=1
            self.prdara()
    def prdara(self):
        if self.keys[0] == 1 and self.keys[1] == 1 and self.keys[2]==1:
                self.keys = [-1, -1,-1]
                with open(csv_file_path, mode='a', newline='') as file:
            
                    writer = csv.writer(file)
                    row=[self.vxwheel,self.vywheel,self.vxoptical,self.vyoptical,self.vxground,self.vyground,rospy.Time.now().to_sec()]
                    writer.writerow(row)
                    print(row)

                self.keys=[0,0,0]
                        







if __name__ == '__main__':
    try:
        csv_file_path = "/home/abdelkader/Desktop/aamfsd_2024/src/aam_perception/lidar_cone_detection/src/output.csv"
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["vx wheel encoder","vy wheel encoder","vx optical flow","vy optical flow","vx ground truth","vy ground truth","Time Stamp"])
        printdata=printdata()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()          