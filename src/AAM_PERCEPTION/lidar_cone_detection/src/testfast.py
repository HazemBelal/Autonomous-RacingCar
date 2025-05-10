#!/usr/bin/env python3
import pandas as pd
import math
import string
import matplotlib.pyplot as plt
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import csv
def cones_callback(msg):
    with open('/home/abdelkader/Desktop/aamfsd_2024/src/aam_perception/lidar_cone_detection/src/test.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'color'])
        for marker in msg.markers:
            writer.writerow([float(marker.pose.position.x),float(marker.pose.position.y),"red"])
            
        
  

if __name__ == '__main__':
    try:
        rospy.init_node("test_fast", anonymous = True)
        cones_sub = rospy.Subscriber("/test", MarkerArray, cones_callback)
       
       
    except rospy.ROSInterruptException:
        pass
    rospy.spin()       