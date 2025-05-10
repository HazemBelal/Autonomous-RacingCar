#!/usr/bin/env python3
from cmath import nan
from email.mime import image
from gc import disable
from unittest import skip
import rospy
import cv2
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import colorsys
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import math
class hsvfilter():
    def __init__(self):
        rospy.init_node("hsv_node", anonymous = True,disable_signals=True)
        self.image_sub = rospy.Subscriber("/zed/left/image_color",Image,self.callbackimage)
        #self.image_sub2 = rospy.Subscriber("/zed/left/image_rect_color",Image,self.callbackimage2)
        self.pointcloud_sub = rospy.Subscriber("/zed/points2",PointCloud2,self.pointcloudcall)

        
        self.bb_sub = rospy.Subscriber("/yolov5/detections",BoundingBoxes,self.bbs)
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.pub = rospy.Publisher('pointcloudfiltered_topic', PointCloud2, queue_size=10)
        self.bridge = CvBridge()
        self.image=[]
        self.image2=[]
        #used to sync images with bounding boxes and point cloud
        self.keys=[0,0,0]


        self.bboxes=[]
        self.my_pcl=PointCloud2()
    def callbackimage(self,data):
                if self.keys[0] != -1:
                     self.keys[0] = 0
                #Converting Image from ros msg format 
                     self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                     self.keys[0] = 1
                     #self.filter()
    #def callbackimage2(self,data):
        #if self.keys[2]!=-1:
         
          #self.keys[2] = 0
          #self.image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
          #self.keys[2] = 1
          #self.filter()
                
    def bbs(self,data):
            if self.keys[1] != -1:
                self.keys[1] = 0
                self.bboxes=data.bounding_boxes
                self.keys[1] = 1
                self.filter()     


    def pointcloudcall(self,data):
         if self.keys[2]!=-1:
              self.keys[2] = 0
              self.my_pcl=data
              self.keys[2] = 1
              self.filter()
    
    





    def filter(self):
            
        if self.keys[0] == 1 and self.keys[1] == 1 and self.keys[2]==1:
            self.keys = [-1, -1,-1]
        
            #intializing point cloud
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.my_pcl.header.frame_id  # Replace with the appropriate frame ID
    # Create the fields for x, y, z, r, g, b
            fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('r', 12, PointField.FLOAT32, 1),
                    PointField('g', 16, PointField.FLOAT32, 1),
                    PointField('b', 20, PointField.FLOAT32, 1),
                    ]
            points=[]
            for boundingbox in self.bboxes:


                minx=boundingbox.ymin
                maxx=boundingbox.ymax
                miny=boundingbox.xmin
                maxy=boundingbox.xmax

                if boundingbox.Class=="blue_cone":
                    red=0
                    green=0
                    blue=200
                elif boundingbox.Class=="yellow_cone":
                    red=200
                    green=200
                    blue=0
                elif boundingbox.Class=="orange_cone":
                    red=200
                    green=100
                    blue=0
                elif boundingbox.Class=="large_orange_cone":
                    red=200
                    green=0
                    blue=0      
                for i in range(minx,maxx):
                      for m in range (miny,maxy):
                            (b,g,r)=self.image[i,m]
                            (h,s,v)=colorsys.rgb_to_hsv(r/255,g/255,b/255)
                            if s<0.3 or v<0.3:
                                 self.image[i,m]=(0,255,0)
                                 #self.image2[i,m]=(0,255,0)
                            else:
                                             
                                # getting each pixels coordinates in ZED_FRAME
                                array_position = i * self.my_pcl.row_step + m * self.my_pcl.point_step
                                array_pos_x = array_position + self.my_pcl.fields[0].offset  # X has an offset of 0
                                array_pos_y = array_position + self.my_pcl.fields[1].offset  # Y has an offset of 4
                                array_pos_z = array_position + self.my_pcl.fields[2].offset  # Z has an offset of 8

                                x = struct.unpack('f', self.my_pcl.data[array_pos_x:array_pos_x + 4])[0]
                                y = struct.unpack('f', self.my_pcl.data[array_pos_y:array_pos_y + 4])[0]
                
                                z = struct.unpack('f', self.my_pcl.data[array_pos_z:array_pos_z + 4])[0]
                                
                                if math.isnan(x): 
                                     m=1
                                else:
                                   points.append([x,y,z,red,green,blue])
                                       
                                
            cloud = pc2.create_cloud(header, fields, points)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
            #self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.image2, "bgr8"))
            self.pub.publish(cloud)
            self.keys=[0,0,0]
            

if __name__ == '__main__':
    try:
        hsvfilter = hsvfilter()
        del hsvfilter
    except rospy.ROSInterruptException:
        pass
    rospy.spin()