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
import tf2_ros
from tf2_geometry_msgs import PointStamped
import geometry_msgs.msg

class hsvfilter():
    def __init__(self):
        rospy.init_node("hsv_node", anonymous = True,disable_signals=True)
        self.image_sub = rospy.Subscriber("/yolov5/image_out",Image,self.callbackimage)
        #self.image_sub2 = rospy.Subscriber("/zed/left/image_rect_color",Image,self.callbackimage2)
        self.pointcloud_sub = rospy.Subscriber("/zed/points2",PointCloud2,self.pointcloudcall)

        
        self.bb_sub = rospy.Subscriber("/yolov5/detections",BoundingBoxes,self.bbs)
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.pub = rospy.Publisher('pointcloudfiltered_topic', PointCloud2, queue_size=10)
        self.pub2 = rospy.Publisher('pointcloudfiltered_topic2', PointCloud2, queue_size=10)
        self.bridge = CvBridge()
        self.image=[]
        self.image2=[]

        #used to sync images with bounding boxes and point cloud
        self.keys=[0,0,0]


        self.bboxes=[]
        self.my_pcl=PointCloud2()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Wait for the transform between the frames to become available
        self.tf_buffer.can_transform("base_link", "zed_frame", rospy.Time(), rospy.Duration(1))
    def transform_point(self,cone):
# Define the point in the zed_frame
        point_in_zed_frame = geometry_msgs.msg.PointStamped()
        point_in_zed_frame.header.frame_id = "zed_frame"
        point_in_zed_frame.point.x = cone[0]  # Replace with your point's x-coordinate
        point_in_zed_frame.point.y = cone[1]  # Replace with your point's y-coordinate
        point_in_zed_frame.point.z = cone[2]  # Replace with your point's z-coordinate

      # Transform the point to the base_link frame
        try:
          transformed_point = (self.tf_buffer).transform(point_in_zed_frame, "base_link")
          return transformed_point
        except tf2_ros.TransformException as e:
            rospy.logwarn("Failed to transform point: %s", str(e))    
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
                #self.filter()     


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
            header.frame_id = "base_link"  # Replace with the appropriate frame ID
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
            pointstest=[]
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
                    red=0
                    green=200
                    blue=0
                elif boundingbox.Class=="large_orange_cone":
                    red=200
                    green=0
                    blue=0
                xavg=0     
                yavg=0         
                zavg=0
                countz=0
                
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
                                   #transformed_point=self.transform_point([x,y,z])
                                   #xavg=xavg+transformed_point.point.x
                                   #yavg=yavg+transformed_point.point.y
                                   #zavg=zavg+transformed_point.point.z
                                   xavg=xavg+x
                                   yavg=yavg+y
                                   zavg=zavg+z
                                   countz+=1
                                   transformed_point1=self.transform_point([x,y,z])
                                   pointstest.append([transformed_point1.point.x,transformed_point1.point.y,transformed_point1.point.z,red,green,blue])
                if(countz!=0):

                    transformed_point=self.transform_point([xavg/countz,yavg/countz,zavg/countz])
                    points.append([transformed_point.point.x,transformed_point.point.y,transformed_point.point.z,red,green,blue])
            xmin=0
            xmax=20
            ymin=-9
            ymax=9
            filtered_points=[p for p in points if xmin<=p[0]<=xmax and ymin<=p[1]<=ymax]
            cloud = pc2.create_cloud(header, fields, filtered_points)
            cloud2= pc2.create_cloud(header, fields, pointstest)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
            #self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.image2, "bgr8"))
            #print(len(self.bboxes))
            #print(len(points))
            self.pub.publish(cloud)
            self.pub2.publish(cloud2)
            self.keys=[0,0,0]
            

if __name__ == '__main__':
    
    try:
        hsvfilter = hsvfilter()
        del hsvfilter
    except rospy.ROSInterruptException:
        pass
    rospy.spin()