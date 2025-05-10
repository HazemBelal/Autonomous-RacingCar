#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math
from sensor_msgs import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np 
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped

def transform_point(cone):
# Define the point in the zed_frame
        point_in_zed_frame = geometry_msgs.msg.PointStamped()
        point_in_zed_frame.header.frame_id = "rplidar"
        point_in_zed_frame.point.x = cone[0]  # Replace with your point's x-coordinate
        point_in_zed_frame.point.y = cone[1]  # Replace with your point's y-coordinate
        point_in_zed_frame.point.z = cone[2]  # Replace with your point's z-coordinate

      # Transform the point to the base_link frame
        try:
          transformed_point = (tf_buffer).transform(point_in_zed_frame, "base_link")
          return transformed_point
        except tf2_ros.TransformException as e:
            rospy.logwarn("Failed to transform point: %s", str(e))    
def callback(msg):
    ranges = np.array(msg.ranges)
    angle_min = np.array(msg.angle_min)
    angle_increment = np.array (msg.angle_increment)
        
     # convert the message of type LaserScan to a PointCloud2
    points=[]
    for i in range(len(ranges)):
        angle = angle_min + (i * angle_increment)
        range_val = ranges[i]
        x = range_val * math.cos(angle)
        y = range_val * math.sin(angle)
        points.append((x, y))
  
    xmin=0
    xmax=10
    ymin=-6
    ymax=6
    np_points=np.array(points)
    filtered_points=[p for p in np_points if xmin<=p[0]<=xmax and ymin<=p[1]<=ymax]
    i=0
    while i<len(filtered_points):
        n=i+1
        while n<len(filtered_points):
            #  print(n,len(filtered_points))
             if (((filtered_points[i][0]-filtered_points[n][0])**2+(filtered_points[i][1]-filtered_points[n][1])**2)**0.5)<0.2:
                  filtered_points.pop(n)
                  n=n-1
                  
             n=n+1
        i=i+1             
    # Create a marker array
    marker_array = MarkerArray()

    # Create a marker
    
    id1=0
    for point in filtered_points:
        transformed_point=transform_point([point[0],point[1],0])
        # Set the marker position to the point position
        marker = Marker()
        marker.header.frame_id = "base_link"
        print(msg.header.frame_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.id=id1
        marker.pose.position.x = (transformed_point.point.x)
        marker.pose.position.y = (transformed_point.point.y)
        marker.pose.position.z = 0
        marker.lifetime=rospy.Time(0.03)
        id1+=1
        print(point[1])
        # Add the marker to the marker array
        marker_array.markers.append(marker)
    # print(len(marker_array.markers)) 
    pc_pub.publish(marker_array)
    filtered_points.clear() 
def main1():
    rospy.init_node('lidar',anonymous =False)
    global pc_pub
    pc_pub= rospy.Publisher("converted_pc", MarkerArray, queue_size=1)
    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)
        # Wait for the transform between the frames to become available
    tf_buffer.can_transform("base_link", "rplidar", rospy.Time(), rospy.Duration(1))

    rospy.Subscriber('/rplidar_points',LaserScan,callback,queue_size=1)
    rospy.spin()
if __name__ == '__main__':
    print('2d lidar is running...')
    main1()   
