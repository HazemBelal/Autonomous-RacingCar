#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2

def point_cloud_callback(point_cloud):
    # Create a MarkerArray message
    marker_array = MarkerArray()
    points=[]    
    # Iterate through all the points in the point cloud
    for idx, point in enumerate(pc2.read_points(point_cloud, field_names=("x", "y", "z","r","g","b"), skip_nans=True)):
        # Create a Marker message for each point
        marker = Marker()
        marker.header.frame_id = point_cloud.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "point_cloud_markers"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = point[3]
        marker.color.g = point[4]
        marker.color.b = point[5]
        points.append([point[0],point[1],point[2]])
        # Add the marker to the MarkerArray
        marker_array.markers.append(marker)

    # Publish the MarkerArray message
    marker_pub.publish(marker_array)
    print(points)
    print("")


if __name__ == '__main__':
    rospy.init_node('pointcloud_marker_publisher')
    
    # Create a publisher for the MarkerArray topic
    marker_pub = rospy.Publisher('point_cloud_markers', MarkerArray, queue_size=10)
    
    # Create a subscriber for the point cloud topic
    rospy.Subscriber('pointcloudfiltered_topic', PointCloud2, point_cloud_callback)
    
    rospy.spin()