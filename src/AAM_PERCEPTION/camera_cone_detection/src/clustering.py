#!/usr/bin/env python3
from functools import total_ordering
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
centroidpublisher = rospy.Publisher('/camera_cones_marker', MarkerArray,queue_size=0)
global tf_buffer
def transform_point(cone):
    global tf_buffer
# Define the point in the zed_frame
    point_in_zed_frame = geometry_msgs.msg.PointStamped()
    point_in_zed_frame.header.frame_id = "zed_frame"
    point_in_zed_frame.point.x = cone[0]  # Replace with your point's x-coordinate
    point_in_zed_frame.point.y = cone[1]  # Replace with your point's y-coordinate
    point_in_zed_frame.point.z = cone[2]  # Replace with your point's z-coordinate

# Transform the point to the base_link frame
    try:
        transformed_point = tf_buffer.transform(point_in_zed_frame, "base_link")
        return transformed_point
    except tf2_ros.TransformException as e:
        rospy.logwarn("Failed to transform point: %s", str(e))
    
def centroid_visuals(cones):
        c = 0

        rviz_msg = Marker()
        rviz_msg_array = MarkerArray()

        while c < len(cones):
           
        
            
           

            
            count = 0
            MARKERS_MAX = 100

            rviz_msg = Marker()
            rviz_msg.header.frame_id = "base_link"
            rviz_msg.ADD
            rviz_msg.SPHERE
            rviz_msg.pose.position.x = cones[c][0]
            rviz_msg.pose.position.y = cones[c][1]
            rviz_msg.pose.position.z = cones[c][2]
            rviz_msg.lifetime = rospy.Duration(0.1)
            rviz_msg.pose.orientation.w = 1
            rviz_msg.scale.x = 0.2
            rviz_msg.scale.y = 0.2
            rviz_msg.scale.z = 0.2
            rviz_msg.color.a = 1
            rviz_msg.color.g = 1
            rviz_msg.color.r = cones[c][3]
            rviz_msg.color.g = cones[c][4]
            rviz_msg.color.b = cones[c][5]
            
                



            rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
            rviz_msg.type = rviz_msg.SPHERE
	    #self.rviz_msg.type = Marker.MESH_RESOURCE
            rviz_msg.mesh_use_embedded_materials = True

            if(count > MARKERS_MAX):
                rviz_msg_array.markers.pop(0)
            
            rviz_msg_array.markers.append(rviz_msg)
            c +=1
        m = 0
        id = 0
        for m in rviz_msg_array.markers:
            m.id = id
            id += 1
        
        centroidpublisher.publish(rviz_msg_array)
def euclidean_clustering_callback(msg):
    # Convert the PointCloud2 message to a numpy array
    point_array = pc2.read_points(msg, field_names=("x", "y", "z", "r","g","b"))
    point_array = np.asarray(list(point_array))

    # Extract the spatial coordinates and RGB values
    spatial_coords = point_array[:, :3]
    rgb_values = point_array[:, 3:6]

    # Create a PCL PointCloud object
    cloud = pcl.PointCloud()
    cloud.from_array(spatial_coords.astype(np.float32))

    # Create a k-d tree representation of the points
    kdtree = cloud.make_kdtree()

    # Create a Euclidean Cluster Extraction object
    ec = cloud.make_EuclideanClusterExtraction()

    # Set the distance threshold for cluster extraction
    ec.set_ClusterTolerance(0.200)  # Adjust this value as needed

    # Set the minimum and maximum cluster size
    ec.set_MinClusterSize(40)  # Adjust these values as needed
    ec.set_MaxClusterSize(25000)

    # Extract the clusters
    cluster_indices = ec.Extract()
    cones=[]
    totalx=0
    totaly=0
    totalz=0
    totalr=0
    totalg=0
    totalb=0
    # Iterate through the cluster indices and extract the points
    for i in range(len(cluster_indices)):
        u=len(cluster_indices[i])
        totalx=0
        totaly=0
        totalz=0
        totalr=0
        totalg=0
        totalb=0
        for m in range(len(cluster_indices[i])):
            totalx=totalx+spatial_coords[cluster_indices[i][m]][0]
            totaly=totaly+spatial_coords[cluster_indices[i][m]][1]
            totalz=totalz+spatial_coords[cluster_indices[i][m]][2]
            totalr=totalr+rgb_values[cluster_indices[i][m]][0]
            totalg=totalg+rgb_values[cluster_indices[i][m]][1]
            totalb =totalb+rgb_values[cluster_indices[i][m]][2]     
        transformed_point=transform_point([totalx/u,totaly/u,totalz/u])
        print(u)    
        print([transformed_point.point.x,transformed_point.point.y,transformed_point.point.z,totalr/u,totalg/u,totalb/u])
        cones.append([transformed_point.point.x,transformed_point.point.y,transformed_point.point.z,totalr/u,totalg/u,totalb/u])

    print("....................")    
    centroid_visuals(cones)
    


if __name__ == '__main__':
    rospy.init_node('euclidean_clustering_node')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

# Wait for the transform between the frames to become available
    tf_buffer.can_transform("base_link", "zed_frame", rospy.Time(), rospy.Duration(1))

    
    rospy.Subscriber('/pointcloudfiltered_topic', PointCloud2, euclidean_clustering_callback)
    rospy.spin()