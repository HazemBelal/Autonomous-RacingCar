#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from ma_rrt_path_plan.msg import Map  # Your custom Map message

def marker_array_callback(marker_array_msg):
    rospy.loginfo("Received MarkerArray with %d markers", len(marker_array_msg.markers))
    
    # Create a new Map message and set its header to the local frame "base_link"
    my_map = Map()
    my_map.header.stamp = rospy.Time.now()
    my_map.header.frame_id = "base_link"  # local frame (Option A)

    my_map.cones = []
    # For each marker, directly copy its position without transformation.
    for marker in marker_array_msg.markers:
        cone_position = Point()
        cone_position.x = marker.pose.position.x
        cone_position.y = marker.pose.position.y
        cone_position.z = marker.pose.position.z  # Typically zero for ground vehicles
        my_map.cones.append(cone_position)
    
    rospy.loginfo("Publishing Map message with %d cones", len(my_map.cones))
    map_pub.publish(my_map)

if __name__ == '__main__':
    rospy.init_node('cone_map_adapter', anonymous=True)
    
    # Create the publisher for the Map message.
    map_pub = rospy.Publisher('/map', Map, queue_size=1)
    
    # Subscribe to the cone marker topic. Make sure that the markers have header.frame_id set to "base_link".
    rospy.Subscriber('/camera_cones_marker', MarkerArray, marker_array_callback)
    
    rospy.loginfo("cone_map_adapter node started: Using local coordinates (base_link) for cones")
    rospy.spin()
