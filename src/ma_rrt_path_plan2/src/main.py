#!/usr/bin/env python3

import rospy
from MaRRTPathPlanNode import MaRRTPathPlanNode

def main():
    rospy.init_node('MaRRTPathPlanNode')
    maRRTPathPlanNode = MaRRTPathPlanNode()

    rate = rospy.Rate(1) # big amount on purpose
    #iteration = 0

    while not rospy.is_shutdown():
        #rospy.loginfo("Main loop iteration: %d", iteration)
        maRRTPathPlanNode.sampleTree()
        #iteration += 1
        rate.sleep()

    # Spin until ctrl + c
    # rospy.spin()

if __name__ == '__main__':
    main()
