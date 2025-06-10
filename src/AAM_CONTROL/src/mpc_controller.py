#!/usr/bin/env python3
import rospy, subprocess
if __name__ == "__main__":
    rospy.init_node("mpc_wrapper", anonymous=True)
    subprocess.Popen(["rosrun", "mpc_control", "mpc_node"])
    rospy.spin()
