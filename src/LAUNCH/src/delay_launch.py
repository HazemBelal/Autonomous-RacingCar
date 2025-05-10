#!/usr/bin/env python3

import time
import subprocess
import rospy

def delay_launch(delay, launch_file):
    rospy.init_node('delay_launch', anonymous=True)
    rospy.loginfo(f"Sleeping for {delay} seconds before launching {launch_file}")
    time.sleep(delay)
    rospy.loginfo(f"Launching {launch_file}")
    subprocess.Popen(["roslaunch", launch_file])

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print("Usage: delay_launch.py delay_in_seconds package_name/launch_file")
        sys.exit(1)
    delay = int(sys.argv[1])
    launch_file = sys.argv[2]
    try:
        delay_launch(delay, launch_file)
    except rospy.ROSInterruptException:
        pass

