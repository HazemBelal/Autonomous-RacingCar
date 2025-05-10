#!/usr/bin/env python3

import rospy
import subprocess
import time

def launch_in_new_tab(package, launch_file):
    command = f"roslaunch {package} {launch_file}"
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'bash', '-c', command])
def run_in_new_tab(package, launch_file):
    command = f"rosrun {package} {launch_file}"
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'bash', '-c', command])

def main():
    rospy.init_node('sequential_launcher', anonymous=True)
    
    # Step 1: Launch yolov5 in a new tab
    launch_in_new_tab("yolov5_ros", "yolov5.launch")
    rospy.sleep(5)  # Adjust sleep time if needed
    rospy.set_param('yolov5_ready', True)
    # Step 2: Launch cameranewapproach after yolov5 in a new tab
    while not rospy.get_param('yolov5_ready'):
        time.sleep(5)
    launch_in_new_tab("camera_cone_detection", "cameranewapproach.launch")
    rospy.set_param('cameranewapproach_ready', True)

    # Step 3: Launch pathplanning after cameranewapproach in a new tab
    while not rospy.get_param('cameranewapproach_ready'):
        time.sleep(5)
    run_in_new_tab("AAM_PATH_PLANNING", "newMidMarch.py")
    rospy.set_param('pathplanning_ready', True)

    # Step 4: Launch stateestimation after pathplanning in a new tab

    # Step 5: Launch localization after stateestimation in a new tab
    #while not rospy.get_param('stateestimation_ready'):
        #time.sleep(1)
    #launch_in_new_tab("AAM_LOCALIZTION", "localization.launch")
    #rospy.set_param('localization_ready', True)

    # Step 6: Launch control after localization in a new tab
    run_in_new_tab("AAM_CONTROL", "newpaper.py")

if __name__ == '__main__':
    main()