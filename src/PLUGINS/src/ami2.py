#!/usr/bin/env python3
from PyQt5 import QtCore, QtGui, QtWidgets
import subprocess
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import time
import os
import signal
import psutil
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        self.currentstate="AS_OFF"
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setGeometry(QtCore.QRect(560, 50, 231, 25))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")

        self.comboBox.currentIndexChanged.connect(self.on_combo_box_changed)
        self.changestate = QtWidgets.QPushButton(self.centralwidget)
        self.changestate.setGeometry(QtCore.QRect(560, 160, 89, 25))
        self.changestate.setObjectName("changestate")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(0, 50, 101, 17))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(110, 50, 81, 17))
        self.label_2.setObjectName("label_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        self.changestate.clicked.connect(self.on_change_state_clicked)
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(680, 160, 89, 25))
        self.pushButton.setObjectName("RequestEBS")
        self.pushButton.clicked.connect(self.on_EBS)
        MainWindow.setStatusBar(self.statusbar)
        self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)
        self.retranslateUi(MainWindow)
        self.launched_processes = []
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
    def on_change_state_clicked(self):
        print(1)
        global currentstate
        current_selection = self.comboBox.currentText()
        if self.currentstate == "AS_READY" and current_selection == "AMI_ACCELERATION":

            currentstate = "AS_DRIVING"
            package_name = "LAUNCH"  # Replace with your package name
            launch_file_name = "acceleration_drive.launch"  # Replace with your launch file name
            try:
                # Print the command to check if it's correct
                print(f"roslaunch {package_name} {launch_file_name}")
                # Launch the file using roslaunch
                process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", package_name, launch_file_name])
                self.launched_processes.append(process)
                self.currentstate="AS_DRIVING"
                self.label_2.setText("AS_DRIVING")
            except Exception as e:
                self.label_2.setText("Failed to launch: " + str(e))
        if self.currentstate=="AS_READY" and current_selection== "AMI_TRACK_DRIVE":
            currentstate = "AS_DRIVING"
            package_name = "LAUNCH"  # Replace with your package name
            launch_file_name = "track_drive.launch"  # Replace with your launch file name
            try:
                # Print the command to check if it's correct
                print(f"roslaunch {package_name} {launch_file_name}")
                # Launch the file using roslaunch
                process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", package_name, launch_file_name])
                self.launched_processes.append(process)
                self.label_2.setText("AS_DRIVING")
                self.currentstate="AS_DRIVING"
            except Exception as e:
                self.label_2.setText("Failed to launch: " + str(e))
    def on_combo_box_changed(self):
        current_selection = self.comboBox.currentText()
        if(current_selection!="AMI_NOT_SELECTED" and current_selection!="AMI_MANUAL" and self.currentstate!="AS_DRIVING"):
            self.label_2.setText("AS_READY")
            self.currentstate="AS_READY"


                       

    def get_ros_nodes(self):
        """Get a list of active ROS nodes."""
        result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE)
        nodes = result.stdout.decode('utf-8').split('\n')
        return [node for node in nodes if node]

    def kill_ros_node(self,node_name):
        """Kill a specific ROS node by name."""
        try:
            subprocess.run(['rosnode', 'kill', node_name], check=True)
            print(f"Killed ROS node: {node_name}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to kill ROS node {node_name}: {e}")

    def kill_specific_nodes(self,node_names):
        """Kill a list of specific ROS nodes."""
        active_nodes = self.get_ros_nodes()
        for node_name in node_names:
            if node_name in active_nodes:
                self.kill_ros_node(node_name)
            else:
                print(f"ROS node {node_name} is not active.")
                continue    
    def on_EBS(self):
        print("A7a")
        global currentstate
        current_selection = self.comboBox.currentText()
        if (self.currentstate=="AS_DRIVING" or self.currentstate=="AS_READY" ):
            print("a7a")
            self.label_2.setText("AS_EMERGENCY")
            QtWidgets.QApplication.processEvents() 
            self.currentstate = "AS_EMERGENCY"
            print(self.currentstate)
            nodes_to_kill = ['/control', '/pathplanning','/localization','/detect','/hsv_node','/euclidean_clustering_node','/rviz','/rviz2']
            self.kill_specific_nodes(nodes_to_kill)
            SA = AckermannDriveStamped()
            SA.drive.steering_angle = 0
            SA.drive.speed = 0
            SA.drive.steering_angle_velocity = 0
            SA.drive.acceleration = 0
            SA.drive.jerk = 0
            self.robot_control_pub.publish(SA) 
            time.sleep(4)
            self.label_2.setText("AS_OFF")
            self.currentstate = "AS_OFF"
        
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.comboBox.setItemText(0, _translate("MainWindow", "AMI_NOT_SELECTED"))
        self.comboBox.setItemText(1, _translate("MainWindow", "AMI_ACCELERATION"))
        self.comboBox.setItemText(2, _translate("MainWindow", "AMI_SKIDPAD"))
        self.comboBox.setItemText(3, _translate("MainWindow", "AMI_AUTOCROSS"))
        self.comboBox.setItemText(4, _translate("MainWindow", "AMI_TRACK_DRIVE"))
        self.comboBox.setItemText(5, _translate("MainWindow", "AMI_ADS_EBS"))
        self.comboBox.setItemText(6, _translate("MainWindow", "AMI_MANUAL"))
        self.comboBox.setItemText(7, _translate("MainWindow", "AMI_JOYSTICK"))
        self.comboBox.setItemText(8, _translate("MainWindow", "AMI_DDT_INSPECTION_A"))
        self.comboBox.setItemText(9, _translate("MainWindow", "AMI_DDT_INSPECTION_B"))
        self.changestate.setText(_translate("MainWindow", "GO_SIGNAL"))
        self.label.setText(_translate("MainWindow", "Current State"))
        self.label_2.setText(_translate("MainWindow", self.currentstate))
        self.pushButton.setText(_translate("MainWindow", "RequestEBS"))

if __name__ == "__main__":
    import sys
    rospy.init_node('qt_ros_node', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    