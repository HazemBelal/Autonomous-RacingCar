#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from gekko import GEKKO
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState


class PIDControllerNode:
    def __init__(self):
        rospy.init_node("pid_controller_node")
        rospy.loginfo("PID Controller Node has been started")

        self.actual_angle_msg = Float64()

        # Initialize PID parameters
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.01

        self.prev_error = 0
        self.integral = 0
        self.dt = 0.1  # Adjust loop rate accordingly

        # Initialize control variables
        self.reference_angle = 0.0
        self.actual_angle = 0.0
        self.control_output = 0.0

        self.actual_angle_pub = rospy.Publisher("control_output_topic", Float64, queue_size=10)
        self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)


        # Initialize GEKKO model
        self.model = GEKKO(remote=False)

        # GEKKO PID parameters for optimization
        self.Kp_gk = self.model.FV(value=self.Kp, lb=0.1, ub=10.0)
        self.Ki_gk = self.model.FV(value=self.Ki, lb=0.01, ub=1.0)
        self.Kd_gk = self.model.FV(value=self.Kd, lb=0.001, ub=0.1)

        self.error = self.model.CV(value=0.0)
        self.model.Equation(self.error == self.reference_angle - self.actual_angle)

        # Objective function: minimize sum of squared errors
        self.model.Obj(self.model.sum([self.error**2]))
        # Time discretization for optimization (adjust time horizon)
        self.model.time = np.linspace(0, 1, 21)

        # Control mode (consider IMODE 1 or 3)
        self.model.options.IMODE = 3

        # Saturation limits for control output
        self.limit = 27.2  # Maximum steering angle
        self.anti_windup_limit = self.limit * 0.1  # Anti-windup buffer

        # Subscribe to reference angles
        self.ref_angle_sub = rospy.Subscriber("reference_angle_topic", Float64, self.ref_angle_callback)

        # Subscribe to feedback sensor
        self.feedback_sub =rospy.Subscriber("/aamfsd/joint_states", JointState, self.feedback_callback)   # for steering and velocity

        #self.feedback_sub = rospy.Subscriber("feedback_angle_topic", Float64, self.feedback_callback)

        self.rate = rospy.Rate(10)  # Adjust loop rate based on dt

    def ref_angle_callback(self, msg):
        self.reference_angle = msg.data
        print("steering angle : ",self.reference_angle)

    def feedback_callback(self, msg):
        self.actual_angle = (msg.position[4]+msg.position[9])/2
        print("actual angle : ",self.actual_angle)
        self.update_pid()

    def update_pid(self):
        self.Kp = self.Kp_gk.VALUE
        self.Ki = self.Ki_gk.VALUE
        self.Kd = self.Kd_gk.VALUE

        error = self.reference_angle - self.actual_angle
        self.integral = np.clip(self.integral + error * self.dt, -self.limit + self.anti_windup_limit, self.limit - self.anti_windup_limit)

        # PID Output with saturation
        self.control_output = self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.prev_error) / self.dt
        self.control_output = np.clip(self.control_output, -self.limit, self.limit)

        # Update previous error
        self.prev_error = error

        # Publish control output
        control_msg = Float64()
        control_msg.data = self.control_output
        self.actual_angle_pub.publish(control_msg)
        # publish real outputs
        SA = AckermannDriveStamped()
        SA.drive.steering_angle = self.control_output
        SA.drive.speed = 3
        self.robot_control_pub.publish(SA)

        # Log the output
        rospy.loginfo(f"Reference Angle: {self.reference_angle}, Actual Angle: {self.actual_angle}, Control Output: {SA}")

if __name__ == '__main__':
    try:
        controller_node = PIDControllerNode()
        #reference_angle = 17.100978505612005
        #controller_node.set_reference_angle(reference_angle)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass