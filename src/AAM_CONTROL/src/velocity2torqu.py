#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import math
from nav_msgs.msg import Odometry

class MotorController:
    def __init__(self, torque_to_rpm_ratio):
        rospy.init_node("torque_control", anonymous=True)

        self.torque_to_rpm_ratio = torque_to_rpm_ratio
        self.kp = 60  # Proportional gain for the controller, can be tuned
        self.ki = 120  # Integral gain for the controller, can be tuned
        self.kd = 7.5  # Derivative gain for the controller, can be tuned
        self.previous_error = 0
        self.integral = 0
        self.Act_Vel = 0
        self.des_vel = 0
        self.wheel_rpm = 0
        self.required_torque = 0
        self.output = 0
        self.dt = 0.1

        # Subscribers
        rospy.Subscriber("/ground_truth/state_raw", Odometry, self.odom_callback)
        rospy.Subscriber("/robot_control/command", AckermannDriveStamped, self.control_callback)

    def odom_callback(self, od):
        vx = od.twist.twist.linear.x
        vy = od.twist.twist.linear.y
        self.Act_Vel = math.sqrt(vx ** 2 + vy ** 2)
        #print("Actual velocity = ", self.Act_Vel)

    def control_callback(self, con):
        self.des_vel = con.drive.speed
        #print("Desired velocity = ", self.des_vel)
        self.pid_control()
        self.velocity_to_rpm(0.513)

    def velocity_to_rpm(self, wheel_radius):
        # Convert car velocity (m/s) to wheel RPM
        wheel_circumference = 2 * math.pi * wheel_radius
        self.wheel_rpm = (self.des_vel / wheel_circumference) * 60
        self.rpm_to_torque()

    def rpm_to_torque(self):
        # Convert wheel RPM to required torque using the motor's torque to RPM ratio
        self.required_torque = self.wheel_rpm / self.torque_to_rpm_ratio
        self.get_required_torque()

    def pid_control(self):
        # PID controller to calculate the required torque
        error = self.des_vel - self.Act_Vel
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error

        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative

    def get_required_torque(self):
        if int(self.required_torque) >= 195:
            self.required_torque = 195
        return self.required_torque

class BrakingSystem(MotorController):
    def __init__(self, torque_to_rpm_ratio):
        super().__init__(torque_to_rpm_ratio)
        self.front_brake_pressure = 0
        self.rear_brake_pressure = 0
        self.desired_deceleration = 0

    def calculate_brake_force(self):
        # Constants
        total_vehicle_mass = 300  # kg
        front_ratio = 0.7
        rear_ratio = 0.3

        # Basic physics calculations
        total_braking_force = total_vehicle_mass * self.desired_deceleration
        front_braking_force = total_braking_force * front_ratio
        rear_braking_force = total_braking_force * rear_ratio

        # Convert forces to pressures or actuator commands as needed
        self.front_brake_pressure = self.force_to_pressure(front_braking_force)
        self.rear_brake_pressure = self.force_to_pressure(rear_braking_force)

        return self.front_brake_pressure, self.rear_brake_pressure

    def force_to_pressure(self, force):
        # Placeholder function to convert force to brake pressure
        pressure = force / 10  # This is a simplification; real conversion is more complex
        return pressure

    def apply_brakes(self):
        # Calculate desired deceleration
        self.desired_deceleration = (self.Act_Vel - self.des_vel) / self.dt
        front_pressure, rear_pressure = self.calculate_brake_force()
        return front_pressure, rear_pressure

    def check(self, event):
        if self.des_vel < self.Act_Vel:
            # Apply brakes if the car needs to decelerate
            front_pressure, rear_pressure = self.apply_brakes()
            actual_velocity = simulate_car_response(self.Act_Vel, 0, CAR_MASS, WHEEL_RADIUS, TIME_STEP, braking=True, front_pressure=front_pressure, rear_pressure=rear_pressure)  # Here want the real actual velocity
            print(f"Actual Velocity: {actual_velocity:.2f} m/s, Front Brake Pressure: {front_pressure:.2f} units, Rear Brake Pressure: {rear_pressure:.2f} units")
        else:
            # Apply motor torque to accelerate
            required_torque = self.get_required_torque()
            actual_velocity = simulate_car_response(self.Act_Vel, required_torque, CAR_MASS, WHEEL_RADIUS, TIME_STEP)  # Here want the real actual velocity
            print(f"Actual Velocity: {actual_velocity:.2f} m/s, Required Torque: {required_torque:.2f} Nm")

# State estimation
def simulate_car_response(current_velocity, torque, mass, wheel_radius, dt, braking=False, front_pressure=0, rear_pressure=0):
    if braking:
        # Simplified simulation of car response to braking
        deceleration = (front_pressure + rear_pressure) / mass
        new_velocity = current_velocity - deceleration * dt
        new_velocity = max(new_velocity, 0)  # Ensure velocity doesn't go negative
    else:
        # Simplified simulation of car response to applied torque
        wheel_force = torque / wheel_radius  # Force applied by the wheel
        acceleration = wheel_force / mass  # Newton's second law: F = ma
        new_velocity = current_velocity + acceleration * dt
    return new_velocity

# Constants
TORQUE_TO_RPM_RATIO = 3103 / 27.2
WHEEL_RADIUS = 0.513  # Example wheel radius in meters
TIME_STEP = 0.1  # Time step for control loop in seconds
CAR_MASS = 300  # Mass of the car in kg

if __name__ == '__main__':
    try:
        controller = MotorController(TORQUE_TO_RPM_RATIO)
        braking_system = BrakingSystem(TORQUE_TO_RPM_RATIO)
        rospy.Timer(rospy.Duration(0.07), braking_system.check)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
