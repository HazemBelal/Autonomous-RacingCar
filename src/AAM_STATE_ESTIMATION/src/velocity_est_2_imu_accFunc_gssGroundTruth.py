#!/usr/bin/env python3
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf_conversions

class sensor(object):
    def __init__(self):
        rospy.Subscriber("/sensor_imu_hector", Imu, self.imu_callback)
        rospy.Subscriber("/ground_truth/state_raw", Odometry, self.odom_callback)

        # 2×1 state: [vx; vy]
        self.xEst = np.zeros((2, 1))
        # 2×2 covariance
        self.pEst = np.eye(2) * 0.1
        # yaw angle
        self.yaw = 0.0

        self.ax_body = 0.0
        self.ay_body = 0.0
        self.angular_vel = 0.0
        self.z = np.zeros((2, 1))

        # track time for dt
        self.last_time = rospy.Time.now()
        # run EKF predict+update at 50 Hz
        self.ekf_timer = rospy.Timer(
            rospy.Duration(1.0/50.0),
            self.ekf_timer_callback
        )

        self.pub = rospy.Publisher('/ekf_odom', Odometry, queue_size=1)

    def imu_callback(self, msg):
        self.ax_body = msg.linear_acceleration.x
        self.ay_body = msg.linear_acceleration.y
        self.angular_vel = msg.angular_velocity.z

    def odom_callback(self, odom_msg):
        self.z = np.array([
            [odom_msg.twist.twist.linear.x],
            [odom_msg.twist.twist.linear.y]
        ])

    def ekf_timer_callback(self, event):
        now = event.current_real
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        self.ekf_predict(dt)
        self.ekf_update()
        self.publish_estimate()

    def ekf_predict(self, dt):
        yaw = self.yaw
        ax_w = self.ax_body * math.cos(yaw) - self.ay_body * math.sin(yaw)
        ay_w = self.ax_body * math.sin(yaw) + self.ay_body * math.cos(yaw)

        self.xEst[0, 0] += ax_w * dt
        self.xEst[1, 0] += ay_w * dt

        self.yaw += self.angular_vel * dt
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi

        Q = np.diag([0.1, 0.1]) * dt
        self.pEst += Q

    def ekf_update(self):
        y = self.z - self.xEst

        R = np.diag([0.05, 0.05])
        S = self.pEst + R

        K = self.pEst @ np.linalg.inv(S)

        self.xEst = self.xEst + K @ y
        self.pEst = (np.eye(2) - K) @ self.pEst

    def publish_estimate(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        # world‐frame EKF estimate
        vx_w = self.xEst[0, 0]
        vy_w = self.xEst[1, 0]
        # convert back to body‐frame using current yaw
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        v_body_x =  c * vx_w + s * vy_w
        v_body_y = -s * vx_w + c * vy_w

        odom.twist.twist.linear.x = v_body_x
        odom.twist.twist.linear.y = v_body_y
        odom.twist.twist.angular.z = self.angular_vel
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        self.pub.publish(odom)


if __name__ == '__main__':
    try:
        rospy.init_node('ekf_node')
        ekf = sensor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

