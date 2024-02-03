#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from planner.PID import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab3/README.md

# class PID:
#     def __init__(self, Kp = 0., Ki = 0., Kd = 0.):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.bias = 0.

#         self.set_point = 0.
#         self.integral = 0.
#         self.prev_error = 0.
#         self.dt = 0.1
#         self.prev_dt = 0.1

#         self.P_term = 0.
#         self.I_term = 0.
#         self.D_term =0.

#         self.integral_cutoff = None

#     def update(self, feedback, dt=None):
#         error = self.set_point - feedback
#         if dt is not None:
#             if dt == 0:
#                 self.dt = self.prev_dt
#             else:
#                 self.dt = dt
#                 self.prev_dt = dt

#         self.P_term = self.Kp * error + self.bias

#         self.integral += error * self.dt
#         self.apply_integral_cutoff()
#         self.I_term = self.Ki * self.integral

#         d_error = error - self.prev_error
#         self.D_term = self.Kd * d_error/self.dt

#         control_signal = self.P_term + self.I_term + self.D_term

#         return control_signal 
    
#     def apply_integral_cutoff(self):
#         if self.integral_cutoff != None:
#             sign = 1.0
#             if self.integral < 0.:
#                 sign = -1.0
#             if abs(self.integral) > self.integral_cutoff:
#                 self.integral = self.integral_cutoff * sign

#     def set_PID_gains(self, Kp, Ki, Kd, bias = 0.):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.bias = bias

class WallFollowerAlgorithm():
    def __init__(self,  set_dist2wall = 1., 
                        lookahead_dist = 0., 
                        angle_increment = 0.00435, 
                        deg_a = 85,
                        deg_b = 90,
                        Kp = 0.2, Ki = 0.01, Kd = 0.):
        # Controller
        self.set_dist2wall = set_dist2wall
        self.steering_pid = PID(Kp, Ki, Kd)
        self.steering_pid.set_point = self.set_dist2wall

        # WallFollower Params
        self.deg_a = deg_a
        self.deg_b = deg_b
        self.lookahead_dist = lookahead_dist
        self.angle_increment = angle_increment

    def find_pred_dist2wall(self):
        deg_x = [self.deg_a, self.deg_b]
        radian_x = map(lambda x: x/180 * math.pi, deg_x)
        theta = deg_x[1] - deg_x[0]
        index_x = map(lambda rad: len(self.ranges)//2 - rad//self.angle_increment, radian_x)
        distance_x = map(lambda x: self.ranges[x], index_x)

        distance_a = distance_x[0]
        distance_b = distance_x[1]

        alpha = math.atan((distance_a * math.cos(theta) - distance_b)/(distance_a * math.sin(theta)))
        dist2wall = distance_b * math.cos(alpha)
        lookahead_dist = self.linX * self.dt
        pred_dist2wall = dist2wall + lookahead_dist * math.sin(alpha)
        self.pred_dist2wall = pred_dist2wall

    def apply_controller(self):
        steering_angle = self.steering_pid.update(self.pred_dist2wall, self.dt)
        speed = self.pred_dist2wall
        self.twist = (speed, steering_angle)
    
    def update(self, ranges, odom, dt):
        self.ranges = ranges
        self.dt = dt
        self.linX = odom[3]
        self.find_pred_dist2wall()
        self.apply_controller()
        return self.twist

class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')
        # Laser Scans Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, '/wall_follower/scan', self.scan_callback, 10)
        self.scan_subscriber  # prevent unused variable warning
        self.ranges = []
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/wall_follower/odom', self.odom_callback, 10)
        self.odom_subscriber
        # Drive Publisher
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/aeb/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # WallFollower Algorithm
        self.wall_follower = WallFollowerAlgorithm()

    def scan_callback(self, scan_msg):
        self.current_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec*1e-9
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angulr.z
        self.odom = (x, y, yaw, linX, angZ)
    
    def publish_drive_msg(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.twist[0]
        drive_msg.drive.steering_angle = self.twist[1]
        self.drive_publisher.publish(drive_msg)

    def timer_callback(self):
        self.run()

    def run(self):
        dt = self.current_time - self.last_time
        if dt == 0:
            return
        self.twist = self.wall_follower.update(self.ranges, self.odom, dt)
        self.publish_drive_msg()
        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)

    wall_follower = WallFollowerNode()

    rclpy.spin(wall_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
