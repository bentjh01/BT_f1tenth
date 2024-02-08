#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class AutomaticEmergencyBraking_Algorithm():
    def __init__(self, time2collision_threshold_0 = 3, field_of_view = 90):
        self.time2collision_threshold_0= time2collision_threshold_0
        self.last_ranges = None
        self.angle_increment = None
        self.speed_gain = 1
        self.field_of_view = field_of_view

    def update(self, ranges, odom):
        self.ranges = ranges
        self.linX = odom[3]
        # self.angle_max = self.angle_increment * len(ranges)
        self.set_field_of_vision()
        self.calculate_time2collision()
        return self.speed_gain

    def calculate_time2collision(self):
        for i, radius in enumerate(self.ranges):
            angle = self.angle_increment * len(self.ranges)//2 - i * self.angle_increment
            r_dot = max(self.linX * math.cos(angle), 0)
            if r_dot != 0:
                ttc = radius / r_dot
            else:
                ttc = float('inf')
            if ttc <= self.time2collision_threshold_0:
                self.speed_gain = 0
                break
    
    def set_field_of_vision(self):
        lower_bound_index = len(self.ranges)//2 - self.field_of_view//self.angle_increment//2
        upper_bound_index = lower_bound_index + self.field_of_view//self.angle_increment
        self.ranges = self.ranges[lower_bound_index:upper_bound_index]

class AutomaticEmergencyBrakingNode(Node):
    def __init__(self):
        super().__init__('AEB')
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber
        self.scan_init = False
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # WallFollower Subscriber
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, '/wall_follower/drive', self.drive_callback, 10)
        self.drive_subscriber
        self.set_drive = AckermannDriveStamped()
        # Drive Publisher
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # AEB Algorithm
        self.aeb = AutomaticEmergencyBraking_Algorithm()
        self.cmd_drive = AckermannDriveStamped()

    def scan_callback(self, scan_msg):
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angular.z
        self.odom = (x, y, yaw, linX, angZ)
    
    def drive_callback(self, ackermann_msg):
        self.set_drive = ackermann_msg
        self.run()

    def run(self):
        speed_gain = self.aeb.update(self.ranges, self.odom)
        self.cmd_twist = self.set_twist
        self.cmd_twist.drive/speed *= speed_gain
        self.drive_publisher.publish(self.cmd_twist)

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
