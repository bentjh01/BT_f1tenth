import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class AutomaticEmergencyBraking(Node):
    def __init__(self):
        super().__init__('AEB')
        self.time2collision = 3
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber
        self. ranges = []
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # Drive
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # ESTOP
        self.pub_rate

    def scan_callback(self):
        pass

    def odom_callback(self):
        pass

    def drive_publish(self):
        pass

    def timer_callback(self):
        pass 