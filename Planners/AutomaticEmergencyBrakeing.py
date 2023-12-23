import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class AutomaticEmergencyBraking(Node):
    def __init__(self):
        super().__init__('AEB')
        self.time2collision = 3
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber
        self.scan_init = False
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # Drive
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/aeb/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # ESTOP
        self.estop_publisher = self.create_publisher(String, '/mux_cmd', 10)
        self.e_brake = False
        self.ttc_threshhold = 1

    def scan_callback(self, scan_msg):
        if not self.scan_init:
            self.scan_angle_increment = scan_msg.angle_increment
            self.scan_angle_max = scan_msg.angle_max
            self.scan_angle_min = scan_msg.angle_min
            self.len_range = len(scan_msg.ranges)
        ranges = scan_msg.ranges
        return ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angulr.z
        pose = [x, y, yaw, linX, angZ]
        return pose
        
    def drive_publish(self, drive_msg):
        self.drive_publisher.publish(drive_msg)

    def mux_cmd_publish(self, mux_cmd_msg):
        self.drive_publisher.publish(mux_cmd_msg) 

    def collision(self):
        for i, radius in enumerate(self.ranges):
            angle = self.angle_max - i * self.angle_increment
            r_dot = max(-1 * self.linX * math.cos(angle), 0)
            if r_dot != 0:
                ttc = radius / r_dot
            else:
                ttc = float('inf')
            if ttc <= self.ttc_threshhold:
                self.e_brake = True
                break

    def timer_callback(self):
        self.collision()
        if self.e_brake:
            self.drive_publish(AckermannDriveStamped())
            self.mux_cmd_publish(self.get_name())

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBraking()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()