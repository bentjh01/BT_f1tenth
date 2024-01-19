import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class AutomaticEmergencyBraking_Algorithm():
    def __init__(self, time2collision_threshold_0 = 3):
        self.time2collision_threshold_0= time2collision_threshold_0
        self.last_ranges = None
        self.angle_increment = None
        self.speed_gain = 1

    def update(self, ranges, odom):
        self.ranges = ranges
        self.linX = odom[3]
        self.angle_max = self.angle_increment * len(ranges)
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
        # Drive
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # AEB Algorithm
        self.aeb = AutomaticEmergencyBraking_Algorithm()
        self.twist = (0, 0)

    def scan_callback(self, scan_msg):
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angulr.z
        self.odom = (x, y, yaw, linX, angZ)
        
    def drive_publish(self, drive_msg):
        self.drive_publisher.publish(drive_msg)

    def timer_callback(self):
        self.run()

    def publish_drive_msg(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.twist[0]
        drive_msg.drive.steering_angle = self.twist[1]
        self.drive_publisher.publish(drive_msg)

    def run(self):
        self.twist = map(lambda x : x * self.aeb.update(self.ranges, self.odom), self.twist)
        self.publish_drive_msg()

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
