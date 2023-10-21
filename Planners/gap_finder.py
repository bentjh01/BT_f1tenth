import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from Controllers import PID
from Config.scan_config import SimScanConfig

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md

# 1. Find the closest point
#       -  ensure angles are from -pi to pi i.e. [0] is forward, [1] and [-1] are increments t the right and left respectively. 
#       - take only theta angle infront of car. could be determined by a lookahead distance and the max steering angle.
# 2. Draw safety bubble around closest point. 
#       - assume r * angle_increment = straight line with midpoint at range.
#       - number of indexes in safety bubble = safety_bubble//(r * angle_increment) 
# 3. Set closest point ranges to zero i.e. no-go zone. 
# 4. go to furtherest n-consecutive ranges
#       - calculate the average of n-consecutive ranges to left and right of a range. 
#       - set steering angle = angle increment * index

# 1. Find the closest point
#       -  ensure angles are from -pi to pi i.e. [0] is forward, [1] and [-1] are increments t the right and left respectively. 
#       - take only theta angle infront of car. could be determined by a lookahead distance and the max steering angle.
# 2. Draw safety bubble around closest point. 
#       - assume r * angle_increment = straight line with midpoint at range.
#       - number of indexes in safety bubble = safety_bubble//(r * angle_increment) 
# 3. Set closest point ranges to zero i.e. no-go zone. 
# 4. go to largest n-consecutive ranges
#       - use a threshold distance. 
#       - set everthing below the threshold distance to 0.
#       - 
# Scale linear velocity to the distance to closest obstacle

scanConfig = SimScanConfig()

class GapFinder(Node):

    def __init__(self):
        super().__init__('gap_finder')
        # Laser Scans Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber  # prevent unused variable warning
        self.ranges = [] #TODO-find the position of the first index for the sim and the car. 
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # Drive Publisher
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # FindTheGap Params
        self.safety_bubble = 0.4 # [m]
        self.margin_bubble_count = 5 # [number of indexes to the left and right]
        self.max_range = 0. # Maximum distance 
        # Drive speed controller
        self.speed_controller = PID()
        self.speed_controller.Kp = 1.0
        self.speed_controller.Ki = 0.0
        self.speed_controller.Kd = 0.5
        self.speed_controller.dt = 1/self.pub_rate
        self.min_range = 0
        # Car Control Signal
        self.speed = 0.
        self.steering_angle = 0.

    def get_time(self, scan_msg):
        sec = scan_msg.header.stamp.sec
        nsec = scan_msg.header.stamp.nanosec
        ros_time = sec + nsec/1e9
        return ros_time

    def scan_callback(self, scan_msg):
        self.ros_time = self.get_time(scan_msg)
        self.ranges = scanConfig.reorientate(scan_msg.ranges)

    # def odom_callback(self, odom_msg):
    #     self.linX = odom_msg.twist.twist.linear.x
    
    def apply_safetybubble(self):
        self.modified_ranges = self.ranges.copy()
        min_range_index = self.modified_ranges.index(min(self.modified_ranges))
        arc_thetha = self.modified_ranges[min_range_index] * self.angle_increment
        n_cutoff = self.safety_bubble/arc_thetha//2
        for i in range(int(min_range_index-n_cutoff), int(min_range_index+n_cutoff,1)):
            self.modified_ranges[i] = 0.0
        self.min_range = self.ranges[min_range_index]

    def rolling_mean(self, avg, x_n, n):
        n = n + 1 # index correctioni
        avg = ((n-1) * avg + x_n)/n
        return avg
    
    def get_max_range_index (self):
        # this function returns the index of the range that has the highest average in n*2 consecutive ranges. 
        mean_ranges = []
        n = self.margin_bubble_count
        mean = 0.0
        for i, range in enumerate(self.modified_ranges):
            mean += range/n
            if i >= n:
                mean -= self.modified_ranges[i-n]/n
            mean_ranges.append(mean)
        max_range_index = mean_ranges.index(max(mean_ranges))
        self.steering_angle = self.ranges_angle_2_base_angle(max_range_index)
        self.max_range = self.modified_ranges[max_range_index]

    def ranges_angle_2_base_angle(self, index):
        # receive index wrt to ranges and output angle (deg) wrt to base frame
        # assumes left-back 
        ranges_mid_angle = (scanConfig.max_angle-scanConfig.min_angle)/2
        ranges_angle = index*scanConfig.angle_increment
        base_angle = ranges_angle-ranges_mid_angle
        return base_angle  
    
    def speed_control(self):
        speed = self.speed_controller.update(self.min_range)
        self.speed = speeds
    
    def generate_drive_msg(self, steering_angle, dt):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        return drive_msg
    
    def timer_callback(self):
        if self.count >= 1:
            dt = self.ros_time - self.last_time
            dist2wall, pred_dist2wall = self.find_dist2wall(89, 90)
            self.collision()
            drive_msg = self.get_drive_msg(pred_dist2wall, dt)
            self.drive_publisher.publish(drive_msg)
            # print(f"steering_angle = {round(drive_msg.drive.steering_angle,2)}, dist2wall = {round(dist2wall, 2)}")
            # print(f"{self.e_brake}")
            self.lookahead_dist = drive_msg.drive.speed * dt
            self.last_time = self.ros_time

def main(args=None):
    rclpy.init(args=args)

    wall_follower = GapFinder()

    rclpy.spin(wall_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()