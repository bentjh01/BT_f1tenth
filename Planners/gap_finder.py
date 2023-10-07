import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from controllers import PID

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

        # Car Control Signal
        self.speed = 0.
        self.steering_angle = 0.

        # FindTheGap Params
        self.safety_bubble = 0.4 # [m]
        self.margin_bubble = 5 # [number of indexes to the left and right]
        self.max_range = 0. # Maximum distance

        # self.count = 0
        # self.last_time = 0.
        # self.lookahead_dist = 0.
        # self.ros_time = 0.
        
        # self.ttc_threshhold = 1
        # self.e_brake = False
        # self.linX = 0.

    def scan_init(self, scan_msg):
        self.angle_increment = scan_msg.angle_increment
        self.angle_max = scan_msg.angle_max
        self.angle_min = scan_msg.angle_min
        self.pid.integral_cutoff = self.angle_max
        self.len_range = len(scan_msg.ranges)

    def get_time(self, scan_msg):
        sec = scan_msg.header.stamp.sec
        nsec = scan_msg.header.stamp.nanosec
        ros_time = sec + nsec/1e9
        return ros_time

    def scan_callback(self, scan_msg):
        #TODO-check if ranges are in the order specified earlier. 
        if self.count < 1:
            self.scan_init(scan_msg)
            self.count += 1
        self.ros_time = self.get_time(scan_msg)
        self.ranges = scan_msg.ranges

    # def odom_callback(self, odom_msg):
    #     self.linX = odom_msg.twist.twist.linear.x
    
    def apply_safetybubble(self):
        self.mod_ranges = self.ranges.copy()
        min_range_index = self.mod_ranges.index(min(self.mod_ranges))
        r_thetha = self.mod_ranges[min_range_index] * self.angle_increment
        n_cutoff = self.safety_bubble/r_thetha//2
        for i in range(int(min_range_index-n_cutoff), int(min_range_index+n_cutoff,1)):
            self.mod_ranges[i] = 0.0

    def rolling_mean(self, avg, x_n, n):
        n = n + 1 # index correction
        avg = ((n-1) * avg + x_n)/n
        return avg
        
    def get_max_range_index (self):
        # this function returns the index of the range that has the highest average in n*2 consecutive ranges. 
        avg_ranges = []
        n = self.margin_bubble
        for i, r in enumerate(self.mod_ranges):
            temp_list[len(temp_list)//2+1] = r
            if i < n:
                temp_list = [0.]*(n-i) + self.mod_ranges[i:i+n+1]
            elif i > self.len_range - n:
                x = self.len_range-i
                temp_list = self.mod_ranges[i-n:i+x] + [0.]*(x-1)
            else:
                temp_list = self.mod_ranges[i-n:i+n+1]
            avg = np.mean(temp_list)
            avg_ranges.append(avg)
        max_range_index = avg_ranges.index(max(avg_ranges))
        self.steering_angle = max_range_index * self.angle_increment # careful need to be negative value for left turns
        self.max_range = self.mod_ranges[max_range_index]

    
    
    def get_drive_msg(self, steering_angle, dt):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        return drive_msg
    
    # def find_dist2wall(self, deg_a = 89, deg_b = 90):
    #     theta = (deg_b - deg_a)/180 * math.pi

    #     dist_a = self.get_dist_at_deg(deg_a)
    #     dist_b = self.get_dist_at_deg(deg_b)

    #     alpha = math.atan((dist_a * math.cos(theta) - dist_b)/(dist_a * math.sin(theta)))
    #     dist2wall = dist_b * math.cos(alpha)
    #     pred_dist2wall = dist2wall + self.lookahead_dist * math.sin(alpha)
    #     return dist2wall, pred_dist2wall
    
    # def clip_control_signal(self, control_signal):
    #     sign = 1.0
    #     if control_signal < 0:
    #         sign = -1.0
    #         control_signal = abs(control_signal)
    #     if control_signal > self.angle_max:
    #         control_signal = self.angle_max
    #     return sign * control_signal 

    # def collision(self): #Time To Collision
    #     for i, radius in enumerate(self.ranges):
    #         angle = self.angle_max - i * self.angle_increment
    #         r_dot = max(-1 * self.linX * math.cos(angle), 0)
    #         if r_dot != 0:
    #             ttc = radius / r_dot
    #         else:
    #             ttc = float('inf')
    #         if ttc <= self.ttc_threshhold:
    #             self.e_brake = True
    #             break

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

    wall_follower = Wall_Follower()

    rclpy.spin(wall_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()