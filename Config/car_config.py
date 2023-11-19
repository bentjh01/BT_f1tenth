class F1tenthCar:
    def __init__(self): 
        self.steering_angle_limit = 0.26
        self.steering_change_limit = 0.13 # guess
        self.speed_limit = 2.0
        self.acceleration_limit = 1.0
        self.min_rpm = 600 # found empirically

    def apply_speed_limits(self, set_speed, current_speed, dt):
        speed_delta = set_speed-current_speed

        cmd_speed = set_speed
        if abs(speed_delta) > self.acceleration_limit*dt:
            accelerating = True
            if speed_delta < 0:
                accelerating = False
            if accelerating:
                cmd_speed = current_speed+self.acceleration_limit*dt
            else:
                cmd_speed = current_speed-self.acceleration_limit*dt
        return cmd_speed
    
    def apply_steering_limits(self, set_steering_angle, current_steering_angle, dt):
        steering_angle_delta = set_steering_angle-current_steering_angle
        cmd_steering_angle = set_steering_angle
        if abs(steering_angle_delta) > self.steering_angle_limit*dt:
            steering_left = True
            if steering_angle_delta < 0:
                steering_left = False
            if steering_left:
                cmd_steering_angle = current_steering_angle + self.steering_angle_limit*dt
            else:
                cmd_steering_angle = current_steering_angle - self.steering_angle_limit*dt
        return cmd_steering_angle
