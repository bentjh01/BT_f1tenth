def collision(self): #Time To Collision
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