# radians

class SimScanConfig():
    # if index == 0, then on simulator, angle == back right
    def __init__(self):
        self.max_angle = 2.3499999046325684
        self.min_angle = -2.3499999046325684
        self.angle_increment = 0.004351851996034384
        self.ranges_max = 30
        self.ranges_min = 0
        self.ranges_count = 1080

    def reorientate(self, ranges:list):
        return ranges

class CarScanConfig():
    #TODO - calibrate
    def __init__(self):
        self.max_angle = 2.3499999046325684
        self.min_angle = -2.3499999046325684
        self.angle_increment = 0.004351851996034384
        self.ranges_max = 30
        self.ranges_min = 0
        self.ranges_count = 1080

    def reorientate(self, ranges:list):
        return ranges.reverse()