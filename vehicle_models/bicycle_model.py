"""
Inspired from the work of : Atsushi Sakai (@Atsushi_twi)
"""


import math


class AbstractCar:

    def __init__(self, yaw=-1.57, v=0.0, dt=1/10, wb=26):
        self.img = self.IMG
        self.x = self.START_POS_X  # + Settings.CAR_X_OFFSET
        self.y = self.START_POS_Y  # + Settings.CAR_Y_OFFSET
        self.yaw = yaw
        self.v = v
        self.wb = wb
        self.dt = dt
        self.rear_x = self.x - ((self.wb) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wb) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += int(self.v * math.cos(self.yaw) * self.dt)
        self.y += int(self.v * math.sin(self.yaw) * self.dt)
        self.yaw += self.v / self.wb * math.tan(delta) * self.dt
        self.v += a * self.dt
        self.rear_x = int(self.x - ((self.wb) * math.cos(self.yaw)))
        self.rear_y = int(self.y - ((self.wb) * math.sin(self.yaw)))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
