"""
Inspired from the work of : Atsushi Sakai (@Atsushi_twi)
"""


import numpy as np

STEER_LIM_MIN = -1.57
STEER_LIM_MAX = 1.57

class AbstractCar:

    def __init__(self, yaw=-1.57, v=0.0, dt=1/10, wb=26):
        self.img = self.IMG
        self.x = self.START_POS_X
        self.y = self.START_POS_Y
        self.yaw = yaw
        self.v = v
        self.wb = wb
        self.dt = dt

    def update(self, a, delta):
        delta = np.clip(delta, STEER_LIM_MIN, STEER_LIM_MAX)
        self.x += int(self.v * np.cos(self.yaw) * self.dt)
        self.y += int(self.v * np.sin(self.yaw) * self.dt)
        self.yaw += self.v / self.wb * np.tan(delta) * self.dt
        self.yaw = AbstractCar.normalize_angle(self.yaw)
        self.v += a * self.dt

    @classmethod
    def normalize_angle(cls, angle):
        """
        Normalize an angle to [-pi, pi].
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
