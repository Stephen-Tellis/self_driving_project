"""
Reference  : http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf
           : Atsushi Sakai (@Atsushi_twi)
           : (https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
"""

# Local and Self made classes
# from control.control_util import TargetCourse
from logger.logger_config import setup_logger

# Python Libs
import numpy as np

# Setup Logging
logger = setup_logger(__name__)

# Controller Gains
K = 2


class SteeringControl:

    def __init__(self, cx, cy, wb, cyaw):
        logger.info("Stanley Controller")
        self.wb = wb
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw

    @classmethod
    def normalize_angle(cls, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def search_target_index(self, x, y, yaw, v):
        return self._calc_target_index(x, y, yaw)

    def control(self, prev_ind, x, y, yaw, v, wb):
        current_idx, error_front_axle = self._calc_target_index(
            x, y, yaw)

        if prev_ind >= current_idx:
            current_idx = prev_ind

        # theta_e corrects the heading error
        theta_e = SteeringControl.normalize_angle(
            self.cyaw[current_idx] - yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(K * error_front_axle, v)
        # Steering control
        delta = theta_e + theta_d
        logger.debug(f"{delta}, {theta_e}, {theta_d}")
        return delta, current_idx

    def _calc_target_index(self, x, y, yaw):

        # Search nearest point index
        dx = [x - icx for icx in self.cx]
        dy = [y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2),
                          -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
