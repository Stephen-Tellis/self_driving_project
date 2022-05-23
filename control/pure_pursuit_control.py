"""
Inspired by: (https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
           : Atsushi Sakai (@Atsushi_twi)
"""
# Local and Self made classes
from control.control_util import TargetCourse
from logger.logger_config import setup_logger
from render_engine.env_renderer import Settings

# Python Libs
import math

# Setup Logging
logger = setup_logger(__name__)

# Controller Gains
K = 0.1  # Look forward gain
LFC = 45  # Look forward distances in px


class SteeringControl:

    def __init__(self, cx, cy, wb, cyaw):
        logger.info("Pure Pursuit Controller")
        self.trajectory = TargetCourse(cx, cy, K, LFC)

    def search_target_index(self, x, y, yaw, v):
        return self.trajectory._search_target_index(x, y, v)

    def control(self, prev_ind, x, y, yaw, v, wb):
        rear_x = int(x - ((Settings.WB) * math.cos(yaw)))
        rear_y = int(y - ((Settings.WB) * math.sin(yaw)))
        current_idx, Lf = self.trajectory._search_target_index(
            rear_x, rear_y, v)

        if prev_ind >= current_idx:
            current_idx = prev_ind

        if current_idx < len(self.trajectory.cx):
            tx = self.trajectory.cx[current_idx]
            ty = self.trajectory.cy[current_idx]
        else:  # toward goal
            tx = self.trajectory.cx[-1]
            ty = self.trajectory.cy[-1]
            current_idx = len(self.trajectory.cx) - 1

        alpha = math.atan2(ty - rear_y, tx -
                           rear_x) - yaw

        delta = math.atan2(2.0 * wb * math.sin(alpha) / Lf, 1.0)
        logger.debug(f"{alpha=},{delta=}")
        return delta, current_idx
