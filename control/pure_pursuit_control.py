"""
Inspired by: (https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
           : Atsushi Sakai (@Atsushi_twi)
"""

import math


class SteeringControl:

    def __init__(self, state, trajectory) -> None:
        self.state = state
        self.trajectory = trajectory

    def control(self, pind):
        ind, Lf = self.trajectory.search_target_index(self.state)

        if pind >= ind:
            ind = pind

        if ind < len(self.trajectory.cx):
            tx = self.trajectory.cx[ind]
            ty = self.trajectory.cy[ind]
        else:  # toward goal
            tx = self.trajectory.cx[-1]
            ty = self.trajectory.cy[-1]
            ind = len(self.trajectory.cx) - 1

        alpha = math.atan2(ty - self.state.rear_y, tx -
                           self.state.rear_x) - self.state.yaw

        delta = math.atan2(2.0 * self.state.wb * math.sin(alpha) / Lf, 1.0)

        return delta, ind
