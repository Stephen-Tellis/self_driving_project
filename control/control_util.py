import numpy as np


class TargetCourse:
    """
    Navigates through waypoints in the true path to find the next closest point for the controller.
    """

    def __init__(self, cx, cy, k, Lfc):
        self.k = k
        self.Lfc = Lfc
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def _search_target_index(self, rear_x, rear_y, v):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [rear_x - icx for icx in self.cx]
            dy = [rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = self.calc_distance(rear_x, rear_y, self.cx[ind],
                                                     self.cy[ind])
            while True:
                distance_next_index = self.calc_distance(rear_x, rear_y, self.cx[ind + 1],
                                                         self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = self.k * v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > self.calc_distance(rear_x, rear_y, self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

    def calc_distance(self, rear_x, rear_y, point_x, point_y):
        dx = rear_x - point_x
        dy = rear_y - point_y
        return np.hypot(dx, dy)
