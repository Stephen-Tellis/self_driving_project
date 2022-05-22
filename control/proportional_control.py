# Controller Gains
Kp = 1


class LongitudionalControl:

    def control(self, target, current):
        a = Kp * (target - current)
        return a
