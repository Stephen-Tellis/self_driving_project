
class LongitudionalControl:
    def __init__(self, Kp):
        self.kp = Kp

    def control(self, target, current):
        a = self.kp * (target - current)

        return a
