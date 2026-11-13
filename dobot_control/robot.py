"""Nonblocking simulator. SDK backend is imported only for real hardware."""
from .control import Pose


class SimRobot:
    real = False

    def __init__(self, pose=None):
        self.pose = pose or Pose(200, 0, 50, 0)
        self.suction = False
        self.moves = []
        self.pump_events = []
        self.stop_count = 0

    def read_pose(self):
        return self.pose

    def move(self, target, speed, deadline=None):
        self.moves.append((self.pose, target))
        self.pose = target

    def is_idle(self):
        return True

    def stop(self):
        self.stop_count += 1

    def set_suction(self, enabled, deadline=None):
        self.suction = enabled
        self.pump_events.append(enabled)

    def close(self):
        pass
