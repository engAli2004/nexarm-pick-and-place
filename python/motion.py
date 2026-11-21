"""
motion.py - smooth, NON-BLOCKING trajectories.

The arm is never sent a big jump. A Trajectory is planned once (start, goal, duration) and
then SAMPLED at the current time on every FSM tick: the FSM sends the sampled point to the
arm as a short segment (100 ms). Nothing waits or sleeps, so the FSM loop keeps reading the
inputs (E-stop, buttons, gripper feedback) while the arm moves.

Speed profile: cosine (smooth start and stop)
    s(t) = (1 - cos(pi * t / T)) / 2            0 -> 1
    speed is zero at both ends and highest in the middle -> no jerk on the servo gears.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """Gripper tip in the robot base frame: mm and degrees."""
    x: float
    y: float
    z: float
    pitch: float = -90.0      # -90 = gripper pointing straight down
    roll: float = 0.0

    def dist(self, other: "Pose") -> float:
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))

    def moved(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> "Pose":
        return Pose(self.x + dx, self.y + dy, self.z + dz, self.pitch, self.roll)

    def above(self, height: float) -> "Pose":
        return Pose(self.x, self.y, self.z + height, self.pitch, self.roll)


class Trajectory:
    """Straight line from `start` to `goal` (and gripper claw0 -> claw1) with a cosine profile."""

    def __init__(self, start: Pose, goal: Pose, claw0: float, claw1: float,
                 speed_mm_s: float, t_now: float, min_time: float = 0.5) -> None:
        self.start, self.goal = start, goal
        self.claw0, self.claw1 = claw0, claw1
        length = start.dist(goal)
        turn = max(abs(goal.pitch - start.pitch), abs(goal.roll - start.roll))
        claw_time = abs(claw1 - claw0) / 80.0                    # gripper: 80 deg/s
        # average speed of a cosine profile = peak / (pi/2): duration from the PEAK speed
        self.duration = max(min_time, length * math.pi / 2 / speed_mm_s, turn / 45.0, claw_time)
        self.t0 = t_now
        self.t_end = t_now + self.duration

    def sample(self, t_now: float) -> tuple[Pose, float, bool]:
        """Point to command at time t_now: (pose, claw, finished)."""
        u = min(1.0, max(0.0, (t_now - self.t0) / self.duration))
        s = (1.0 - math.cos(math.pi * u)) / 2.0
        a, b = self.start, self.goal
        pose = Pose(a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s,
                    a.pitch + (b.pitch - a.pitch) * s, a.roll + (b.roll - a.roll) * s)
        claw = self.claw0 + (self.claw1 - self.claw0) * s
        return pose, claw, u >= 1.0
