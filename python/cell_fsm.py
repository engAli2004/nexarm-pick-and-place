"""
cell_fsm.py - the brain of the NexArm pick-and-place cell: a finite state machine.

Task: a part travels between two taught stations A and B (pick at one, place at the other),
N times, autonomously. Every arrival is measured -> accuracy and repeatability (ISO 9283).

Rules of this FSM
  * ONE method, tick(), is called every 50 ms by the HMI. It never blocks and never sleeps.
  * Every tick it FIRST checks the inputs that matter in every state (E-stop from the HMI,
    E-stop from the arm's KEY2, lost / stale telemetry), THEN runs the current state with a
    single `match self.state:` statement.
  * A state only decides; motion is done by sampling a Trajectory (motion.py) and sending
    a short segment to the arm every 100 ms.
  * Branch logic from sensors:
      - gripper feedback after closing: part held -> continue, empty -> RETRY once -> FAULT
      - gripper feedback while carrying: part lost -> FAULT "part dropped"
      - position feedback after each move: arrived -> continue, too far -> FAULT (jam/collision)
      - KEY1 / START -> start,  KEY2 / E-STOP -> ESTOP (latched until RESET)
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from enum import Enum, auto

from motion import Pose, Trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(HERE, "cell_config.json")
LOG_DIR = os.path.join(HERE, "logs")


class State(Enum):
    INIT = auto()            # connect, handshake, wait for valid telemetry
    HOMING = auto()          # safe home: lift first, then go to HOME
    IDLE = auto()            # waiting: START / KEY1, manual jog and teaching allowed
    JOG = auto()             # manual move requested from the HMI
    CALIB_GRIP = auto()      # close the empty gripper once and remember the reading
    APPROACH = auto()        # above the pick station
    DESCEND_PICK = auto()    # straight down to the part
    GRIP = auto()            # close the gripper
    VERIFY_GRIP = auto()     # sensor check: is the part really in the gripper?
    RETRY = auto()           # open, lift a little, try the grip again
    LIFT_PICK = auto()       # straight up with the part
    TRANSFER = auto()        # above the place station
    DESCEND_PLACE = auto()   # straight down
    RELEASE = auto()         # open the gripper
    LIFT_PLACE = auto()      # straight up, empty
    CYCLE_END = auto()       # count the trip, decide: next trip or go home
    ESTOP = auto()           # emergency stop (latched)
    FAULT = auto()           # error that needs the operator (RESET)


MOTION_STATES = {State.HOMING, State.JOG, State.CALIB_GRIP, State.APPROACH, State.DESCEND_PICK,
                 State.GRIP, State.RETRY, State.LIFT_PICK, State.TRANSFER, State.DESCEND_PLACE,
                 State.RELEASE, State.LIFT_PLACE}


@dataclass
class CellConfig:
    """Everything the operator can change. Saved in cell_config.json."""
    station_a: list = field(default_factory=lambda: [201.0, 2.0, 20.0, -90.0, 0.0])   # taught 12 Nov
    station_b: list = field(default_factory=lambda: [301.0, 2.0, 20.0, -90.0, 0.0])
    home: list = field(default_factory=lambda: [200.0, 0.0, 180.0, -90.0, 0.0])
    approach_height: float = 80.0      # mm above a station before going straight down
    safe_z: float = 100.0              # never move sideways below this when going home
    place_lift: float = 2.0            # release this much above the taught height
    claw_open: float = -50.0
    claw_grip: float = 0.0
    claw_empty: float = 0.0            # gripper reading when it closes on NOTHING (CALIB GRIP)
    claw_tolerance: float = 6.0        # |reading - claw_empty| above this = a part is held
    verify_grip: bool = True           # use the gripper sensor branch
    grip_retries: int = 1
    speed_travel: float = 120.0        # mm/s (peak)
    speed_slow: float = 50.0           # mm/s (peak), last 80 mm to the part
    arrive_tol: float = 8.0            # mm: arrived
    fault_tol: float = 20.0            # mm: still this far after settling -> FAULT
    # virtual workspace envelope (mm): no setpoint outside it is ever sent
    x_min: float = 100.0
    x_max: float = 360.0
    y_max: float = 220.0
    z_min: float = 15.0
    z_max: float = 260.0

    @staticmethod
    def load() -> "CellConfig":
        cfg = CellConfig()
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE) as f:
                for k, v in json.load(f).items():
                    if hasattr(cfg, k):
                        setattr(cfg, k, v)
        return cfg

    def save(self) -> None:
        with open(CONFIG_FILE, "w") as f:
            json.dump(asdict(self), f, indent=2)


def pose_of(values: list) -> Pose:
    return Pose(*values)


class CellFSM:
    SEND_PERIOD = 0.10          # s between two segments sent to the arm
    SEGMENT_MS = 120            # each segment lasts a bit longer than the period: no gaps
    TEL_TIMEOUT = 0.6           # s without fresh telemetry -> link lost
    SETTLE_MIN = 0.35           # s to wait at a measurement point before reading the position
    SETTLE_TIMEOUT = 2.0        # s to reach arrive_tol after the trajectory ended
    GRIP_TIME = 0.6             # s for the fingers to close / open completely

    def __init__(self, link, cfg: CellConfig | None = None) -> None:
        self.link = link
        self.cfg = cfg or CellConfig.load()
        self.state = State.INIT
        self.t_state = time.time()          # when the current state was entered
        self.history: deque[str] = deque(maxlen=12)
        self.log_lines: deque[str] = deque(maxlen=200)
        self.requests: deque[tuple[str, object]] = deque()   # from the HMI (thread-safe append)
        # commanded setpoint (what was last sent) and motion
        self.cmd_pose: Pose | None = None
        self.cmd_claw = self.cfg.claw_open
        self.traj: Trajectory | None = None
        self.next_after_move: State | None = None
        self.last_send = 0.0
        self.final_sent = False
        # job
        self.object_at = "A"
        self.trips_target = 10
        self.trips_done = 0
        self.stop_after_cycle = False
        self.retries_left = 0
        self.lost_count = 0
        self.last_checked = 0.0
        self.home_phase = 0
        self.alarm = ""
        self.estop_latched = False
        self.reset_pending = False
        self.init_sent = 0.0
        self.trip_t0 = 0.0
        self.key1_time = 0.0
        # results
        self.arrivals = {"A": [], "B": []}  # (target Pose, measured (x, y, z))
        self.cycle_times: list[float] = []
        self.grip_failures = 0
        self.csv_path = ""
        self._enter(State.INIT)

    # ------------------------------------------------------------------ helpers
    def request(self, name: str, arg: object = None) -> None:
        """Called by the HMI (or KEY1): START, STOP, HOME, RESET, ESTOP, JOG, CLAW, SAVE, CALIB, OBJECT_AT."""
        self.requests.append((name, arg))

    def log(self, msg: str) -> None:
        self.log_lines.append(time.strftime("%H:%M:%S ") + msg)

    def _enter(self, new: State) -> None:
        self.history.append(new.name)
        if new is not self.state:
            self.log(f"{self.state.name} -> {new.name}")
        self.state, self.t_state = new, time.time()

    def _fault(self, why: str) -> None:
        self.alarm = why
        self.traj = None
        self.log("FAULT: " + why)
        self._enter(State.FAULT)

    def _station(self, name: str) -> Pose:
        return pose_of(self.cfg.station_a if name == "A" else self.cfg.station_b)

    def _other(self, name: str) -> str:
        return "B" if name == "A" else "A"

    def tel_fresh(self, now: float) -> bool:
        t = self.link.tel
        return t.received > 0 and now - t.received < self.TEL_TIMEOUT and t.fb_age_ms < 600

    def measured(self) -> Pose:
        t = self.link.tel
        return Pose(t.x, t.y, t.z, t.pitch, t.roll)

    def holding_part(self) -> bool:
        """Gripper sensor: closed on nothing -> reading near claw_empty; on a part -> it stops earlier."""
        if not self.cfg.verify_grip:
            return True
        return abs(self.link.tel.claw - self.cfg.claw_empty) > self.cfg.claw_tolerance

    def in_envelope(self, p: Pose) -> bool:
        c = self.cfg
        return c.x_min <= p.x <= c.x_max and abs(p.y) <= c.y_max and c.z_min <= p.z <= c.z_max

    def _move(self, goal: Pose, claw: float, speed: float, now: float) -> bool:
        """Plans a smooth move from the last commanded point. False (and FAULT) if outside the envelope."""
        if not self.in_envelope(goal):
            self._fault(f"target ({goal.x:.0f}, {goal.y:.0f}, {goal.z:.0f}) outside the workspace")
            return False
        start = self.cmd_pose or self.measured()
        self.traj = Trajectory(start, goal, self.cmd_claw, claw, speed, now)
        self.final_sent = False
        self.last_send = 0.0
        return True

    def _follow(self, now: float, settle_min: float = 0.0) -> bool:
        """Sends the next segment. True once the motion is finished AND the arm really arrived."""
        if self.traj is None:
            return True
        pose, claw, done = self.traj.sample(now)
        if now - self.last_send >= self.SEND_PERIOD or (done and not self.final_sent):
            self.link.move(pose.x, pose.y, pose.z, pose.pitch, pose.roll, claw, self.SEGMENT_MS)
            self.cmd_pose, self.cmd_claw, self.last_send = pose, claw, now
            self.final_sent = done
        if not done:
            return False
        waited = now - self.traj.t_end
        err = self.measured().dist(self.traj.goal)
        if waited >= max(settle_min, 0.15) and err <= self.cfg.arrive_tol:
            self.traj = None
            return True
        if waited > self.SETTLE_TIMEOUT:
            if err <= self.cfg.fault_tol:
                self.log(f"arrived {err:.1f} mm from target (slow settle)")
                self.traj = None
                return True
            self._fault(f"position not reached: {err:.0f} mm off (jam or collision?)")
        return False

    def _record_arrival(self, station: str) -> None:
        """Measurement point: taught target vs. position measured by the arm."""
        tgt = self._station(station)
        t = self.link.tel
        self.arrivals[station].append((tgt, (t.x, t.y, t.z)))
        err = math.hypot(t.x - tgt.x, t.y - tgt.y)
        if not self.csv_path:
            os.makedirs(LOG_DIR, exist_ok=True)
            self.csv_path = os.path.join(LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv"))
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(["time", "trip", "station", "action", "target_x", "target_y",
                                        "target_z", "real_x", "real_y", "real_z", "error_xy_mm"])
        action = "pick" if self.state is State.DESCEND_PICK else "place"
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow([time.strftime("%H:%M:%S"), self.trips_done + 1, station, action,
                                    tgt.x, tgt.y, tgt.z, t.x, t.y, t.z, f"{err:.2f}"])

    def stats(self, station: str) -> dict | None:
        """ISO 9283 in the table plane: AP = |mean - taught|, RP = mean(l) + 3 std(l)."""
        data = self.arrivals[station]
        n = len(data)
        if n == 0:
            return None
        tgt = data[0][0]
        mx = sum(m[0] for _, m in data) / n
        my = sum(m[1] for _, m in data) / n
        dists = [math.hypot(m[0] - mx, m[1] - my) for _, m in data]
        lbar = sum(dists) / n
        sd = math.sqrt(sum((d - lbar) ** 2 for d in dists) / (n - 1)) if n > 1 else 0.0
        return {"n": n, "ap": math.hypot(mx - tgt.x, my - tgt.y), "rp": lbar + 3 * sd,
                "dx": mx - tgt.x, "dy": my - tgt.y}

    # ------------------------------------------------------------------ the FSM
    def tick(self) -> None:
        now = time.time()
        reqs = list(self.requests)
        self.requests.clear()
        names = [r[0] for r in reqs]

        # ---- 1. inputs monitored in EVERY state, before anything else ----
        while not self.link.events.empty():
            ev = self.link.events.get()
            if ev.startswith("ESTOP"):
                self.estop_latched = True
                self.log(f"E-STOP from the arm ({ev.split(',')[-1]})")
            elif ev == "KEY1":
                self.key1_time = now
                self.log("KEY1 pressed on the arm")
                names.append("START")
                reqs.append(("START", None))
        while not self.link.messages.empty():
            self.log("arm: " + self.link.messages.get())
        if "ESTOP" in names and not self.estop_latched:
            self.estop_latched = True
            self.link.estop()                      # the arm latches it too
            self.log("E-STOP from the HMI")
        if self.link.tel.estop and self.link.tel.received and not self.reset_pending:
            self.estop_latched = True
        if self.estop_latched and self.state is not State.ESTOP:
            self.traj = None
            self._enter(State.ESTOP)
        if (self.state in MOTION_STATES or self.state is State.IDLE) and not self.tel_fresh(now):
            self._fault("no fresh position data from the arm (cable / power?)")
        if "STOP" in names and self.state not in (State.IDLE, State.FAULT, State.ESTOP, State.INIT):
            self.stop_after_cycle = True
            self.log("STOP: the arm will stop at the end of this trip")

        # ---- 2. the current state ----
        match self.state:
            case State.INIT:
                # handshake: ping + telemetry on, then wait for valid, fresh data
                if now - self.init_sent > 1.0:
                    self.link.ping()
                    self.link.telemetry(100)
                    self.init_sent = now
                if self.tel_fresh(now) and not self.link.tel.estop:
                    self.cmd_pose = self.measured()        # start planning from where it really is
                    self.cmd_claw = self.link.tel.claw
                    self.log(f"arm connected at ({self.cmd_pose.x:.0f}, {self.cmd_pose.y:.0f}, {self.cmd_pose.z:.0f})")
                    self.home_phase = 0
                    self._enter(State.HOMING)
                elif self.tel_fresh(now) and self.link.tel.estop:
                    self.estop_latched = True

            case State.HOMING:
                # phase 0: straight up to the safe height, phase 1: to HOME
                home = pose_of(self.cfg.home)
                if self.home_phase == 0:
                    here = self.cmd_pose or self.measured()
                    up = Pose(here.x, here.y, max(here.z, self.cfg.safe_z), -90.0, 0.0)
                    if not self._move(up, self.cmd_claw, self.cfg.speed_slow, now):
                        return
                    self.home_phase = 1
                elif self.home_phase == 1:
                    if self._follow(now):
                        if self._move(home, self.cmd_claw, self.cfg.speed_travel, now):
                            self.home_phase = 2
                elif self._follow(now):
                    self.log("at HOME")
                    self._enter(State.IDLE)

            case State.IDLE:
                for name, arg in reqs:
                    match name:
                        case "START":
                            self.trips_done = 0
                            self.stop_after_cycle = False
                            self.alarm = ""
                            self.log(f"START: {self.trips_target} trip(s), part at {self.object_at}")
                            self.trip_t0 = now
                            if self._move(self._station(self.object_at).above(self.cfg.approach_height),
                                          self.cfg.claw_open, self.cfg.speed_travel, now):
                                self._enter(State.APPROACH)
                            return
                        case "HOME":
                            self.home_phase = 0
                            self._enter(State.HOMING)
                            return
                        case "JOG":
                            dx, dy, dz = arg
                            goal = self.cmd_pose.moved(dx, dy, dz)
                            if not self.in_envelope(goal):          # manual override stays inside the envelope
                                self.log(f"jog refused: ({goal.x:.0f}, {goal.y:.0f}, {goal.z:.0f}) is outside the workspace")
                            elif self._move(goal, self.cmd_claw, self.cfg.speed_slow, now):
                                self._enter(State.JOG)
                            return
                        case "CLAW":
                            claw = self.cfg.claw_open if arg == "open" else self.cfg.claw_grip
                            if self._move(self.cmd_pose, claw, self.cfg.speed_slow, now):
                                self._enter(State.JOG)
                            return
                        case "SAVE":
                            values = [round(v, 1) for v in (self.cmd_pose.x, self.cmd_pose.y, self.cmd_pose.z,
                                                            self.cmd_pose.pitch, self.cmd_pose.roll)]
                            setattr(self.cfg, "station_a" if arg == "A" else "station_b", values)
                            self.cfg.save()
                            self.log(f"station {arg} taught at {values[:3]}")
                        case "CALIB":
                            if self._move(self.cmd_pose, self.cfg.claw_grip, self.cfg.speed_slow, now):
                                self._enter(State.CALIB_GRIP)
                            return
                        case "OBJECT_AT":
                            self.object_at = arg
                        case "TRIPS":
                            self.trips_target = max(1, min(100, int(arg)))

            case State.JOG:
                if self._follow(now):
                    self._enter(State.IDLE)

            case State.CALIB_GRIP:
                if self._follow(now, settle_min=self.GRIP_TIME):
                    self.cfg.claw_empty = round(self.link.tel.claw, 1)
                    self.cfg.save()
                    self.log(f"empty gripper reads {self.cfg.claw_empty:.1f} deg (saved)")
                    self._move(self.cmd_pose, self.cfg.claw_open, self.cfg.speed_slow, now)
                    self._enter(State.JOG)

            case State.APPROACH:
                if self._follow(now):
                    self.retries_left = self.cfg.grip_retries
                    if self._move(self._station(self.object_at), self.cfg.claw_open, self.cfg.speed_slow, now):
                        self._enter(State.DESCEND_PICK)

            case State.DESCEND_PICK:
                if self._follow(now, settle_min=self.SETTLE_MIN):
                    self._record_arrival(self.object_at)
                    if self._move(self.cmd_pose, self.cfg.claw_grip, self.cfg.speed_slow, now):
                        self._enter(State.GRIP)

            case State.GRIP:
                if self._follow(now, settle_min=self.GRIP_TIME):
                    self._enter(State.VERIFY_GRIP)

            case State.VERIFY_GRIP:
                # wait for a telemetry line measured AFTER the fingers stopped (no stale data)
                if self.link.tel.received <= self.t_state:
                    return
                if self.holding_part():
                    self.lost_count = 0
                    self._move(self._station(self.object_at).above(self.cfg.approach_height),
                               self.cfg.claw_grip, self.cfg.speed_slow, now)
                    self._enter(State.LIFT_PICK)
                elif self.retries_left > 0:
                    self.retries_left -= 1
                    self.grip_failures += 1
                    self.log(f"no part in the gripper (reads {self.link.tel.claw:.0f} deg) - retry")
                    self._move(self._station(self.object_at).above(30), self.cfg.claw_open, self.cfg.speed_slow, now)
                    self._enter(State.RETRY)
                else:
                    self.grip_failures += 1
                    self._fault(f"no part at station {self.object_at} (gripper closed empty)")

            case State.RETRY:
                if self._follow(now):
                    self._move(self._station(self.object_at), self.cfg.claw_open, self.cfg.speed_slow, now)
                    self._enter(State.DESCEND_PICK)

            case State.LIFT_PICK | State.TRANSFER:
                # carrying: the gripper sensor must keep seeing the part
                if self.link.tel.received != self.last_checked:          # each NEW reading once
                    self.last_checked = self.link.tel.received
                    self.lost_count = 0 if self.holding_part() else self.lost_count + 1
                if self.lost_count >= 3:                                 # 3 readings in a row = lost
                    self._fault("part dropped while carrying")
                    return
                if self._follow(now):
                    target = self._station(self._other(self.object_at))
                    if self.state is State.LIFT_PICK:
                        if self._move(target.above(self.cfg.approach_height), self.cfg.claw_grip,
                                      self.cfg.speed_travel, now):
                            self._enter(State.TRANSFER)
                    else:
                        if self._move(target.above(self.cfg.place_lift), self.cfg.claw_grip,
                                      self.cfg.speed_slow, now):
                            self._enter(State.DESCEND_PLACE)

            case State.DESCEND_PLACE:
                if self._follow(now, settle_min=self.SETTLE_MIN):
                    self._record_arrival(self._other(self.object_at))
                    if self._move(self.cmd_pose, self.cfg.claw_open, self.cfg.speed_slow, now):
                        self._enter(State.RELEASE)

            case State.RELEASE:
                if self._follow(now, settle_min=self.GRIP_TIME):
                    target = self._station(self._other(self.object_at))
                    if self._move(target.above(self.cfg.approach_height), self.cfg.claw_open,
                                  self.cfg.speed_slow, now):
                        self._enter(State.LIFT_PLACE)

            case State.LIFT_PLACE:
                if self._follow(now):
                    self._enter(State.CYCLE_END)

            case State.CYCLE_END:
                self.object_at = self._other(self.object_at)
                self.trips_done += 1
                self.cycle_times.append(now - self.trip_t0)
                self.log(f"trip {self.trips_done}/{self.trips_target} done in {now - self.trip_t0:.1f} s")
                self.trip_t0 = now
                if self.stop_after_cycle or self.trips_done >= self.trips_target:
                    self.home_phase = 0
                    self._enter(State.HOMING)
                elif self._move(self._station(self.object_at).above(self.cfg.approach_height),
                                self.cfg.claw_open, self.cfg.speed_travel, now):
                    self._enter(State.APPROACH)

            case State.ESTOP:
                # nothing moves; RESET clears the latch in the arm, then re-home safely
                if "RESET" in names and not self.reset_pending:
                    self.link.reset()
                    self.reset_pending = True
                    self.log("RESET requested")
                if self.reset_pending and self.link.tel.received > self.t_state and not self.link.tel.estop:
                    self.estop_latched = False
                    self.reset_pending = False
                    self.cmd_pose = self.measured()
                    self.home_phase = 0
                    self._enter(State.HOMING)

            case State.FAULT:
                if "RESET" in names and self.tel_fresh(now):
                    self.alarm = ""
                    self.cmd_pose = self.measured()
                    self.home_phase = 0
                    self._enter(State.HOMING)
