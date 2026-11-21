"""
robot_link.py - communication between the laptop (Python) and the NexArm (USB serial).

The arm runs the custom C++ "motion driver" (firmware/system_task_handle.cpp). The PC sends one short trajectory
segment every 100 ms and the arm sends back what it MEASURES (servo encoders -> Hiwonder
forward kinematics) about 10 times per second:

    PC  -> arm   @mv x y z pitch roll claw ms     @tel 100   @estop   @reset   @ping
    arm -> PC    TEL,ms,x,y,z,pitch,roll,claw,claw_raw,estop,feedback_age_ms
                 EVT,KEY1    EVT,ESTOP,KEY2    EVT,RESET    PONG,...    ERR,...

Two classes with the same interface:
    SerialLink  the real arm (needs pyserial:  python -m pip install pyserial)
    SimLink     a simulated arm + object, to test the FSM and the HMI without hardware
"""
from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass


@dataclass
class Telemetry:
    """One TEL line: the arm's MEASURED state."""
    t_ms: int = 0             # arm clock (ms)
    x: float = 0.0            # gripper tip, mm, robot base frame (x forward, y left, z up)
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0        # deg (-90 = pointing down)
    roll: float = 0.0         # deg
    claw: float = 0.0         # gripper angle, deg (-50 open .. 0 closed)
    claw_raw: int = 0         # gripper servo raw position (0..4095)
    estop: bool = False       # E-stop latched inside the arm (KEY2 or @estop)
    fb_age_ms: int = 9999     # age of the arm's servo feedback (data validation)
    received: float = 0.0     # PC time.time() when this line arrived (0 = never)


def parse_tel(line: str) -> Telemetry | None:
    """'TEL,...' -> Telemetry, or None if the line is damaged (validation)."""
    parts = line.split(",")
    if len(parts) != 11 or parts[0] != "TEL":
        return None
    try:
        return Telemetry(int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]),
                         float(parts[5]), float(parts[6]), float(parts[7]), int(parts[8]),
                         parts[9] == "1", int(parts[10]), time.time())
    except ValueError:
        return None


class _LinkBase:
    """What the FSM sees: latest telemetry + a queue of events + send commands."""

    def __init__(self) -> None:
        self.tel = Telemetry()
        self.events: queue.Queue[str] = queue.Queue()      # "KEY1", "ESTOP,KEY2", "RESET", ...
        self.messages: queue.Queue[str] = queue.Queue()    # other text from the arm (for the log)
        self.pong = False

    # ---- commands (same for real and simulated arm) ----
    def ping(self) -> None:
        self._send("@ping")

    def telemetry(self, period_ms: int) -> None:
        self._send(f"@tel {period_ms}")

    def move(self, x: float, y: float, z: float, pitch: float, roll: float, claw: float, ms: int) -> None:
        self._send(f"@mv {x:.1f} {y:.1f} {z:.1f} {pitch:.1f} {roll:.1f} {claw:.1f} {ms:d}")

    def estop(self) -> None:
        self._send("@estop")

    def reset(self) -> None:
        self._send("@reset")

    def close(self) -> None:
        pass

    def _send(self, line: str) -> None:
        raise NotImplementedError

    def _handle_line(self, line: str) -> None:
        """Sorts one received line: telemetry, event, pong, or text."""
        if line.startswith("TEL,"):
            t = parse_tel(line)
            if t is not None:
                self.tel = t                       # replaced as a whole: never half-updated
        elif line.startswith("EVT,"):
            self.events.put(line[4:])
        elif line.startswith("PONG"):
            self.pong = True
        elif line:
            self.messages.put(line)


class SerialLink(_LinkBase):
    """The real arm on a COM port. A background thread reads every line."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        super().__init__()
        import serial                              # pyserial
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.05
        self.ser.dtr = False                       # do NOT reset the ESP32 when opening
        self.ser.rts = False
        self.ser.open()
        self._lock = threading.Lock()
        self._alive = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _send(self, line: str) -> None:
        with self._lock:
            self.ser.write((line + "\n").encode("ascii"))

    def _reader(self) -> None:
        buf = b""
        while self._alive:
            try:
                buf += self.ser.read(256)
            except Exception as e:                 # cable pulled out
                self.messages.put(f"serial error: {e}")
                time.sleep(0.5)
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle_line(raw.decode("ascii", "replace").strip())

    def close(self) -> None:
        self._alive = False
        try:
            self.telemetry(0)                      # arm back to its own console mode
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass


class SimLink(_LinkBase):
    """Simulated arm: follows the segments with a small lag and a fixed offset like the real
    one (about 3 mm short in x), a gripper that stops on the object, and an object that can
    be removed from the HMI to test the 'part missing' branch."""

    OBJECT_HALF_WIDTH = 16.5                       # the real part is 33 mm wide

    def __init__(self, a_xy=(201.0, 2.0), b_xy=(301.0, 2.0)) -> None:
        super().__init__()
        self.pose = [200.0, 0.0, 194.0, -2.0, 0.0, -50.0]       # measured x y z pitch roll claw
        self.goal = list(self.pose)
        self.seg_t0, self.seg_ms, self.seg_from = time.time(), 1, list(self.pose)
        self.fw_estop = False
        self.tel_period = 0
        self.stations = {"A": a_xy, "B": b_xy}
        self.part_at: str | None = "A"             # "A", "B", "held" or None (removed)
        self._alive = True
        threading.Thread(target=self._run, daemon=True).start()

    # --- buttons of the simulated arm, used by the HMI's SIM panel ---
    def press_key1(self) -> None:
        self.events.put("KEY1")

    def press_key2(self) -> None:
        self.fw_estop = True
        self.events.put("ESTOP,KEY2")

    def remove_part(self) -> None:
        if self.part_at in ("A", "B"):
            self.part_at = None

    def put_part(self, station: str) -> None:
        if self.part_at != "held":
            self.part_at = station

    def _send(self, line: str) -> None:
        p = line[1:].split()
        match p[0]:
            case "ping":
                self._handle_line("PONG,sim,1")
            case "tel":
                self.tel_period = int(p[1])
            case "mv":
                if self.fw_estop:
                    self._handle_line("ERR,estop")
                    return
                self.seg_from = list(self.pose)
                self.goal = [float(v) for v in p[1:7]]
                self.goal[0] -= 2.8 * self.goal[0] / 300.0      # sag: a bit short, more when far
                self.seg_ms, self.seg_t0 = max(20, int(float(p[7]))), time.time()
            case "estop":
                self.fw_estop = True
                self._handle_line("EVT,ESTOP,PC")
            case "reset":
                self.fw_estop = False
                self._handle_line("EVT,RESET")

    def _run(self) -> None:
        last_tel = 0.0
        while self._alive:
            now = time.time()
            k = min(1.0, (now - self.seg_t0) * 1000.0 / self.seg_ms)
            if not self.fw_estop:
                for i in range(6):
                    self.pose[i] = self.seg_from[i] + (self.goal[i] - self.seg_from[i]) * k
            self._gripper_physics()
            if self.tel_period and now - last_tel >= self.tel_period / 1000.0:
                last_tel = now
                x, y, z, pi, ro, cl = self.pose
                self._handle_line(f"TEL,{int(now * 1000) % 100000000},{round(x)},{round(y)},{round(z)},"
                                  f"{pi:.1f},{ro:.1f},{cl:.1f},{int(2048 + cl * 11)},"
                                  f"{1 if self.fw_estop else 0},{30}")
            time.sleep(0.01)

    def _gripper_physics(self) -> None:
        x, y, z, _, _, claw = self.pose
        if self.part_at in ("A", "B"):
            sx, sy = self.stations[self.part_at]
            near = math.hypot(x - sx, y - sy) < 10 and z < 35
            if near and claw > -22:               # fingers touch the part -> it is gripped
                self.part_at = "held"
        if self.part_at == "held":
            self.pose[5] = max(self.pose[5], -50.0)
            self.pose[5] = min(self.pose[5], -22.0)               # the part stops the fingers
            if claw < -30:                                        # opened -> released
                for name, (sx, sy) in self.stations.items():
                    if math.hypot(x - sx, y - sy) < 15:
                        self.part_at = name
                        break
                else:
                    self.part_at = None
