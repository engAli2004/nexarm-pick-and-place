"""test_fsm.py - automatic tests of the FSM with the simulated arm (no hardware, no window).
Run:  python test_fsm.py      Each test prints PASS / FAIL; these are rows of the testing matrix."""
import os, sys, time, tempfile
import cell_fsm
from cell_fsm import CellFSM, CellConfig, State
from robot_link import SimLink

cell_fsm.CONFIG_FILE = os.path.join(tempfile.gettempdir(), "nexarm_test_cfg.json")   # don't touch the real one
cell_fsm.LOG_DIR = os.path.join(tempfile.gettempdir(), "nexarm_test_logs")
SPEEDUP_DT = 0.05

def run_until(fsm, cond, timeout, during=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        fsm.tick()
        if during: during(fsm)
        if cond(fsm): return True
        time.sleep(SPEEDUP_DT)
    return False

def new_cell():
    link = SimLink()
    cfg = CellConfig(); cfg.speed_travel, cfg.speed_slow = 400.0, 200.0   # faster for the test
    fsm = CellFSM(link, cfg)
    assert run_until(fsm, lambda f: f.state is State.IDLE, 20), "did not reach IDLE"
    return link, fsm

results = []
def check(name, ok, detail=""):
    results.append((name, ok)); print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")

# T1 boot: INIT -> HOMING -> IDLE at home
link, fsm = new_cell()
check("T1 boot goes to safe HOME then IDLE", fsm.measured().dist(cell_fsm.pose_of(fsm.cfg.home)) < 10,
      f"history={list(fsm.history)}")

# T2 calibrate empty gripper
fsm.request("CALIB"); run_until(fsm, lambda f: f.state is State.IDLE and abs(f.link.tel.claw + 50) < 3, 15)
check("T2 empty-gripper calibration", abs(fsm.cfg.claw_empty) < 3, f"claw_empty={fsm.cfg.claw_empty}")

# T3 normal job, 4 trips
fsm.request("TRIPS", 4); fsm.tick(); fsm.request("START")
ok = run_until(fsm, lambda f: f.state is State.IDLE and f.trips_done == 4, 120)
sa, sb = fsm.stats("A"), fsm.stats("B")
check("T3 4 autonomous trips A<->B, part ends at A", ok and link.part_at == "A" and fsm.object_at == "A",
      f"trips={fsm.trips_done} part={link.part_at} A={sa and round(sa['ap'],2)} B={sb and round(sb['ap'],2)}")

# T4 part missing -> retry once -> FAULT
link.remove_part(); fsm.request("START")
ok = run_until(fsm, lambda f: f.state is State.FAULT, 60)
check("T4 missing part: RETRY then FAULT", ok and "RETRY" in fsm.history and "no part" in fsm.alarm, fsm.alarm)
fsm.request("RESET"); run_until(fsm, lambda f: f.state is State.IDLE, 30)
check("T5 RESET after fault re-homes to IDLE", fsm.state is State.IDLE)

# T6 E-stop from HMI during motion, then reset
link.put_part("A"); fsm.object_at = "A"; fsm.request("START")
run_until(fsm, lambda f: f.state is State.TRANSFER, 60)
fsm.request("ESTOP"); fsm.tick(); time.sleep(0.3); p1 = link.pose[:3]; time.sleep(0.5); p2 = link.pose[:3]
moved = max(abs(a - b) for a, b in zip(p1, p2))
check("T6 HMI E-STOP stops the arm (latched in the arm too)", fsm.state is State.ESTOP and link.fw_estop and moved < 0.5,
      f"moved after stop {moved:.2f} mm")
fsm.request("RESET"); ok = run_until(fsm, lambda f: f.state is State.IDLE, 30)
check("T7 RESET after E-stop: lift first, then HOME", ok and fsm.history.count("HOMING") >= 1)

# T8 KEY2 on the arm (hardware E-stop) and KEY1 start
link.press_key2(); run_until(fsm, lambda f: f.state is State.ESTOP, 5)
check("T8 KEY2 on the arm -> ESTOP", fsm.state is State.ESTOP)
fsm.request("RESET"); run_until(fsm, lambda f: f.state is State.IDLE, 30)
# the part is still in the gripper after T6 -> open it at home, put part back at A
fsm.request("CLAW", "open"); run_until(fsm, lambda f: f.state is State.IDLE and f.link.tel.claw < -45, 10)
link.part_at = None; link.put_part("A"); fsm.object_at = "A"; fsm.trips_target = 1
link.press_key1(); ok = run_until(fsm, lambda f: f.state is State.IDLE and f.trips_done == 1, 60)
check("T9 KEY1 on the arm starts a trip", ok and link.part_at == "B")

# T10 part dropped while carrying
fsm.trips_target = 1; fsm.request("START")
def drop(f):
    if f.state is State.TRANSFER and link.part_at == "held": link.part_at = None
ok = run_until(fsm, lambda f: f.state is State.FAULT, 60, during=drop)
check("T10 part dropped while carrying -> FAULT", ok and "dropped" in fsm.alarm, fsm.alarm)
fsm.request("RESET"); run_until(fsm, lambda f: f.state is State.IDLE, 30)

# T11 jam: the arm stops following -> position fault
link.put_part("B"); fsm.object_at = "B"; fsm.request("START")
run_until(fsm, lambda f: f.state is State.TRANSFER, 60)
orig = link._send
def jammed(line):
    if line.startswith("@mv"): return
    orig(line)
link._send = jammed
ok = run_until(fsm, lambda f: f.state is State.FAULT, 30)
check("T11 jam / collision: position not reached -> FAULT", ok and "position not reached" in fsm.alarm, fsm.alarm)
link._send = orig

# T12 envelope: jog outside workspace refused
fsm.request("RESET"); run_until(fsm, lambda f: f.state is State.IDLE, 30)
before = fsm.cmd_pose; fsm.request("JOG", (0, 0, 500)); fsm.tick()
check("T12 jog outside the envelope refused, arm does not move", fsm.state is State.IDLE and fsm.cmd_pose == before
      and "refused" in fsm.log_lines[-1], fsm.log_lines[-1][9:])

# T13 lost link
link.tel_period = 0; ok = run_until(fsm, lambda f: f.state is State.FAULT, 3)
check("T13 telemetry lost -> FAULT", ok and "fresh" in fsm.alarm, fsm.alarm)

print(f"\n{sum(o for _, o in results)}/{len(results)} tests passed")
sys.exit(0 if all(o for _, o in results) else 1)
