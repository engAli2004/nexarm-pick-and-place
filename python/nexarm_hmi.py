"""
nexarm_hmi.py - operator screen (HMI) of the NexArm pick-and-place cell.  Python 3.10+, Tkinter.

    python nexarm_hmi.py --port COM12      real arm (arm program "Nex_Arm" uploaded)
    python nexarm_hmi.py --sim             simulated arm, no hardware needed

What the operator sees (updated 10 times per second):
  * the E-STOP status banner and a big E-STOP button (keyboard: SPACE or ESC)
  * the robot's MEASURED position X, Y, Z in mm from the base centre, pitch and gripper angle
  * the FSM state and the last transitions
  * inputs/outputs: link, arm E-stop (KEY2), KEY1, part-in-gripper sensor, gripper, motion
  * a top view of the cell: stations A/B, the part, the gripper and its trail, the height bar
  * results: trips, cycle time, accuracy AP and repeatability RP per station (ISO 9283)
  * controls: START / STOP / HOME / RESET, number of trips, where the part is,
    and MANUAL override (jog X/Y/Z, open/close, teach A/B, calibrate gripper) - only in IDLE

The FSM (cell_fsm.py) is ticked every 50 ms from Tkinter's own timer: the window never freezes
and the inputs are read on every tick, also while the arm is moving.
"""
from __future__ import annotations

import argparse
import os
import time
import tkinter as tk
from collections import deque

from cell_fsm import CellConfig, CellFSM, State, MOTION_STATES, pose_of
from robot_link import SerialLink, SimLink

TICK_MS = 50
BG, PANEL, INK, MUTED, LINE = "#15171a", "#1f2226", "#e8e6e1", "#9a988f", "#3a3d42"
GREEN, AMBER, RED, BLUE, GREY = "#2fbf71", "#f0a020", "#e5484d", "#3b8de0", "#5a5d63"
MONO = ("Consolas", 20, "bold")
UI = ("Segoe UI", 10)
UI_B = ("Segoe UI", 10, "bold")
TITLE = ("Segoe UI", 11, "bold")


def state_color(s: State) -> str:
    if s is State.ESTOP:
        return RED
    if s is State.FAULT:
        return AMBER
    if s is State.IDLE:
        return GREEN
    if s in (State.INIT, State.HOMING):
        return "#c9b458"
    return BLUE


class HMI:
    def __init__(self, root: tk.Tk, fsm: CellFSM, link, sim: bool) -> None:
        self.root, self.fsm, self.link, self.sim = root, fsm, link, sim
        self.trail: deque[tuple[float, float]] = deque(maxlen=80)
        self.tick_count = 0
        root.title("NexArm Pick & Place Cell - HMI" + ("  [SIMULATION]" if sim else ""))
        root.configure(bg=BG)
        root.bind("<space>", lambda e: self.fsm.request("ESTOP"))
        root.bind("<Escape>", lambda e: self.fsm.request("ESTOP"))
        root.protocol("WM_DELETE_WINDOW", self.quit)

        self._build_header()
        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True, padx=10, pady=(0, 6))
        left = tk.Frame(body, bg=BG, width=330)
        mid = tk.Frame(body, bg=BG)
        right = tk.Frame(body, bg=BG, width=360)
        left.pack(side="left", fill="y")
        mid.pack(side="left", fill="both", expand=True, padx=10)
        right.pack(side="left", fill="y")
        self._build_position(left)
        self._build_state(left)
        self._build_io(left)
        self._build_view(mid)
        self._build_results(mid)
        self._build_controls(right)
        self._build_manual(right)
        if sim:
            self._build_sim(right)
        self._build_log()
        root.after(TICK_MS, self._loop)

    # ------------------------------------------------------------------ layout helpers
    def _panel(self, parent: tk.Widget, title: str) -> tk.Frame:
        outer = tk.Frame(parent, bg=PANEL, highlightbackground=LINE, highlightthickness=1)
        outer.pack(fill="x", pady=4)
        tk.Label(outer, text=title, bg=PANEL, fg=MUTED, font=UI_B, anchor="w").pack(fill="x", padx=8, pady=(6, 2))
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        return inner

    def _button(self, parent: tk.Widget, text: str, cmd, color: str = "#2c3036", width: int = 9,
                big: bool = False) -> tk.Button:
        return tk.Button(parent, text=text, command=cmd, bg=color, fg=INK, activebackground=color,
                         activeforeground=INK, relief="flat", width=width,
                         font=("Segoe UI", 13, "bold") if big else UI_B, disabledforeground=GREY,
                         cursor="hand2", pady=6 if big else 3)

    def _led(self, parent: tk.Widget, row: int, text: str) -> tuple[tk.Canvas, tk.Label]:
        c = tk.Canvas(parent, width=16, height=16, bg=PANEL, highlightthickness=0)
        c.grid(row=row, column=0, sticky="w", pady=2)
        c.create_oval(2, 2, 14, 14, fill=GREY, outline="", tags="dot")
        tk.Label(parent, text=text, bg=PANEL, fg=INK, font=UI, anchor="w").grid(row=row, column=1, sticky="w", padx=6)
        val = tk.Label(parent, text="", bg=PANEL, fg=MUTED, font=UI, anchor="e")
        val.grid(row=row, column=2, sticky="e")
        parent.grid_columnconfigure(2, weight=1)
        return c, val

    @staticmethod
    def _set_led(led: tuple[tk.Canvas, tk.Label], color: str, text: str) -> None:
        led[0].itemconfigure("dot", fill=color)
        led[1].configure(text=text)

    # ------------------------------------------------------------------ build
    def _build_header(self) -> None:
        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill="x", padx=10, pady=8)
        tk.Label(bar, text="NexArm Pick & Place Cell", bg=BG, fg=INK, font=("Segoe UI", 16, "bold")).pack(side="left")
        tk.Label(bar, text="  A <-> B  accuracy & repeatability", bg=BG, fg=MUTED, font=UI).pack(side="left")
        self.banner = tk.Label(bar, text="", font=("Segoe UI", 14, "bold"), width=34, pady=6)
        self.banner.pack(side="right")
        self.mode_lbl = tk.Label(bar, text="SIMULATION" if self.sim else "REAL ARM", bg=BG,
                                 fg=AMBER if self.sim else MUTED, font=UI_B)
        self.mode_lbl.pack(side="right", padx=12)

    def _build_position(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "ROBOT POSITION (measured, mm from base centre)")
        self.pos_lbl = {}
        for i, axis in enumerate(("X", "Y", "Z")):
            tk.Label(p, text=axis, bg=PANEL, fg=MUTED, font=MONO).grid(row=0, column=2 * i, sticky="e")
            lbl = tk.Label(p, text="---", bg=PANEL, fg=INK, font=MONO, width=5, anchor="e")
            lbl.grid(row=0, column=2 * i + 1, sticky="w", padx=(2, 8))
            self.pos_lbl[axis] = lbl
        self.pose_extra = tk.Label(p, text="", bg=PANEL, fg=MUTED, font=UI, anchor="w", justify="left")
        self.pose_extra.grid(row=1, column=0, columnspan=6, sticky="w", pady=(4, 0))

    def _build_state(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "FSM STATE")
        self.state_lbl = tk.Label(p, text="", bg=PANEL, fg=INK, font=("Segoe UI", 18, "bold"), anchor="w")
        self.state_lbl.pack(fill="x")
        self.hist_lbl = tk.Label(p, text="", bg=PANEL, fg=MUTED, font=UI, anchor="w", justify="left",
                                 wraplength=300)
        self.hist_lbl.pack(fill="x")
        self.alarm_lbl = tk.Label(p, text="", bg=PANEL, fg=AMBER, font=UI_B, anchor="w", justify="left",
                                  wraplength=300)
        self.alarm_lbl.pack(fill="x", pady=(4, 0))

    def _build_io(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "INPUTS / OUTPUTS")
        self.led_link = self._led(p, 0, "Link to arm (telemetry)")
        self.led_estop = self._led(p, 1, "E-stop (arm KEY2 / HMI)")
        self.led_key1 = self._led(p, 2, "KEY1 start button")
        self.led_part = self._led(p, 3, "Part in gripper (sensor)")
        self.led_claw = self._led(p, 4, "Gripper command")
        self.led_motion = self._led(p, 5, "Arm in motion")

    def _build_view(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "CELL - TOP VIEW  (robot base at the bottom, x forward)")
        row = tk.Frame(p, bg=PANEL)
        row.pack()
        self.W, self.H, self.S = 520, 400, 1.0          # canvas px, 1 px per mm
        self.view = tk.Canvas(row, width=self.W, height=self.H, bg="#101214", highlightthickness=0)
        self.view.pack(side="left")
        self.zbar = tk.Canvas(row, width=56, height=self.H, bg="#101214", highlightthickness=0)
        self.zbar.pack(side="left", padx=(6, 0))
        self._draw_static()

    def _xy(self, x: float, y: float) -> tuple[float, float]:
        """Robot mm -> canvas px (robot's left = screen left when standing behind the robot)."""
        return self.W / 2 - y * self.S, self.H - 15 - x * self.S

    def _draw_static(self) -> None:
        v, cfg = self.view, self.fsm.cfg
        v.delete("static")
        for d in range(-250, 251, 50):                          # 50 mm grid
            x0, y0 = self._xy(0, d)
            x1, y1 = self._xy(400, d)
            v.create_line(x0, y0, x1, y1, fill="#1d2024", tags="static")
        for d in range(0, 401, 50):
            x0, y0 = self._xy(d, -260)
            x1, y1 = self._xy(d, 260)
            v.create_line(x0, y0, x1, y1, fill="#1d2024", tags="static")
            v.create_text(6, y0, text=f"{d}", fill=GREY, anchor="w", font=("Segoe UI", 7), tags="static")
        a, b = self._xy(cfg.x_min, cfg.y_max), self._xy(cfg.x_max, -cfg.y_max)
        v.create_rectangle(a[0], b[1], b[0], a[1], outline="#4a4630", dash=(4, 3), tags="static")
        v.create_text(b[0] - 4, b[1] + 4, text="safe envelope", fill="#8a8050", anchor="ne",
                      font=("Segoe UI", 8), tags="static")
        bx, by = self._xy(0, 0)
        v.create_oval(bx - 45, by - 45, bx + 45, by + 45, outline=GREY, fill="#202327", tags="static")
        v.create_text(bx, by - 18, text="BASE", fill=MUTED, font=("Segoe UI", 8, "bold"), tags="static")
        for name, values in (("A", cfg.station_a), ("B", cfg.station_b)):
            sx, sy = self._xy(values[0], values[1])
            v.create_oval(sx - 22, sy - 22, sx + 22, sy + 22, outline="#5a6470", dash=(3, 2), tags="static")
            v.create_text(sx + 30, sy, text=f"{name}  ({values[0]:.0f}, {values[1]:.0f})", fill=INK,
                          anchor="w", font=UI_B, tags="static")
        z = self.zbar
        z.delete("all")
        z.create_text(28, 10, text="Z mm", fill=MUTED, font=("Segoe UI", 8))
        for h in range(0, 251, 50):
            y = self._zy(h)
            z.create_line(20, y, 36, y, fill=GREY)
            z.create_text(52, y, text=str(h), fill=GREY, font=("Segoe UI", 7), anchor="e")
        z.create_line(28, self._zy(0), 28, self._zy(250), fill=LINE, width=3)

    def _zy(self, h: float) -> float:
        return self.H - 15 - h * (self.H - 40) / 250.0

    def _build_results(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "RESULTS (this session)")
        self.res_lbl = tk.Label(p, text="", bg=PANEL, fg=INK, font=("Consolas", 11), anchor="w", justify="left")
        self.res_lbl.pack(fill="x")

    def _build_controls(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "SAFETY")
        self.btn_estop = self._button(p, "E-STOP", lambda: self.fsm.request("ESTOP"), RED, 14, big=True)
        self.btn_estop.pack(side="left", padx=(0, 6))
        self.btn_reset = self._button(p, "RESET", lambda: self.fsm.request("RESET"), "#3a3f46", 9, big=True)
        self.btn_reset.pack(side="left")

        p = self._panel(parent, "AUTOMATIC")
        top = tk.Frame(p, bg=PANEL)
        top.pack(fill="x")
        self.btn_start = self._button(top, "START", self._start, "#1f6f46", 8, big=True)
        self.btn_stop = self._button(top, "STOP", lambda: self.fsm.request("STOP"), "#6b5a1f", 8, big=True)
        self.btn_home = self._button(top, "HOME", lambda: self.fsm.request("HOME"), "#2c3036", 8, big=True)
        for b in (self.btn_start, self.btn_stop, self.btn_home):
            b.pack(side="left", padx=(0, 4))
        opt = tk.Frame(p, bg=PANEL)
        opt.pack(fill="x", pady=(8, 0))
        tk.Label(opt, text="Trips", bg=PANEL, fg=INK, font=UI).pack(side="left")
        self.trips_var = tk.StringVar(value=str(self.fsm.trips_target))
        tk.Spinbox(opt, from_=1, to=100, width=4, textvariable=self.trips_var, font=UI,
                   bg="#2c3036", fg=INK, buttonbackground="#2c3036").pack(side="left", padx=(4, 14))
        tk.Label(opt, text="Part is at", bg=PANEL, fg=INK, font=UI).pack(side="left")
        self.part_var = tk.StringVar(value=self.fsm.object_at)
        for s in ("A", "B"):
            tk.Radiobutton(opt, text=s, value=s, variable=self.part_var, font=UI_B, bg=PANEL, fg=INK,
                           selectcolor="#2c3036", activebackground=PANEL,
                           command=lambda s=s: self.fsm.request("OBJECT_AT", s)).pack(side="left")
        tk.Label(p, text="KEY1 on the arm = START     KEY2 on the arm = E-STOP", bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 8)).pack(anchor="w", pady=(6, 0))

    def _build_manual(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "MANUAL OVERRIDE / TEACHING  (only in IDLE)")
        steps = tk.Frame(p, bg=PANEL)
        steps.pack(fill="x")
        tk.Label(steps, text="Step mm", bg=PANEL, fg=INK, font=UI).pack(side="left")
        self.step_var = tk.IntVar(value=10)
        for s in (1, 5, 10, 20, 50):
            tk.Radiobutton(steps, text=str(s), value=s, variable=self.step_var, font=UI, bg=PANEL, fg=INK,
                           selectcolor="#2c3036", activebackground=PANEL).pack(side="left")
        grid = tk.Frame(p, bg=PANEL)
        grid.pack(pady=(6, 0))
        self.manual_buttons = []
        jog = [("X+", 1, 0, 0, 0, 1), ("X-", -1, 0, 0, 2, 1), ("Y+  left", 0, 1, 0, 1, 0),
               ("Y-  right", 0, -1, 0, 1, 2), ("Z+", 0, 0, 1, 0, 3), ("Z-", 0, 0, -1, 2, 3)]
        for text, dx, dy, dz, r, c in jog:
            b = self._button(grid, text, lambda dx=dx, dy=dy, dz=dz: self._jog(dx, dy, dz), width=8)
            b.grid(row=r, column=c, padx=2, pady=2)
            self.manual_buttons.append(b)
        row = tk.Frame(p, bg=PANEL)
        row.pack(pady=(6, 0))
        for text, cmd in (("OPEN", lambda: self.fsm.request("CLAW", "open")),
                          ("CLOSE", lambda: self.fsm.request("CLAW", "close")),
                          ("SAVE A", lambda: self._save("A")), ("SAVE B", lambda: self._save("B"))):
            b = self._button(row, text, cmd, width=7)
            b.pack(side="left", padx=2)
            self.manual_buttons.append(b)
        b = self._button(p, "CALIBRATE GRIPPER (close empty)", lambda: self.fsm.request("CALIB"), width=30)
        b.pack(pady=(6, 0))
        self.manual_buttons.append(b)

    def _build_sim(self, parent: tk.Widget) -> None:
        p = self._panel(parent, "SIMULATION ONLY")
        rows = (tk.Frame(p, bg=PANEL), tk.Frame(p, bg=PANEL))
        for r in rows:
            r.pack(pady=1)
        for k, (text, cmd) in enumerate((("KEY1", self.link.press_key1), ("KEY2", self.link.press_key2),
                                          ("Remove part", self.link.remove_part),
                                          ("Part at A", lambda: self.link.put_part("A")),
                                          ("Part at B", lambda: self.link.put_part("B")))):
            self._button(rows[0 if k < 2 else 1], text, cmd, "#3a2f4a", width=9).pack(side="left", padx=2)

    def _build_log(self) -> None:
        f = tk.Frame(self.root, bg=PANEL, highlightbackground=LINE, highlightthickness=1)
        f.pack(fill="x", padx=10, pady=(0, 10))
        tk.Label(f, text="EVENT LOG", bg=PANEL, fg=MUTED, font=UI_B, anchor="w").pack(fill="x", padx=8, pady=(4, 0))
        self.log_txt = tk.Text(f, height=6, bg="#101214", fg=INK, font=("Consolas", 9), relief="flat",
                               state="disabled")
        self.log_txt.pack(fill="x", padx=8, pady=(2, 8))
        self.log_seen = 0

    # ------------------------------------------------------------------ actions
    def _start(self) -> None:
        try:
            self.fsm.request("TRIPS", int(self.trips_var.get()))
        except ValueError:
            pass
        self.fsm.request("OBJECT_AT", self.part_var.get())
        self.fsm.request("START")

    def _jog(self, dx: int, dy: int, dz: int) -> None:
        s = self.step_var.get()
        self.fsm.request("JOG", (dx * s, dy * s, dz * s))

    def _save(self, station: str) -> None:
        self.fsm.request("SAVE", station)
        self.root.after(200, self._draw_static)          # show the new station position

    def quit(self) -> None:
        self.link.close()
        self.root.destroy()

    # ------------------------------------------------------------------ periodic
    def _loop(self) -> None:
        self.fsm.tick()                                   # the FSM: inputs + current state
        self.tick_count += 1
        if self.tick_count % 2 == 0:                      # screen: 10 times per second
            self._refresh()
        self.root.after(TICK_MS, self._loop)

    def _refresh(self) -> None:
        fsm, tel, st = self.fsm, self.link.tel, self.fsm.state
        now = time.time()
        fresh = fsm.tel_fresh(now)

        # banner: E-stop status, always visible
        if st is State.ESTOP or fsm.estop_latched:
            on = (self.tick_count // 10) % 2 == 0
            self.banner.configure(text="E-STOP ACTIVE  -  press RESET", bg=RED if on else "#7a1f22", fg="white")
        elif st is State.FAULT:
            self.banner.configure(text="FAULT  -  check, then RESET", bg=AMBER, fg="black")
        elif not fresh:
            self.banner.configure(text="CONNECTING TO ARM...", bg="#c9b458", fg="black")
        else:
            self.banner.configure(text="SYSTEM OK  -  E-stop released", bg=GREEN, fg="black")

        # position
        for axis, val in (("X", tel.x), ("Y", tel.y), ("Z", tel.z)):
            self.pos_lbl[axis].configure(text=f"{val:5.0f}" if fresh else "---")
        cmd = fsm.cmd_pose
        cmd_txt = f"command ({cmd.x:.0f}, {cmd.y:.0f}, {cmd.z:.0f})" if cmd else "command -"
        self.pose_extra.configure(text=f"pitch {tel.pitch:.0f} deg   roll {tel.roll:.0f} deg   gripper {tel.claw:.0f} deg\n"
                                       f"{cmd_txt}   data age {tel.fb_age_ms} ms")

        # state
        self.state_lbl.configure(text=st.name, fg=state_color(st))
        self.hist_lbl.configure(text=" > ".join(list(fsm.history)[-7:]))
        self.alarm_lbl.configure(text=fsm.alarm)

        # inputs / outputs
        self._set_led(self.led_link, GREEN if fresh else RED, "OK" if fresh else "NO DATA")
        self._set_led(self.led_estop, RED if (tel.estop or fsm.estop_latched) else GREEN,
                      "ACTIVE" if (tel.estop or fsm.estop_latched) else "released")
        pressed = now - fsm.key1_time < 1.0                     # lit for 1 s after a press
        self._set_led(self.led_key1, BLUE if pressed else GREY, "pressed" if pressed else "")
        holding = fsm.holding_part() and tel.claw > fsm.cfg.claw_open + 10
        self._set_led(self.led_part, GREEN if holding else GREY,
                      f"{'YES' if holding else 'no'}  ({tel.claw:.0f} / empty {fsm.cfg.claw_empty:.0f})")
        closed = fsm.cmd_claw > (fsm.cfg.claw_open + fsm.cfg.claw_grip) / 2
        self._set_led(self.led_claw, BLUE if closed else GREY, "CLOSED" if closed else "OPEN")
        moving = st in MOTION_STATES and fsm.traj is not None
        self._set_led(self.led_motion, BLUE if moving else GREY, "moving" if moving else "still")

        # buttons allowed in this state
        idle = st is State.IDLE
        for b in self.manual_buttons + [self.btn_start, self.btn_home]:
            b.configure(state="normal" if idle else "disabled")
        self.btn_stop.configure(state="normal" if st in MOTION_STATES else "disabled")
        self.btn_reset.configure(state="normal" if st in (State.ESTOP, State.FAULT) else "disabled")
        self.part_var.set(fsm.object_at)

        self._draw_dynamic(fresh, holding)
        self._update_results()
        self._update_log()

    def _draw_dynamic(self, fresh: bool, holding: bool) -> None:
        v, tel, fsm = self.view, self.link.tel, self.fsm
        v.delete("dyn")
        if fresh:
            self.trail.append(self._xy(tel.x, tel.y))
        if len(self.trail) > 1:
            v.create_line(*[c for p in self.trail for c in p], fill="#2b5f93", width=2, tags="dyn")
        # the part (33 mm square): in the gripper or on its station
        half = 16.5 * self.S
        if holding and fsm.state in (State.LIFT_PICK, State.TRANSFER, State.DESCEND_PLACE):
            px, py = self._xy(tel.x, tel.y)
        else:
            s = pose_of(fsm.cfg.station_a if fsm.object_at == "A" else fsm.cfg.station_b)
            px, py = self._xy(s.x, s.y)
        v.create_rectangle(px - half, py - half, px + half, py + half, fill="#d9d6cf", outline="", tags="dyn")
        v.create_text(px, py, text="part", fill="#333", font=("Segoe UI", 7), tags="dyn")
        # target of the current move
        if fsm.traj is not None:
            gx, gy = self._xy(fsm.traj.goal.x, fsm.traj.goal.y)
            v.create_line(gx - 8, gy, gx + 8, gy, fill=AMBER, width=2, tags="dyn")
            v.create_line(gx, gy - 8, gx, gy + 8, fill=AMBER, width=2, tags="dyn")
        # the gripper (measured)
        if fresh:
            gx, gy = self._xy(tel.x, tel.y)
            color = state_color(fsm.state)
            v.create_oval(gx - 7, gy - 7, gx + 7, gy + 7, fill=color, outline="white", width=2, tags="dyn")
            self.zbar.delete("dyn")
            zy = self._zy(max(0.0, min(250.0, tel.z)))
            self.zbar.create_rectangle(18, zy - 4, 38, zy + 4, fill=color, outline="white", tags="dyn")

    def _update_results(self) -> None:
        fsm = self.fsm
        lines = [f"Trips {fsm.trips_done}/{fsm.trips_target}   grip failures {fsm.grip_failures}"]
        if fsm.cycle_times:
            ct = fsm.cycle_times
            lines[0] += f"   cycle {sum(ct) / len(ct):.1f} s"
        lines.append("Station  arrivals   accuracy AP   repeatability RP   mean offset x / y")
        for s in ("A", "B"):
            r = fsm.stats(s)
            if r:
                lines.append(f"   {s}   {r['n']:3d}   {r['ap']:6.2f} mm  {r['rp']:6.2f} mm  {r['dx']:+.1f}/{r['dy']:+.1f}")
            else:
                lines.append(f"   {s}         -")
        if fsm.csv_path:
            lines.append("log: logs/" + os.path.basename(fsm.csv_path))
        self.res_lbl.configure(text="\n".join(lines))

    def _update_log(self) -> None:
        lines = list(self.fsm.log_lines)
        total = len(lines)
        if total == self.log_seen and total < self.fsm.log_lines.maxlen:
            return
        self.log_txt.configure(state="normal")
        self.log_txt.delete("1.0", "end")
        self.log_txt.insert("end", "\n".join(lines[-60:]))
        self.log_txt.see("end")
        self.log_txt.configure(state="disabled")
        self.log_seen = total


def main() -> None:
    ap = argparse.ArgumentParser(description="NexArm pick & place cell - FSM + HMI")
    ap.add_argument("--port", default="COM12", help="COM port of the arm (Device Manager -> Ports)")
    ap.add_argument("--sim", action="store_true", help="simulated arm, no hardware")
    a = ap.parse_args()
    cfg = CellConfig.load()
    if a.sim:
        link = SimLink((cfg.station_a[0], cfg.station_a[1]), (cfg.station_b[0], cfg.station_b[1]))
    else:
        link = SerialLink(a.port)
    fsm = CellFSM(link, cfg)
    root = tk.Tk()
    HMI(root, fsm, link, a.sim)
    root.mainloop()


if __name__ == "__main__":
    main()
