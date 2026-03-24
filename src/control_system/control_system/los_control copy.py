#!/usr/bin/env python3
"""
LOS / ILOS guidance controller + live tuner GUI — single file.

Usage
-----
Controller only (headless):
    ros2 run control_system los_controller

Controller + GUI in one process:
    ros2 run control_system los_controller --ros-args -p gui:=true

Or just run directly:
    python3 los_control.py
    python3 los_control.py --ros-args -p gui:=true
"""

import math
import os
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
import rclpy.parameter
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from scipy.spatial.transform import Rotation as R


# ══════════════════════════════════════════════════════════════════════════════
#  Parameter manifest  — single source of truth for controller + GUI
# ══════════════════════════════════════════════════════════════════════════════
#  (name, label, min, max, step, default, unit, group, tooltip)

PARAMS = [
    ("lkd_base",        "Base lookahead",        0.1,  3.0,  0.05,  0.6,   "m",
     "Lookahead",
     "Distance ahead the AUV steers toward.\nToo small → oscillation.  Too large → wide arcs."),

    ("lkd_k",           "Speed coefficient",     0.0,  1.0,  0.02,  0.2,   "s",
     "Lookahead",
     "Scales lookahead with speed: Δ = base + k·u.\n0 = fixed lookahead."),

    ("lkd_min",         "Lookahead floor",        0.1,  2.0,  0.05,  0.4,   "m",
     "Lookahead",
     "Absolute minimum lookahead distance."),

    ("lkd_max",         "Lookahead ceiling",      0.2,  5.0,  0.1,   1.2,   "m",
     "Lookahead",
     "Absolute maximum lookahead distance."),

    ("sigma",           "ILOS integral gain σ",   0.0,  0.2,  0.005, 0.02,  "",
     "ILOS",
     "Eliminates steady-state CTE from drag / current.\nToo large → slow oscillation."),

    ("int_ye_limit",    "Integral anti-windup",   0.1,  5.0,  0.1,   2.0,   "m·s",
     "ILOS",
     "Clamps ∫y_e to ± this value."),

    ("k_d",             "Derivative damping k_d", 0.0,  2.0,  0.05,  0.4,   "s",
     "ILOS",
     "Reduces heading command when converging fast.\nPrevents overshoot at merge.\nOnly active within damp_zone (0.4 m) of path."),

    ("v_max",           "Max cruise speed",       0.1,  5.0,  0.1,   1.5,   "m/s",
     "Speed",
     "Top forward speed on straight segments."),

    ("k_u",             "Surge P gain",           0.1,  5.0,  0.1,   1.5,   "",
     "Speed",
     "Proportional gain on forward speed error."),

    ("k_yaw",           "Yaw rate gain",          0.5, 15.0,  0.5,   5.0,   "1/s",
     "Heading",
     "Gain on yaw error → yaw rate command.\nToo low → turns late.  Too high → heading oscillation."),

    ("yaw_rate_max",    "Yaw rate clamp",         0.2,  6.0,  0.1,   2.5,   "rad/s",
     "Heading",
     "Saturation limit on angular rate.\nRaise if turns feel clipped."),

    ("k_pitch",         "Pitch rate gain",        0.1,  8.0,  0.2,   2.0,   "1/s",
     "Heading",
     "Gain on pitch error → pitch rate command."),

    ("pitch_rate_max",  "Pitch rate clamp",       0.1,  3.0,  0.1,   1.2,   "rad/s",
     "Heading",
     "Saturation limit on pitch rate."),

    ("k_depth",         "Depth P gain",           0.0,  3.0,  0.05,  0.8,   "1/s",
     "Depth",
     "Gain on depth error → pitch bias.\n0 = no depth correction (2-D mode)."),

    ("switch_dist",     "Waypoint switch dist",   0.1,  3.0,  0.05,  0.4,   "m",
     "Switching",
     "Switch to next segment when within this distance of next waypoint.\nSmaller = tighter corners."),

    ("tau",             "Prediction horizon τ",   0.0,  2.0,  0.05,  0.6,   "s",
     "Prediction",
     "Seconds ahead to project the AUV for guidance.\nIncrease if AUV turns late.\n0 = purely reactive."),
]

GROUPS = ["Lookahead", "ILOS", "Speed", "Heading", "Depth", "Switching", "Prediction"]

GROUP_COLORS = {
    "Lookahead":  "#1D9E75",
    "ILOS":       "#7F77DD",
    "Speed":      "#D85A30",
    "Heading":    "#378ADD",
    "Depth":      "#639922",
    "Switching":  "#BA7517",
    "Prediction": "#D4537E",
}

YAML_PATH = os.path.expanduser("~/.ros/los_params.yaml")


# ══════════════════════════════════════════════════════════════════════════════
#  LOS Controller node
# ══════════════════════════════════════════════════════════════════════════════

class LOSController(Node):

    def __init__(self):
        # use_sim_time MUST be passed here — before super().__init__ creates
        # the clock object. Setting it anywhere after is too late.
        super().__init__(
            'los_controller',
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL,
                    True
                )
            ]
        )

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(Path,     '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom',         self.odom_cb,       10)

        # ── Publisher ──────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control timer (20 Hz nominal) ─────────────────────────────
        # dt is re-measured each cycle from the ROS clock.
        # use_sim_time MUST be passed at launch (not declared here) so the
        # clock object created at super().__init__() already uses /clock.
        self._dt_nominal = 0.05
        self._last_time  = None
        self.timer = self.create_timer(self._dt_nominal, self.control_loop)

        # ── Parameters: seed from manifest, then declare to ROS2 ──────
        for name, _, _, _, _, default, *_ in PARAMS:
            setattr(self, name, default)

        for name, _, _, _, _, default, *_ in PARAMS:
            self.declare_parameter(name, default)

        self.add_on_set_parameters_callback(self._on_params_changed)

        # ── GUI flag ───────────────────────────────────────────────────
        self.declare_parameter('gui', False)

        # ── Internal state ─────────────────────────────────────────────
        self.waypoints   = []
        self.idx         = 0
        self.pos         = None
        self.yaw         = 0.0
        self.pitch       = 0.0
        self.u           = 0.0
        self.vel         = (0.0, 0.0, 0.0)
        self.int_ye      = 0.0
        self.ye_prev     = 0.0
        self.ye_dot      = 0.0

        self.get_logger().info("LOS Controller started")

    # ── Parameter callback ─────────────────────────────────────────────
    def _on_params_changed(self, params):
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
        return SetParametersResult(successful=True)

    # ── ROS callbacks ──────────────────────────────────────────────────
    def path_callback(self, msg):
        self.waypoints = [
            (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
            for ps in msg.poses
        ]
        self.idx     = 0
        self.int_ye  = 0.0
        self.ye_prev = 0.0
        self.ye_dot  = 0.0
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)

        q   = msg.pose.pose.orientation
        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        self.pitch = rpy[1]
        self.yaw   = rpy[2]
        self.u     = msg.twist.twist.linear.x

        vb_x = msg.twist.twist.linear.x
        vb_y = msg.twist.twist.linear.y
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.vel = (cy*vb_x - sy*vb_y, sy*vb_x + cy*vb_y,
                    msg.twist.twist.linear.z)

    # ── LOS geometry ───────────────────────────────────────────────────
    def _los_2d(self, px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        L = math.sqrt(dx*dx + dy*dy)
        if L < 1e-6:
            return ax, ay, 0.0, L, 0.0
        tx, ty = dx / L, dy / L
        s = max(0.0, min((px-ax)*tx + (py-ay)*ty, L))
        proj_x, proj_y = ax + s*tx, ay + s*ty
        ex, ey = px - proj_x, py - proj_y
        return proj_x, proj_y, s, L, -ty*ex + tx*ey

    def _lookahead(self):
        base = max(self.lkd_min, min(self.lkd_max,
                                     self.lkd_base + self.lkd_k * self.u))
        i, n = self.idx, len(self.waypoints)
        if i + 2 < n:
            A, B, C = self.waypoints[i], self.waypoints[i+1], self.waypoints[i+2]
            ab = (B[0]-A[0], B[1]-A[1], B[2]-A[2])
            bc = (C[0]-B[0], C[1]-B[1], C[2]-B[2])
            lab = math.sqrt(ab[0]**2 + ab[1]**2 + ab[2]**2)
            lbc = math.sqrt(bc[0]**2 + bc[1]**2 + bc[2]**2)
            if lab > 1e-6 and lbc > 1e-6:
                cos_a = (ab[0]*bc[0]+ab[1]*bc[1]+ab[2]*bc[2]) / (lab*lbc)
                turn  = math.acos(max(-1.0, min(1.0, cos_a)))
                base *= max(0.3, 1.0 - 0.7 * (turn / math.pi))
        return base

    def _maybe_advance(self, s, L, B):
        if self.idx >= len(self.waypoints) - 2:
            return
        bx, by, bz = B
        px, py, pz = self.pos
        near = math.sqrt((px-bx)**2+(py-by)**2+(pz-bz)**2) < self.switch_dist
        if s >= L - 1e-3 or near:
            self.idx    += 1
            self.int_ye  = 0.0
            self.ye_prev = 0.0
            self.ye_dot  = 0.0

    def _speed_demand(self):
        i, n = self.idx, len(self.waypoints)
        if i + 2 >= n:
            B = self.waypoints[n-1]
            px, py, pz = self.pos
            d = math.sqrt((px-B[0])**2+(py-B[1])**2+(pz-B[2])**2)
            return self.v_max * min(1.0, d / 2.0)
        A, B, C = self.waypoints[i], self.waypoints[i+1], self.waypoints[i+2]
        ab = (B[0]-A[0], B[1]-A[1], B[2]-A[2])
        bc = (C[0]-B[0], C[1]-B[1], C[2]-B[2])
        lab = math.sqrt(ab[0]**2+ab[1]**2+ab[2]**2)
        lbc = math.sqrt(bc[0]**2+bc[1]**2+bc[2]**2)
        if lab < 1e-6 or lbc < 1e-6:
            return self.v_max
        cos_a = (ab[0]*bc[0]+ab[1]*bc[1]+ab[2]*bc[2]) / (lab*lbc)
        turn  = math.acos(max(-1.0, min(1.0, cos_a)))
        return self.v_max * max(0.15, 1.0 - 0.85*(turn/math.pi))

    # ── Main control loop ──────────────────────────────────────────────
    def control_loop(self):
        if self.pos is None or len(self.waypoints) < 2:
            return

        # ── Measure actual dt from ROS clock (handles sim-time scaling) ─
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return                             # skip first tick — no valid dt yet
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        # Guard against clock jumps (pause/resume) or very first real tick
        if dt <= 0.0 or dt > 1.0:
            return
        if self.pos is None or len(self.waypoints) < 2:
            return
        i = self.idx
        if i >= len(self.waypoints) - 1:
            self.cmd_pub.publish(Twist())
            return

        A, B = self.waypoints[i], self.waypoints[i+1]
        px, py, pz = self.pos
        ax, ay, az = A
        bx, by, bz = B

        # Predicted position
        vx, vy, _ = self.vel
        _, _, s_xy, L_xy, y_e = self._los_2d(
            px + vx*self.tau, py + vy*self.tau, ax, ay, bx, by)
        _, _, s_actual, _, _  = self._los_2d(px, py, ax, ay, bx, by)

        # CTE derivative (low-pass filtered)
        ye_dot_raw  = (y_e - self.ye_prev) / dt
        self.ye_dot = 0.3*ye_dot_raw + 0.7*self.ye_dot
        self.ye_prev = y_e

        # ILOS integral
        self.int_ye = max(-self.int_ye_limit,
                          min(self.int_ye_limit, self.int_ye + y_e*dt))

        # Heading law: ILOS + gated derivative damping
        damp_gate = max(0.0, 1.0 - abs(y_e) / 0.4)
        numerator = y_e + self.sigma*self.int_ye + self.k_d*self.ye_dot*damp_gate
        psi_d     = math.atan2(by-ay, bx-ax) - math.atan2(numerator, self._lookahead())
        yaw_err   = math.atan2(math.sin(psi_d - self.yaw), math.cos(psi_d - self.yaw))

        # Depth / pitch
        dxy      = math.sqrt((bx-ax)**2 + (by-ay)**2)
        pitch_d  = math.atan2(-(bz-az), dxy) if dxy > 1e-6 else 0.0
        t        = max(0.0, min(1.0, s_actual/L_xy)) if L_xy > 1e-6 else 0.0
        z_err    = pz - (az + t*(bz-az))
        pitch_d -= self.k_depth * z_err
        pitch_err = math.atan2(math.sin(pitch_d - self.pitch),
                               math.cos(pitch_d - self.pitch))

        # Speed
        surge_cmd = self.k_u * (self._speed_demand() - self.u)

        # Segment switching
        self._maybe_advance(s_actual, L_xy, B)

        # Publish
        cmd = Twist()
        cmd.linear.x  = float(surge_cmd)
        cmd.angular.z = float(max(-self.yaw_rate_max,
                                  min(self.yaw_rate_max, self.k_yaw*yaw_err)))
        cmd.angular.y = float(max(-self.pitch_rate_max,
                                  min(self.pitch_rate_max, self.k_pitch*pitch_err)))
        self.cmd_pub.publish(cmd)


# ══════════════════════════════════════════════════════════════════════════════
#  Tuner GUI  (runs in the same process, tkinter owns the main thread)
# ══════════════════════════════════════════════════════════════════════════════

class TunerGUI:

    def __init__(self, node: LOSController):
        self.node = node

        self.root = tk.Tk()
        self.root.title("LOS Controller — Live Tuner")
        self.root.configure(bg="#1a1a1a")
        self.root.minsize(680, 500)

        self._vars      = {}   # name → DoubleVar
        self._val_labels= {}   # name → current-value Label
        self._after_ids = {}   # name → debounce after() id

        self._build_header()
        self._build_tabs()
        self._build_footer()

        # Seed sliders from live node values
        for name, _, _, _, _, default, *_ in PARAMS:
            val = getattr(node, name, default)
            self._vars[name].set(val)
            self._val_labels[name].config(text=f"{val:.3f}")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_status()
        self.root.mainloop()

    # ── Layout ─────────────────────────────────────────────────────────

    def _build_header(self):
        hdr = tk.Frame(self.root, bg="#111", pady=8)
        hdr.pack(fill="x")
        tk.Label(hdr, text="LOS Controller  ·  Live Parameter Tuner",
                 bg="#111", fg="#ffffff",
                 font=("TkFixedFont", 13, "bold")).pack()
        self._status_lbl = tk.Label(hdr, text="● running",
                                    bg="#111", fg="#1D9E75",
                                    font=("TkFixedFont", 10))
        self._status_lbl.pack()

    def _build_tabs(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook",      background="#1a1a1a", borderwidth=0)
        style.configure("TNotebook.Tab",  background="#2a2a2a", foreground="#999",
                        padding=[12, 5],  font=("monospace", 9))
        style.map("TNotebook.Tab",
                  background=[("selected", "#333")],
                  foreground=[("selected", "#fff")])
        style.configure("TFrame", background="#1a1a1a")

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=4)

        frames = {g: ttk.Frame(nb) for g in GROUPS}
        for g, f in frames.items():
            nb.add(f, text=f"  {g}  ")

        for row in PARAMS:
            name, label, lo, hi, step, default, unit, group, tip = row
            var = tk.DoubleVar(value=default)
            self._vars[name] = var
            self._make_row(frames[group], name, label, lo, hi,
                           step, unit, tip, var, GROUP_COLORS[group])

    def _make_row(self, parent, name, label, lo, hi, step, unit, tip, var, color):
        frame = tk.Frame(parent, bg="#1a1a1a", pady=4, padx=12)
        frame.pack(fill="x")
        frame.columnconfigure(1, weight=1)

        # Col 0: param name label — white so it's always visible
        name_text = label + (f"  [{unit}]" if unit else "")
        tk.Label(frame, text=name_text,
                 bg="#1a1a1a", fg="#ffffff",
                 font=("TkFixedFont", 10), anchor="w", width=24,
                 justify="left"
                 ).grid(row=0, column=0, sticky="w")

        # Col 1: slider
        sl = tk.Scale(frame, variable=var, from_=lo, to=hi,
                      resolution=step, orient="horizontal",
                      bg="#1a1a1a", fg=color, troughcolor="#333333",
                      activebackground=color,
                      highlightthickness=0, showvalue=False,
                      command=lambda v, n=name: self._on_slide(n, v))
        sl.grid(row=0, column=1, sticky="ew", padx=6)

        # Col 2: numeric readout
        vl = tk.Label(frame, text=f"{var.get():.3f}",
                      bg="#2a2a2a", fg=color,
                      font=("TkFixedFont", 11, "bold"),
                      width=7, anchor="e", padx=4, pady=1,
                      relief="flat")
        vl.grid(row=0, column=2, padx=2)
        self._val_labels[name] = vl

        # Col 3: reset button
        default = [p[5] for p in PARAMS if p[0] == name][0]
        tk.Button(frame, text="↺", bg="#2a2a2a", fg="#888",
                  font=("TkFixedFont", 9), relief="flat", cursor="hand2",
                  command=lambda n=name, d=default: self._reset_one(n, d)
                  ).grid(row=0, column=3, padx=2)

        # Col 4: tooltip
        tip_lbl = tk.Label(frame, text="?", bg="#333", fg="#aaa",
                           font=("TkFixedFont", 8), cursor="question_arrow",
                           padx=3, pady=1)
        tip_lbl.grid(row=0, column=4, padx=2)
        self._bind_tooltip(tip_lbl, tip)

    def _build_footer(self):
        bar = tk.Frame(self.root, bg="#111", pady=6)
        bar.pack(fill="x", side="bottom")

        bkw = dict(bg="#2a2a2a", fg="#ffffff", font=("TkFixedFont", 10),
                   relief="flat", cursor="hand2", padx=10, pady=3)
        tk.Button(bar, text="Save",
                  command=self._save_yaml, **bkw).pack(side="left",  padx=6)
        tk.Button(bar, text="Load",
                  command=self._load_yaml, **bkw).pack(side="left",  padx=2)
        tk.Button(bar, text="Reset all",
                  command=self._reset_all,  **bkw).pack(side="right", padx=6)

        self._msg_lbl = tk.Label(bar, text="", bg="#111", fg="#1D9E75",
                                 font=("TkFixedFont", 9))
        self._msg_lbl.pack(side="left", padx=10)

    # ── Slider callbacks ───────────────────────────────────────────────

    def _on_slide(self, name, value_str):
        val = float(value_str)
        self._val_labels[name].config(text=f"{val:.3f}")
        # Debounce 80 ms — avoids spamming the param callback while dragging
        if name in self._after_ids:
            self.root.after_cancel(self._after_ids[name])
        self._after_ids[name] = self.root.after(
            80, lambda n=name, v=val: self._push(n, v))

    def _push(self, name, value):
        """Write directly into the node's attribute — instant, no RPC needed."""
        setattr(self.node, name, value)
        self._msg_lbl.config(text=f"✓  {name} = {value:.3f}", fg="#1D9E75")

    def _reset_one(self, name, default):
        self._vars[name].set(default)
        self._val_labels[name].config(text=f"{default:.3f}")
        self._push(name, default)

    def _reset_all(self):
        for name, _, _, _, _, default, *_ in PARAMS:
            self._reset_one(name, default)

    # ── Save / load ────────────────────────────────────────────────────

    def _save_yaml(self):
        os.makedirs(os.path.dirname(YAML_PATH), exist_ok=True)
        lines = ["los_controller:\n", "  ros__parameters:\n"]
        for name, *_ in PARAMS:
            lines.append(f"    {name}: {self._vars[name].get():.4f}\n")
        with open(YAML_PATH, "w") as f:
            f.writelines(lines)
        self._msg_lbl.config(text=f"✓  saved → {YAML_PATH}", fg="#1D9E75")

    def _load_yaml(self):
        if not os.path.exists(YAML_PATH):
            messagebox.showwarning("Not found", f"No saved file:\n{YAML_PATH}")
            return
        import yaml
        with open(YAML_PATH) as f:
            data = yaml.safe_load(f)
        vals = data.get("los_controller", {}).get("ros__parameters", {})
        for name, value in vals.items():
            if name in self._vars:
                v = float(value)
                self._vars[name].set(v)
                self._val_labels[name].config(text=f"{v:.3f}")
                self._push(name, v)
        self._msg_lbl.config(text=f"✓  loaded {YAML_PATH}", fg="#1D9E75")

    # ── Status poll ────────────────────────────────────────────────────

    def _poll_status(self):
        wp  = len(self.node.waypoints)
        idx = self.node.idx
        spd = self.node.u
        cte = "—"
        if self.node.pos and wp >= 2 and idx < wp - 1:
            A = self.node.waypoints[idx]
            B = self.node.waypoints[idx + 1]
            px, py, _ = self.node.pos
            *_, ye = self.node._los_2d(px, py, A[0], A[1], B[0], B[1])
            cte = f"{ye:+.2f} m"
        self._status_lbl.config(
            text=f"● running   seg {idx}/{max(wp-1,0)}   "
                 f"speed {spd:.2f} m/s   CTE {cte}")
        self.root.after(200, self._poll_status)

    # ── Tooltip ────────────────────────────────────────────────────────

    def _bind_tooltip(self, widget, text):
        tip = [None]
        def show(e):
            x, y = widget.winfo_rootx() + 22, widget.winfo_rooty() + 8
            w = tk.Toplevel(widget)
            w.wm_overrideredirect(True)
            w.wm_geometry(f"+{x}+{y}")
            tk.Label(w, text=text, justify="left",
                     bg="#2a2a2a", fg="#ffffff", font=("TkFixedFont", 9),
                     relief="solid", borderwidth=1, padx=6, pady=4).pack()
            tip[0] = w
        def hide(e):
            if tip[0]:
                tip[0].destroy()
                tip[0] = None
        widget.bind("<Enter>", show)
        widget.bind("<Leave>", hide)

    def _on_close(self):
        self.root.destroy()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry points
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LOSController()

    show_gui = node.get_parameter('gui').get_parameter_value().bool_value

    if show_gui:
        # GUI needs the main thread; spin ROS in background
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        try:
            TunerGUI(node)          # blocks until window is closed
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Headless — normal spin
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()