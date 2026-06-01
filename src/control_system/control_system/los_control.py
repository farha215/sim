#!/usr/bin/env python3
"""
Vector Field Guidance (VFG) controller + live tuner GUI — single file.

Replaces LOS guidance.  Same topics in/out:
  Sub:  /planned_path  (nav_msgs/Path)
  Sub:  /odom          (nav_msgs/Odometry)
  Pub:  /cmd_vel       (geometry_msgs/Twist)

Usage
-----
Headless:
    ros2 run control_system vfg_controller

With GUI (default):
    ros2 run control_system vfg_controller
    python3 vfg_controller.py
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
#  Parameter manifest
#  (name, label, min, max, step, default, unit, group, tooltip)
# ══════════════════════════════════════════════════════════════════════════════

PARAMS = [
    ("k_e",             "Convergence gain k_e",   0.05, 2.0,  0.05,  0.25,  "1/m",
     "VFG",
     "Core VFG gain.  Desired heading = α − atan(k_e × e).\n"
     "k_e = 1/(u × T) where T = lookahead time (try 4 s).\n"
     "Too high → zigzag.  Too low → slow merge."),

    ("k_e_adapt",       "Speed-adaptive (T)",     0.0,  8.0,  0.2,   4.0,   "s",
     "VFG",
     "If > 0, overrides k_e with 1/(u × T) each cycle.\n"
     "Set to 0 to use fixed k_e above.\n"
     "Recommended: 3–5 s keeps behaviour constant across speeds."),

    ("sigma",           "ILOS integral gain σ",   0.0,  0.2,  0.005, 0.02,  "",
     "ILOS",
     "Adds integral of cross-track error to fight ocean currents.\n"
     "χ_d = α − atan(k_e × (e + σ·∫e))\n"
     "Start at 0, add slowly if drift persists.  Too large → slow oscillation."),

    ("int_ye_limit",    "Integral anti-windup",   0.1,  5.0,  0.1,   2.0,   "m·s",
     "ILOS",
     "Clamps ∫e to ± this value to prevent integrator windup."),

    ("v_max",           "Max cruise speed",       0.1,  5.0,  0.1,   1.5,   "m/s",
     "Speed",
     "Top forward speed on straight segments."),

    ("k_u",             "Surge P gain",           0.1,  5.0,  0.1,   1.5,   "",
     "Speed",
     "Proportional gain on forward speed error."),

    ("k_yaw",           "Yaw rate gain",          0.5,  15.0, 0.5,   5.0,   "1/s",
     "Heading",
     "Gain on yaw error → yaw rate command.\n"
     "Too low → turns late.  Too high → heading oscillation."),

    ("yaw_rate_max",    "Yaw rate clamp",         0.2,  6.0,  0.1,   2.5,   "rad/s",
     "Heading",
     "Saturation limit on angular rate."),

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
     "Switch to next segment when within this distance of waypoint.\n"
     "Smaller = tighter corners."),

    ("tau",             "Prediction horizon τ",   0.0,  2.0,  0.05,  0.6,   "s",
     "Prediction",
     "Seconds ahead to project AUV position before computing e.\n"
     "Compensates for heading lag.  0 = purely reactive."),
]

GROUPS = ["VFG", "ILOS", "Speed", "Heading", "Depth", "Switching", "Prediction"]

GROUP_COLORS = {
    "VFG":        "#1D9E75",
    "ILOS":       "#7F77DD",
    "Speed":      "#D85A30",
    "Heading":    "#378ADD",
    "Depth":      "#639922",
    "Switching":  "#BA7517",
    "Prediction": "#D4537E",
}

YAML_PATH = os.path.expanduser("~/.ros/vfg_params.yaml")


# ══════════════════════════════════════════════════════════════════════════════
#  VFG Controller node
# ══════════════════════════════════════════════════════════════════════════════

class VFGController(Node):

    def __init__(self):
        super().__init__(
            'vfg_controller',
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

        # ── Control timer (20 Hz) ─────────────────────────────────────
        self._dt_nominal = 0.05
        self._last_time  = None
        self.timer = self.create_timer(self._dt_nominal, self.control_loop)

        # ── Parameters ────────────────────────────────────────────────
        for name, _, _, _, _, default, *_ in PARAMS:
            setattr(self, name, default)
            self.declare_parameter(name, default)

        self.declare_parameter('gui', True)          # GUI on by default
        self.add_on_set_parameters_callback(self._on_params_changed)

        # ── State ─────────────────────────────────────────────────────
        self.waypoints = []
        self.idx       = 0
        self.pos       = None
        self.yaw       = 0.0
        self.pitch     = 0.0
        self.u         = 0.0
        self.vel       = (0.0, 0.0, 0.0)
        self.int_ye    = 0.0

        self.get_logger().info("VFG Controller started")

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
        self.idx    = 0
        self.int_ye = 0.0
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

    # ── VFG core ───────────────────────────────────────────────────────
    def _path_errors(self, px, py, ax, ay, bx, by):
        """Returns (along_track s, cross_track e, segment_length L, path_angle alpha)."""
        dx, dy = bx - ax, by - ay
        L = math.sqrt(dx*dx + dy*dy)
        if L < 1e-6:
            return 0.0, 0.0, L, 0.0
        alpha = math.atan2(dy, dx)
        ex, ey = px - ax, py - ay
        s =  math.cos(alpha)*ex + math.sin(alpha)*ey
        e = -math.sin(alpha)*ex + math.cos(alpha)*ey
        s = max(0.0, min(s, L))
        return s, e, L, alpha

    def _effective_ke(self):
        """k_e = 1/(u × T) if adaptive, else fixed k_e."""
        if self.k_e_adapt > 0.0 and self.u > 0.1:
            return 1.0 / (self.u * self.k_e_adapt)
        return self.k_e

    def _speed_demand(self):
        """Slow down before sharp waypoint turns."""
        i, n = self.idx, len(self.waypoints)
        if i + 2 >= n:
            B = self.waypoints[n - 1]
            px, py, pz = self.pos
            d = math.sqrt((px-B[0])**2 + (py-B[1])**2 + (pz-B[2])**2)
            return self.v_max * min(1.0, d / 2.0)
        A, B, C = self.waypoints[i], self.waypoints[i+1], self.waypoints[i+2]
        ab = (B[0]-A[0], B[1]-A[1], B[2]-A[2])
        bc = (C[0]-B[0], C[1]-B[1], C[2]-B[2])
        lab = math.sqrt(sum(x**2 for x in ab))
        lbc = math.sqrt(sum(x**2 for x in bc))
        if lab < 1e-6 or lbc < 1e-6:
            return self.v_max
        cos_a = sum(ab[i]*bc[i] for i in range(3)) / (lab * lbc)
        turn  = math.acos(max(-1.0, min(1.0, cos_a)))
        return self.v_max * max(0.15, 1.0 - 0.85*(turn / math.pi))

    # ── Main control loop ──────────────────────────────────────────────
    def control_loop(self):
        if self.pos is None or len(self.waypoints) < 2:
            return

        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        i = self.idx
        if i >= len(self.waypoints) - 1:
            # ── Position Hold Logic at Final Waypoint ──────────────────
            target = self.waypoints[-1]
            px, py, pz = self.pos
            
            # Errors in global frame
            dx = target[0] - px
            dy = target[1] - py
            dz = target[2] - pz
            
            # Transform to body frame (surge/lateral)
            cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
            error_surge =  dx * cos_y + dy * sin_y
            error_lateral  = -dx * sin_y + dy * cos_y
            
            # Surge/Lateral command (P-velocity with inner loop)
            # Use k_u as a general linear gain
            surge_cmd = self.k_u * (0.8 * error_surge - self.u)
            lateral_cmd  = self.k_u * (0.8 * error_lateral) # No lateral speed feedback in odom?
            
            # Depth / Heave
            # Use k_depth for heave correction
            heave_cmd = -2.0 * self.k_depth * dz
            
            # Yaw: Keep facing the direction of the last segment arrival
            # (or just hold current heading if no better info)
            cmd = Twist()
            cmd.linear.x = float(surge_cmd)
            cmd.linear.y = float(lateral_cmd)
            cmd.linear.z = float(heave_cmd)
            cmd.angular.z = 0.0 # Hold heading
            self.cmd_pub.publish(cmd)
            return

        A, B = self.waypoints[i], self.waypoints[i+1]
        ax, ay, az = A
        bx, by, bz = B

        # Predicted position (compensate heading lag)
        vx, vy, _ = self.vel
        px = self.pos[0] + vx * self.tau
        py = self.pos[1] + vy * self.tau
        pz = self.pos[2]

        s, e, L, alpha = self._path_errors(px, py, ax, ay, bx, by)
        _, _, _, _     # also get actual s for switching
        s_actual, *_  = self._path_errors(
            self.pos[0], self.pos[1], ax, ay, bx, by)

        # ── ILOS integral ──────────────────────────────────────────────
        self.int_ye = max(-self.int_ye_limit,
                          min(self.int_ye_limit, self.int_ye + e * dt))

        # ── VFG heading law ────────────────────────────────────────────
        #   χ_d = α − atan( k_e × (e + σ·∫e) )
        ke    = self._effective_ke()
        chi_d = alpha - math.atan(ke * (e + self.sigma * self.int_ye))
        yaw_err = math.atan2(math.sin(chi_d - self.yaw),
                             math.cos(chi_d - self.yaw))

        # ── Depth / pitch / heave ──────────────────────────────────────
        dxy     = math.sqrt((bx-ax)**2 + (by-ay)**2)
        pitch_d = math.atan2(-(bz-az), dxy) if dxy > 1e-6 else 0.0
        t       = max(0.0, min(1.0, s_actual / L)) if L > 1e-6 else 0.0
        z_target = az + t*(bz-az)
        z_err    = pz - z_target

        # Direct Heave Command (Active depth control)
        # Assuming Z-up: pz > z_target means too high -> heave negative
        heave_cmd = -3.0 * self.k_depth * z_err

        pitch_d -= self.k_depth * z_err
        pitch_err = math.atan2(math.sin(pitch_d - self.pitch),
                               math.cos(pitch_d - self.pitch))

        # ── Surge ──────────────────────────────────────────────────────
        surge_cmd = self.k_u * (self._speed_demand() - self.u)

        # ── Waypoint switching ─────────────────────────────────────────
        bx_, by_, bz_ = B
        px_, py_, pz_ = self.pos
        dist_to_B = math.sqrt((px_-bx_)**2+(py_-by_)**2+(pz_-bz_)**2)
        if (s_actual >= L - 1e-3 or dist_to_B < self.switch_dist) and self.idx < len(self.waypoints) - 1:
            self.idx    += 1
            self.int_ye  = 0.0
            self.get_logger().info(f"Reached waypoint {self.idx}")

        # ── Publish ────────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = float(surge_cmd)
        cmd.linear.z  = float(heave_cmd)
        cmd.angular.z = float(max(-self.yaw_rate_max,
                                  min(self.yaw_rate_max, self.k_yaw * yaw_err)))
        cmd.angular.y = float(max(-self.pitch_rate_max,
                                  min(self.pitch_rate_max, self.k_pitch * pitch_err)))
        self.cmd_pub.publish(cmd)


# ══════════════════════════════════════════════════════════════════════════════
#  Tuner GUI
# ══════════════════════════════════════════════════════════════════════════════

class TunerGUI:

    def __init__(self, node: VFGController):
        self.node = node

        self.root = tk.Tk()
        self.root.title("VFG Controller — Live Tuner")
        self.root.configure(bg="#1a1a1a")
        self.root.minsize(680, 500)

        self._vars       = {}
        self._val_labels = {}
        self._after_ids  = {}

        self._build_header()
        self._build_tabs()
        self._build_footer()

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
        tk.Label(hdr, text="VFG Controller  ·  Live Parameter Tuner",
                 bg="#111", fg="#ffffff",
                 font=("TkFixedFont", 13, "bold")).pack()
        self._status_lbl = tk.Label(hdr, text="● running",
                                    bg="#111", fg="#1D9E75",
                                    font=("TkFixedFont", 10))
        self._status_lbl.pack()

    def _build_tabs(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook",     background="#1a1a1a", borderwidth=0)
        style.configure("TNotebook.Tab", background="#2a2a2a", foreground="#999",
                        padding=[12, 5], font=("monospace", 9))
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

        name_text = label + (f"  [{unit}]" if unit else "")
        tk.Label(frame, text=name_text,
                 bg="#1a1a1a", fg="#ffffff",
                 font=("TkFixedFont", 10), anchor="w", width=26,
                 justify="left").grid(row=0, column=0, sticky="w")

        tk.Scale(frame, variable=var, from_=lo, to=hi,
                 resolution=step, orient="horizontal",
                 bg="#1a1a1a", fg=color, troughcolor="#333333",
                 activebackground=color,
                 highlightthickness=0, showvalue=False,
                 command=lambda v, n=name: self._on_slide(n, v)
                 ).grid(row=0, column=1, sticky="ew", padx=6)

        vl = tk.Label(frame, text=f"{var.get():.3f}",
                      bg="#2a2a2a", fg=color,
                      font=("TkFixedFont", 11, "bold"),
                      width=7, anchor="e", padx=4, pady=1)
        vl.grid(row=0, column=2, padx=2)
        self._val_labels[name] = vl

        default_val = [p[5] for p in PARAMS if p[0] == name][0]
        tk.Button(frame, text="↺", bg="#2a2a2a", fg="#888",
                  font=("TkFixedFont", 9), relief="flat", cursor="hand2",
                  command=lambda n=name, d=default_val: self._reset_one(n, d)
                  ).grid(row=0, column=3, padx=2)

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
        tk.Button(bar, text="Save",     command=self._save_yaml, **bkw).pack(side="left",  padx=6)
        tk.Button(bar, text="Load",     command=self._load_yaml, **bkw).pack(side="left",  padx=2)
        tk.Button(bar, text="Reset all",command=self._reset_all, **bkw).pack(side="right", padx=6)
        self._msg_lbl = tk.Label(bar, text="", bg="#111", fg="#1D9E75",
                                 font=("TkFixedFont", 9))
        self._msg_lbl.pack(side="left", padx=10)

    # ── Slider callbacks ───────────────────────────────────────────────

    def _on_slide(self, name, value_str):
        val = float(value_str)
        self._val_labels[name].config(text=f"{val:.3f}")
        if name in self._after_ids:
            self.root.after_cancel(self._after_ids[name])
        self._after_ids[name] = self.root.after(
            80, lambda n=name, v=val: self._push(n, v))

    def _push(self, name, value):
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
        lines = ["vfg_controller:\n", "  ros__parameters:\n"]
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
        vals = data.get("vfg_controller", {}).get("ros__parameters", {})
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
        ke  = self.node._effective_ke()
        cte = "—"
        if self.node.pos and wp >= 2 and idx < wp - 1:
            A = self.node.waypoints[idx]
            B = self.node.waypoints[idx + 1]
            px, py, _ = self.node.pos
            _, e, *_ = self.node._path_errors(px, py, A[0], A[1], B[0], B[1])
            cte = f"{e:+.2f} m"
        self._status_lbl.config(
            text=f"● running   seg {idx}/{max(wp-1,0)}   "
                 f"speed {spd:.2f} m/s   CTE {cte}   k_e {ke:.3f}")
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
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = VFGController()

    show_gui = node.get_parameter('gui').get_parameter_value().bool_value

    if show_gui:
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        try:
            TunerGUI(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()