#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
import threading


class SixDOFPID(Node):

    def __init__(self):
        super().__init__('six_dof_pid')

        self.dt = 0.05

        self.declare_parameter("enable_tuner", True)

        # PID gains
        self.declare_parameter("Kp", [2.0, 2.0, 3.0, 1.0, 1.0, 6.0])
        self.declare_parameter("Ki", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("Kd", [0.4, 0.4, 0.6, 0.1, 0.1, 0.4])

        # manual targets (used when not following a path)
        self.declare_parameter("target_x",     0.0)
        self.declare_parameter("target_y",     0.0)
        self.declare_parameter("target_z",     0.0)
        self.declare_parameter("target_roll",  0.0)
        self.declare_parameter("target_pitch", 0.0)
        self.declare_parameter("target_yaw",   0.0)

        # speed limits
        self.declare_parameter("max_linear_speed",  20.0)
        self.declare_parameter("max_angular_speed", 20.0)

        # waypoint following thresholds
        self.declare_parameter("waypoint_pos_thresh", 0.15)   # metres
        self.declare_parameter("waypoint_yaw_thresh", 0.08)   # rad
        self.declare_parameter("lookahead_dist",      0.50)   # metres
        self.declare_parameter("waypoint_step",        10)    # keep every Nth waypoint from planner
        self.declare_parameter("min_linear_speed",   2.0)   # speed floor on sharpest curves
        self.declare_parameter("max_curve_speed",   20.0)   # speed ceiling on straights

        # adaptive Kd params
        self.declare_parameter("adaptive_kd",       True)   # enable/disable adaptive Kd
        self.declare_parameter("adaptive_kd_alpha", 0.01)   # adaptation rate — higher = faster but noisier
        self.declare_parameter("kd_min", [0.05, 0.05, 0.05, 0.02, 0.02, 0.02])  # Kd lower bounds
        self.declare_parameter("kd_max", [3.0,  3.0,  3.0,  2.0,  2.0,  2.0])   # Kd upper bounds

        self.current = np.zeros(6)
        self.target  = np.zeros(6)

        self.integral       = np.zeros(6)
        self.prev_error     = np.zeros(6)
        self.integral_limit = 2.0
        self.deadband       = np.array([0.03, 0.03, 0.03, 0.02, 0.02, 0.02])

        # adaptive Kd state — starts at declared Kd values
        self.Kd_adaptive = np.array([0.4, 0.4, 0.6, 0.1, 0.1, 0.4])

        # path following state
        self.waypoints      = []   # list of (x, y, z, yaw)
        self.waypoint_idx   = 0
        self.following_path = False

        self.create_subscription(Odometry, "/odom",         self.odom_callback, 10)
        self.create_subscription(Path,     "/planned_path", self.path_callback,  10)
        self.cmd_pub = self.create_publisher(Twist,   "/cmd_vel", 10)
        self.rpy_pub = self.create_publisher(Vector3, "/rpy",     10)
        self.timer   = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("6DOF PID controller started")

        if self.get_parameter("enable_tuner").value:
            t = threading.Thread(target=self._launch_tuner, daemon=True)
            t.start()


    # ─────────────────────────────── helpers ────────────────────────────────

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    # ──────────────────────────── odom callback ─────────────────────────────

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.current[0] = p.x
        self.current[1] = p.y
        self.current[2] = p.z

        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        sinr_cosp = 2 * (qw*qx + qy*qz)
        cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp  = 2 * (qw*qy - qz*qx)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (qw*qz + qx*qy)
        cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current[3] = roll
        self.current[4] = pitch
        self.current[5] = yaw

        rpy = Vector3()
        rpy.x, rpy.y, rpy.z = roll, pitch, yaw
        self.rpy_pub.publish(rpy)


    # ──────────────────────────── path callback ─────────────────────────────

    def path_callback(self, msg):
        self.waypoints = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            siny_cosp = 2 * (q.w*q.z + q.x*q.y)
            cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.waypoints.append((p.x, p.y, p.z, yaw))

        if self.waypoints:
            step = self.get_parameter("waypoint_step").value
            self.waypoints = self.waypoints[::step]

            # ── compute curvature-based speed limit per waypoint ─────────────
            # curvature = angle change between consecutive segments
            # straight → low curvature → fast
            # tight curve → high curvature → slow
            v_max = self.get_parameter("max_curve_speed").value
            v_min = self.get_parameter("min_linear_speed").value
            n     = len(self.waypoints)
            self.waypoint_speeds = []

            for i in range(n):
                if i == 0 or i == n - 1:
                    # endpoints: start slow, stop at end
                    self.waypoint_speeds.append(v_min)
                else:
                    # compute angle change at this waypoint using neighbours
                    x0, y0 = self.waypoints[i-1][0], self.waypoints[i-1][1]
                    x1, y1 = self.waypoints[i  ][0], self.waypoints[i  ][1]
                    x2, y2 = self.waypoints[i+1][0], self.waypoints[i+1][1]

                    a1 = math.atan2(y1 - y0, x1 - x0)
                    a2 = math.atan2(y2 - y1, x2 - x1)
                    curvature = abs(math.atan2(math.sin(a2 - a1),
                                               math.cos(a2 - a1)))

                    # curvature in [0, π] → speed in [v_max, v_min]
                    t     = min(curvature / math.pi, 1.0)
                    speed = v_max - t * (v_max - v_min)
                    self.waypoint_speeds.append(speed)

            self.waypoint_idx   = 0
            self.following_path = True
            self.integral       = np.zeros(6)
            self.prev_error     = np.zeros(6)
            self.get_logger().info(
                f"New path received — {len(self.waypoints)} waypoints (step={step})"
            )


    # ───────────────────────── read ros2 parameters ─────────────────────────

    def read_parameters(self):
        self.Kp = np.array(self.get_parameter("Kp").value)
        self.Ki = np.array(self.get_parameter("Ki").value)
        self.Kd = np.array(self.get_parameter("Kd").value)

        self.target[0] = self.get_parameter("target_x").value
        self.target[1] = self.get_parameter("target_y").value
        self.target[2] = self.get_parameter("target_z").value
        self.target[3] = self.get_parameter("target_roll").value
        self.target[4] = self.get_parameter("target_pitch").value
        self.target[5] = self.get_parameter("target_yaw").value

        self.max_linear  = self.get_parameter("max_linear_speed").value
        self.max_angular = self.get_parameter("max_angular_speed").value


    # ──────────────────────────── control loop ──────────────────────────────

    def control_loop(self):
        self.read_parameters()

        if self.following_path and self.waypoints:
            pos_thresh = self.get_parameter("waypoint_pos_thresh").value
            yaw_thresh = self.get_parameter("waypoint_yaw_thresh").value
            lookahead  = self.get_parameter("lookahead_dist").value

            # step 1 — advance base index past waypoints already within pos_thresh
            while self.waypoint_idx < len(self.waypoints) - 1:
                wx, wy, wz, _ = self.waypoints[self.waypoint_idx]
                d = math.sqrt((self.current[0]-wx)**2 +
                              (self.current[1]-wy)**2 +
                              (self.current[2]-wz)**2)

                # compute yaw to this waypoint
                yaw_to_wp = math.atan2(wy - self.current[1], wx - self.current[0])
                yaw_diff  = abs(self.wrap_angle(yaw_to_wp - self.current[5]))

                # skip waypoint if:
                #   (a) within pos_thresh — arrived normally, OR
                #   (b) it's behind the robot (>90°) — inertia carried us past it
                if d < pos_thresh or yaw_diff > math.pi / 2:
                    self.waypoint_idx += 1
                    self.integral   = np.zeros(6)
                    self.prev_error = np.zeros(6)
                    self.get_logger().info(
                        f"Waypoint {self.waypoint_idx}/{len(self.waypoints)} reached"
                    )
                else:
                    break

            # step 2 — check if final waypoint is reached
            fx, fy, fz, fyaw = self.waypoints[-1]
            final_dist    = math.sqrt((self.current[0]-fx)**2 +
                                      (self.current[1]-fy)**2 +
                                      (self.current[2]-fz)**2)
            final_yaw_err = abs(self.wrap_angle(fyaw - self.current[5]))

            if final_dist < pos_thresh and final_yaw_err < yaw_thresh:
                self.following_path = False
                self.get_logger().info("Path complete — holding final position")

            else:
                # step 3 — lookahead: target first waypoint >= lookahead_dist away
                lookahead_idx = self.waypoint_idx   # fallback if nothing far enough
                for j in range(self.waypoint_idx, len(self.waypoints)):
                    wx, wy, wz, _ = self.waypoints[j]
                    d = math.sqrt((self.current[0]-wx)**2 +
                                  (self.current[1]-wy)**2 +
                                  (self.current[2]-wz)**2)
                    if d >= lookahead:
                        lookahead_idx = j
                        break
                    lookahead_idx = j

                wx, wy, wz, _ = self.waypoints[lookahead_idx]

                # yaw computed from robot toward the lookahead point —
                # NOT the stored tangent yaw which is only valid at that waypoint
                wyaw = math.atan2(wy - self.current[1], wx - self.current[0])

                self.target[0] = wx
                self.target[1] = wy
                self.target[2] = wz
                self.target[5] = wyaw
                # roll and pitch always stay at 0

        error    = self.target - self.current
        error[5] = self.wrap_angle(error[5])

        # no lateral thruster — zero Y to prevent integral wind-up
        error[1]         = 0.0
        self.integral[1] = 0.0

        yaw = self.current[5]
        ex, ey = error[0], error[1]   # ey is always 0
        error[0] =  math.cos(yaw)*ex + math.sin(yaw)*ey
        error[1] = -math.sin(yaw)*ex + math.cos(yaw)*ey

        # derivative before deadband to avoid kick
        delta_error    = error - self.prev_error
        delta_error[5] = self.wrap_angle(delta_error[5])
        derivative     = delta_error / self.dt

        # suppress both error and derivative inside deadband
        in_deadband             = np.abs(error) < self.deadband
        error[in_deadband]      = 0.0
        derivative[in_deadband] = 0.0

        self.integral += error * self.dt
        self.integral  = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        # ── adaptive Kd (inspired by paper eq.32) ───────────────────────────
        # when error and derivative have same sign → overshooting → increase Kd
        # when they have opposite sign → converging → decrease Kd
        if self.get_parameter("adaptive_kd").value:
            alpha  = self.get_parameter("adaptive_kd_alpha").value
            kd_min = np.array(self.get_parameter("kd_min").value)
            kd_max = np.array(self.get_parameter("kd_max").value)

            # adaptation law: Kd increases when error * derivative > 0 (overshooting)
            #                 Kd decreases when error * derivative < 0 (converging)
            self.Kd_adaptive += -alpha * np.sign(error * derivative) * error * derivative * self.dt
            self.Kd_adaptive  = np.clip(self.Kd_adaptive, kd_min, kd_max)

            effective_Kd = self.Kd_adaptive
        else:
            effective_Kd = self.Kd

        tau = self.Kp * error + self.Ki * self.integral + effective_Kd * derivative

        self.prev_error = error

        # use curvature-based speed limit when following a path
        if self.following_path and hasattr(self, "waypoint_speeds") and self.waypoint_speeds:
            idx = min(self.waypoint_idx, len(self.waypoint_speeds) - 1)
            linear_limit = self.waypoint_speeds[idx]
        else:
            linear_limit = self.max_linear

        tau[0:3] = np.clip(tau[0:3], -linear_limit, linear_limit)
        tau[3:6] = np.clip(tau[3:6], -self.max_angular, self.max_angular)

        cmd = Twist()
        cmd.linear.x  = float(tau[0])
        cmd.linear.y  = float(tau[1])
        cmd.linear.z  = float(tau[2])
        cmd.angular.x = float(tau[3])
        cmd.angular.y = float(tau[4])
        cmd.angular.z = float(tau[5])
        self.cmd_pub.publish(cmd)


    # ──────────────────────────────── GUI ───────────────────────────────────

    def _launch_tuner(self):
        import tkinter as tk
        from tkinter import ttk

        AXES   = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        COLORS = ["#e74c3c", "#27ae60", "#2980b9", "#8e44ad", "#e67e22", "#16a085"]

        root = tk.Tk()
        root.title("6DOF PID Tuner")
        root.configure(bg="#1a1a2e")
        root.resizable(True, True)

        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TNotebook",     background="#1a1a2e", borderwidth=0)
        style.configure("TNotebook.Tab", background="#16213e", foreground="#aaaaaa",
                                         padding=[12, 6], font=("Courier", 10))
        style.map("TNotebook.Tab",
                  background=[("selected", "#0f3460")],
                  foreground=[("selected", "#ffffff")])
        style.configure("TFrame", background="#1a1a2e")
        style.configure("TLabel", background="#1a1a2e", foreground="#cccccc",
                                  font=("Courier", 10))

        def set_param(name, value):
            param = rclpy.parameter.Parameter(
                name, rclpy.parameter.Parameter.Type.DOUBLE, value)
            self.set_parameters([param])

        def set_array_param(name, arr):
            param = rclpy.parameter.Parameter(
                name, rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                [float(v) for v in arr])
            self.set_parameters([param])

        Kp_vals  = list(self.get_parameter("Kp").value)
        Ki_vals  = list(self.get_parameter("Ki").value)
        Kd_vals  = list(self.get_parameter("Kd").value)
        tgt_vals = [
            self.get_parameter("target_x").value,
            self.get_parameter("target_y").value,
            self.get_parameter("target_z").value,
            self.get_parameter("target_roll").value,
            self.get_parameter("target_pitch").value,
            self.get_parameter("target_yaw").value,
        ]
        tgt_names = ["target_x","target_y","target_z",
                     "target_roll","target_pitch","target_yaw"]

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)

        # ── per-axis tabs ──
        for i, axis in enumerate(AXES):
            frame = ttk.Frame(notebook, padding=16)
            notebook.add(frame, text=axis)

            color = COLORS[i]
            tk.Label(frame, text=f"{axis} axis", font=("Courier", 13, "bold"),
                     fg=color, bg="#1a1a2e").grid(
                         row=0, column=0, columnspan=3, sticky="w", pady=(0, 14))

            is_angular = i >= 3
            kp_max  = 20.0  if is_angular else 10.0
            kd_max  = 5.0   if is_angular else 3.0
            tgt_min = -3.14 if is_angular else -5.0
            tgt_max =  3.14 if is_angular else  5.0

            gains = [
                ("Kp", Kp_vals, kp_max, 0.1),
                ("Ki", Ki_vals, 2.0,    0.01),
                ("Kd", Kd_vals, kd_max, 0.01),
            ]

            for row, (label, arr, g_max, step) in enumerate(gains, start=1):
                var = tk.DoubleVar(value=arr[i])

                ttk.Label(frame, text=label, width=4).grid(
                    row=row, column=0, sticky="w", pady=4)

                tk.Scale(frame, variable=var, from_=0.0, to=g_max,
                         resolution=step, orient="horizontal", length=320,
                         bg="#16213e", fg=color, highlightthickness=0,
                         troughcolor="#0f3460", activebackground=color,
                         showvalue=False, bd=0).grid(row=row, column=1, padx=8, pady=4)

                val_lbl = tk.Label(frame, text=f"{arr[i]:.3f}", width=7,
                                   font=("Courier", 10), fg=color, bg="#1a1a2e")
                val_lbl.grid(row=row, column=2, sticky="w")

                def make_gain_cb(arr_ref, idx, lbl_name, vl, v=var):
                    def cb(*_):
                        val = round(v.get(), 3)
                        arr_ref[idx] = val
                        vl.config(text=f"{val:.3f}")
                        set_array_param(lbl_name, arr_ref)
                    return cb

                var.trace_add("write", make_gain_cb(
                    Kp_vals if label=="Kp" else Ki_vals if label=="Ki" else Kd_vals,
                    i,
                    "Kp"    if label=="Kp" else "Ki"    if label=="Ki" else "Kd",
                    val_lbl, var
                ))

            # separator
            tk.Frame(frame, bg="#0f3460", height=1).grid(
                row=4, column=0, columnspan=3, sticky="ew", pady=12)

            # target slider
            ttk.Label(frame, text="Target").grid(row=5, column=0, sticky="w", pady=4)
            tgt_var = tk.DoubleVar(value=tgt_vals[i])

            tk.Scale(frame, variable=tgt_var, from_=tgt_min, to=tgt_max,
                     resolution=0.01, orient="horizontal", length=320,
                     bg="#16213e", fg="#f0c040", highlightthickness=0,
                     troughcolor="#0f3460", activebackground="#f0c040",
                     showvalue=False, bd=0).grid(row=5, column=1, padx=8, pady=4)

            tgt_val_lbl = tk.Label(frame, text=f"{tgt_vals[i]:.3f}", width=7,
                                   font=("Courier", 10), fg="#f0c040", bg="#1a1a2e")
            tgt_val_lbl.grid(row=5, column=2, sticky="w")

            def make_tgt_cb(idx, name, vl, v=tgt_var):
                def cb(*_):
                    val = round(v.get(), 3)
                    vl.config(text=f"{val:.3f}")
                    set_param(name, val)
                return cb

            tgt_var.trace_add("write", make_tgt_cb(i, tgt_names[i], tgt_val_lbl))

            # live current readback
            cur_lbl = tk.Label(frame, text="current: —", font=("Courier", 9),
                               fg="#555577", bg="#1a1a2e")
            cur_lbl.grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))

            def update_current(lbl=cur_lbl, idx=i):
                try:
                    val  = self.current[idx]
                    unit = "rad" if idx >= 3 else "m"
                    lbl.config(text=f"current: {val:.4f} {unit}")
                except Exception:
                    pass
                root.after(100, update_current, lbl, idx)

            root.after(100, update_current, cur_lbl, i)

        # ── limits tab ──
        lim_frame = ttk.Frame(notebook, padding=16)
        notebook.add(lim_frame, text="Limits")

        tk.Label(lim_frame, text="Speed limits", font=("Courier", 13, "bold"),
                 fg="#ffffff", bg="#1a1a2e").grid(
                     row=0, column=0, columnspan=3, sticky="w", pady=(0, 14))

        limit_params = [
            ("max linear",       "max_linear_speed",    self.get_parameter("max_linear_speed").value,    0,    100,  0.5),
            ("max angular",      "max_angular_speed",   self.get_parameter("max_angular_speed").value,   0,    100,  0.5),
            ("pos thresh (m)",   "waypoint_pos_thresh", self.get_parameter("waypoint_pos_thresh").value, 0.01, 1.0,  0.01),
            ("yaw thresh (rad)", "waypoint_yaw_thresh", self.get_parameter("waypoint_yaw_thresh").value, 0.01, 0.5,  0.01),
            ("lookahead (m)",    "lookahead_dist",      self.get_parameter("lookahead_dist").value,      0.1,  5.0,  0.05),
            ("Kd alpha",         "adaptive_kd_alpha",   self.get_parameter("adaptive_kd_alpha").value,   0.0,  0.1,  0.001),
        ]

        # adaptive Kd toggle
        adapt_var = tk.BooleanVar(value=self.get_parameter("adaptive_kd").value)
        tk.Checkbutton(lim_frame, text="Enable adaptive Kd",
                       variable=adapt_var, bg="#1a1a2e", fg="#16a085",
                       selectcolor="#0f3460", activebackground="#1a1a2e",
                       font=("Courier", 10),
                       command=lambda: set_param("adaptive_kd", float(adapt_var.get()))
                       ).grid(row=len(limit_params)+1, column=0, columnspan=3,
                              sticky="w", pady=(12, 4))

        # live adaptive Kd readback
        adapt_lbl = tk.Label(lim_frame, text="Kd live: —",
                             font=("Courier", 9), fg="#16a085", bg="#1a1a2e")
        adapt_lbl.grid(row=len(limit_params)+2, column=0, columnspan=3, sticky="w")

        def update_adapt_kd(lbl=adapt_lbl):
            try:
                vals = [f"{v:.3f}" for v in self.Kd_adaptive]
                lbl.config(text="Kd live: [" + ", ".join(vals) + "]")
            except Exception:
                pass
            root.after(300, update_adapt_kd, lbl)

        root.after(300, update_adapt_kd, adapt_lbl)

        for row, (lbl_text, param_name, init_val, lo, hi, step) in enumerate(limit_params, start=1):
            v = tk.DoubleVar(value=init_val)
            ttk.Label(lim_frame, text=lbl_text, width=18).grid(
                row=row, column=0, sticky="w", pady=6)

            v_lbl = tk.Label(lim_frame, text=f"{init_val:.2f}", width=7,
                             font=("Courier", 10), fg="#ffffff", bg="#1a1a2e")

            def make_lim_cb(name, vl, var=v):
                def cb(*_):
                    val = round(var.get(), 3)
                    vl.config(text=f"{val:.2f}")
                    set_param(name, val)
                return cb

            tk.Scale(lim_frame, variable=v, from_=lo, to=hi,
                     resolution=step, orient="horizontal", length=320,
                     bg="#16213e", fg="#ffffff", highlightthickness=0,
                     troughcolor="#0f3460", activebackground="#ffffff",
                     showvalue=False, bd=0).grid(row=row, column=1, padx=8, pady=6)
            v_lbl.grid(row=row, column=2, sticky="w")
            v.trace_add("write", make_lim_cb(param_name, v_lbl))

        # ── path status indicator ──
        tk.Frame(lim_frame, bg="#0f3460", height=1).grid(
            row=len(limit_params)+1, column=0, columnspan=3, sticky="ew", pady=12)

        status_lbl = tk.Label(lim_frame, text="path: idle",
                              font=("Courier", 10), fg="#555577", bg="#1a1a2e")
        status_lbl.grid(row=len(limit_params)+2, column=0, columnspan=3, sticky="w")

        wp_lbl = tk.Label(lim_frame, text="", font=("Courier", 9),
                          fg="#555577", bg="#1a1a2e")
        wp_lbl.grid(row=len(limit_params)+3, column=0, columnspan=3, sticky="w")

        def update_status():
            if self.following_path:
                status_lbl.config(text="path: FOLLOWING", fg="#27ae60")
                wp_lbl.config(
                    text=f"waypoint {self.waypoint_idx + 1} / {len(self.waypoints)}")
            else:
                status_lbl.config(text="path: idle", fg="#555577")
                wp_lbl.config(text="")
            root.after(200, update_status)

        root.after(200, update_status)
        root.mainloop()


# ─────────────────────────────── main ───────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SixDOFPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()