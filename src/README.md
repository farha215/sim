# `src/` — Hydrogen Workspace Reference

Comprehensive map of the ROS 2 workspace for the **Hydrogen** AUV (MIT-B team, RoboSub 2026). Built for Humble + Ignition Fortress.

This document covers what each package does, how the nodes wire together, where
the tuning knobs live, and any gotchas worth remembering.

---

## 1. Packages at a glance

| Package | Lang | Purpose |
|---|---|---|
| `hydrogen` | Python + xacro/SDF | Robot description, Gazebo worlds, sim ↔ ROS bridge, sensor & teleop nodes, YOLO + depth fusion. |
| `control_system` | Python | Thruster allocation, global planner (A* + spline), LOS / VFG path-following controllers. |
| `prequalification_bt` | C++ | BehaviorTree.CPP mission executor for the pre-qualification course. |
| `custom_interfaces` | msg/srv | Service definitions shared across packages. Currently just `PlanPath.srv`. |

---

## 2. End-to-end runtime architecture

```
                  ┌─────────────────────────┐
                  │  prequalification_bt    │  (mission state-machine)
                  │  ticks BT @ 10 Hz       │
                  └────┬──────────────┬─────┘
                       │              │
                  /cmd_vel        plan_path (srv)
                       │              │
                       ▼              ▼
   ┌──────────────────────────┐   ┌────────────────────────┐
   │ control_system           │   │ control_system         │
   │  allocation_matrix       │◄──┤  los_control / 6dof_pid│◄── /planned_path
   │  6×5 wrench → 5 thrusts  │   │  (path follower)       │    /odom
   └────────┬─────────────────┘   └────────────────────────┘
            │
   /hydrogen/{front,left_1,left_2,right_1,right_2,back}_propeller/cmd_thrust
            │
            ▼
   ros_gz_bridge  ──►  Ignition Gazebo (Thruster + Hydrodynamics + Buoyancy plugins)
            ▲
            │  /imu   /altimeter   /camera/...  /joint_states
            │
   robot_state_publisher  ◄── robot.xacro  (main_frame, propellers, cameras)
            │
            ▼ /tf
   RTAB-Map (rgbd_sync → rgbd_odometry → rtabmap_slam)
            │
            ▼ /odom  /cloud_map
   global_planner_node ──► /planned_path
            ▲
            │ /goal_pose, plan_path srv
            │
   yolo_node ──► /detections_2d                          (front camera + best.pt)
   data_distance_node ──► /detections_3d                 (RGB+depth fusion, no NN)
```

The BT consumes the same `/cmd_vel` interface as the LOS controller. Both
ultimately feed `allocation_matrix`, which is the single owner of per-thruster
topics.

---

## 3. `hydrogen` package

Animesh's package — robot description, worlds, bridge, and the bulk of the
ROS-side helper nodes.

### 3.1 Model (`model/`)

`robot.xacro` is the top-level URDF; it includes:

- `main_frame.xacro` — `hydrogen_frame` link with a box collision (currently
  `0.45 × 0.35 × 0.22 m`, origin `z=0.12 m`). The box collision is also the
  primary **buoyancy displacer** (Gazebo computes buoyancy from collision
  geometry). Mass + inertia are tuned for ~0.5 % positive net buoyancy
  including accessory link masses — see `memory/auv_buoyancy_tuning.md` for the
  recipe.
- `propellers.xacro` — 5 T200 thrusters at fixed positions on `hydrogen_frame`.
  Joint names: `back_propeller_joint`, `{left,right}_propeller_{1,2}_joint`.
  Each 0.1 kg, scale `0.0001 × 0.0001 × 0.0001` on `propellervertical.stl`.
- `cameras.xacro` — `zed_camera` macro with `*_optical_frame` child (converts
  robot-frame → optical-frame for ROS conventions). Robot spawns two: front
  (`xyz="0.31 0 0.1"`) and down (`xyz="0.17 0 -0.04" rpy="0 1.5708 0"`).
- `robot.gazebo` — Gazebo plugins:
  - `Thruster` plugin per propeller (`thrust_coefficient=0.004422`,
    `propeller_diameter=0.2`, `fluid_density=1000`).
  - `Imu`, `Sensors` (ogre2 renderer), `JointStatePublisher`.
  - `Hydrodynamics` plugin on `base_link` with Fossen-style added-mass /
    damping coefficients (xU, yV, zW, …). Tuned on 2026-XX (see commit
    `f22cea1`).
  - Custom `libauv_absolute_depth.so` plugin publishing `/altimeter`
    (Float64 reference-plane depth). The plugin path must be on
    `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` — see top-level `ReadMe.md`.

### 3.2 Worlds (`worlds/`)

Default world is `buoyant_pool.sdf`:

- `physics`: ODE, 1 ms step, RTF=1.
- `Buoyancy` plugin uses graded fluid: `default_density=1000`, change to
  density `1` above `z=1` (i.e., water surface is at z=1, air above).
- Pre-included models: `startgate` (z≈0.22), `pool` (z=-1.5). Pathmarkers,
  bruvs, poles, buoy, octagon, trash, preq_task are present but commented out
  — uncomment the `<include>` block to add them.
- Subfolder per object (`startgate/`, `preq_task/`, etc.) — each has its own
  `.sdf` and meshes. `setup.py` installs them via `package_files()`.

### 3.3 Launch files (`launch/`)

| File | Purpose |
|---|---|
| `model.launch.py` | **Main sim entry point.** Starts Gazebo with `buoyant_pool.sdf`, spawns Hydrogen (default `x=-4.5, y=-22.0, z=2.5`), `robot_state_publisher`, `parameter_bridge` (from `parameters/bridge_params.yaml`), and `allocation_matrix`. CLI args: `x y z R P Y`. |
| `AUV.launch.py` | **Hardware bring-up.** Runs `controller_node` (cascaded PI+LQR attitude), `imu_node` (BNO055 I²C), `dshot_node` (UART → ESCs), `thruster_teleop_GP` (gamepad). |
| `RTAB_map.launch.py` | RTAB-Map SLAM pipeline: `rgbd_sync` → `rgbd_odometry` → `rtabmap_slam` → `rtabmap_viz`, plus `global_planner` from `control_system`. Subscribes to `/camera/RGB_image_raw/front`, `/camera/depth_image_raw/front`, `/camera_info_front`. |

### 3.4 Bridge (`parameters/bridge_params.yaml`)

`parameter_bridge` config that wires Gazebo ↔ ROS:

- `/clock`, `/imu`, `/altimeter`, `/joint_states` — Gazebo → ROS.
- 5 × `/hydrogen/<propeller>/cmd_thrust` (`Float64`) — ROS → Gazebo
  (mapped to `/model/Hydrogen/joint/<name>/cmd_pos`).
- Per camera: `/camera/RGB_image_raw/{front,down}`,
  `/camera/depth_image_raw/{front,down}`, `/camera_info_{front,down}`,
  `/point_cloud_{front,down}`.

### 3.5 Python nodes (`hydrogen/`)

All registered in `setup.py` as console scripts.

| Entry point | Module | Role |
|---|---|---|
| `controller_node` | `controller_node.py` | Cascaded **outer-PI + inner-LQR** attitude controller (200 Hz). Subscribes `/imu`. Publishes 3 thrust commands on `new_thrust_{front,left,right}` (consumed by `thruster_teleop`). Complementary-filter orientation (α=0.98). |
| `imu_node` | `imu_publisher.py` | BNO055 IMU driver over I²C. Publishes `sensor_msgs/Imu` on `/imu` @ 10 Hz, `frame_id="imu_link"`. |
| `dshot_node` | `thruster_Dshot_publisher.py` | UART bridge to thruster ESCs over `/dev/ttyAMA0` @ 115200, 500 Hz. Maps thrust ∈ [-100, +100] → DShot 11-bit throttle (CCW = 1001-2000, CW = 0-1000). Adds 4-bit CRC. |
| `thruster_teleop` | `thruster_teleop.py` | Keyboard teleop that **blends** controller outputs (`new_thrust_*` on 3 channels) with manual offsets across all 5 thrusters. Arrow keys = ascend/descend/yaw, WSAD/IJKL = surge/pitch/roll. |
| `thruster_teleop_GP` | `thruster_teleop_GP.py` | Pygame-based **gamepad → `/cmd_vel`** publisher. Axes 0–3 = sway, surge, yaw, heave. Scaled by `linear_scale=20`, `angular_scale=20`. |
| `yolo_node` | `yolo_node.py` | Ultralytics YOLO inference on `/camera/RGB_image_raw/front` using `share/hydrogen/best.pt`. Publishes `vision_msgs/Detection2DArray` on `/detections_2d` + annotated image on `/yolo/detection_image`. |
| `data_distance_node` | `data_distance_node.py` | **HSV + depth-fusion 3D detector** (no NN). Synchronises depth + RGB, masks orange in HSV, dilates, finds contours, classifies bbox by aspect ratio (`preq_pole` if h/w ≥ 2 else `preq_gate`), extracts z from depth ROI (foreground clustering within 0.5 m of nearest pixel), back-projects via camera intrinsics. Publishes `Detection3DArray` on `/detections_3d`. Gate is filtered if z > 2 m. |
| `distance_node` | `distance_node.py` | Interactive **debug tool**: draw an ROI with the mouse on the RGB feed, prints median depth. Not used at runtime. |
| `dataset_collector` | `dataset_collector.py` | Saves RGB frames at 1 Hz to `~/auv_dataset/images/`. ⚠ File has a syntax issue — duplicated header inside `__init__`. |
| `test_ROI_Publisher` | `test_ROI_publisher.py` | Publishes a fake `Detection2DArray` at 2 Hz for downstream testing. |
| (not wired) | `test.py` | Scratch file. |

---

## 4. `control_system` package

Aditya's package — everything from thruster allocation up to path following.

### 4.1 Entry points (`setup.py`)

| Entry point | Module | Role |
|---|---|---|
| `allocation_matrix` | `allocation_matrix.py` | Maps `/cmd_vel` (6-DOF wrench) → 5 thrust commands via Moore-Penrose pseudo-inverse of a 6×5 allocation matrix. Geometry: `W=0.35, Lf=Lr=0.30`. **No sway authority** (Fy row is zero). Output clipped to ±100 N. |
| `global_planner` | `global_planner_node.py` | Builds a voxel occupancy grid from `/cloud_map`, runs 3D A*, then a 4-tier post-processing chain: (1) collinear prune, (2) cubic spline smooth, (3) arc-length resample @ 0.3 m, (4) curvature limit (κ_max=0.4 → R_min=2.5 m), (5) validity check against grid. Publishes `nav_msgs/Path` on `/planned_path`. Exposes service `plan_path` (`custom_interfaces/PlanPath`). Smoothes per-point yaw/pitch with a 5-sample moving average. |
| `los_controller` | `los_control.py` | **VFG (Vector Field Guidance) + ILOS** path follower with a built-in Tkinter live-tuning GUI (toggle with `--ros-args -p gui:=false`). Subscribes `/planned_path` + `/odom`, publishes `/cmd_vel`. ~16 tunable parameters declared via `declare_parameter`; saves/loads to `~/.ros/vfg_params.yaml`. Handles a position-hold state at the final waypoint. |
| `6dof_pid` | `6dof_pid.py` | Older **adaptive-LOS + 6-DOF PID** controller (X/Z/yaw active, sway forced to 0). Lookahead δ = (δ_max − δ_min)·exp(−k·e_ct) + δ_min. Same in/out topics as `los_controller`. |

`6dof_pid copy.py` and `los_control copy.py` exist as backups — ignore for new work.

### 4.2 Coordinate / sign conventions

- `/cmd_vel` is a 6-DOF wrench in the body frame: `linear.{x,y,z}` = surge,
  sway, heave forces; `angular.{x,y,z}` = roll, pitch, yaw torques.
- Allocation matrix rows are `[Fx, Fy, Fz, τx, τy, τz]`. Rows for sway and
  the third vertical thruster's contribution to pitch encode the physical
  geometry — read `allocation_matrix.py` directly when re-tuning thruster
  positions.

---

## 5. `prequalification_bt` package

C++ BehaviorTree.CPP mission for the pre-qualification course (start gate +
red pole orbit + return). Built with `ament_cmake`.

### 5.1 Files

- `main.cpp` — wires up the ROS node, the shared `RobotContext` (sensor
  callbacks, `/cmd_vel` pub, `plan_path` client, TF buffer), loads the BT XML
  from `share/prequalification_bt/prequalification.xml` (or argv[1]), and
  ticks at 10 Hz.
- `bt_nodes.h` / `bt_nodes.cpp` — node classes + `registerAllNodes`.
- `prequalification.xml` — the mission tree.

### 5.2 `RobotContext` (the shared blackboard object)

Provides every BT node a single interface to:

- `getCurrentPose()` — combines `/odom` position with `/imu` yaw.
- `isObjectSeen(name)` / `getObjectPosition(name, …)` — query
  `/detections_3d` (recognises `GATE`, `POLE`, `SHARK` aliases mapping to
  multiple class IDs like `preq_gate`, `red_pole`, etc.).
- `getGlobalObjectPose(name, out)` — transforms camera-frame detection to
  `map` frame via TF (`zed_camera_front_link_optical_frame` → `map`).
- `publishCmdVel(...)` and `stopMotion()`.

### 5.3 BT node catalog

| Node | Kind | Purpose |
|---|---|---|
| `AllSystemsOK` | Condition | Passes once `/odom` + `/imu` have arrived. |
| `SaveToBlackboard` | SyncAction | Saves current pose to BB key (used for `T0`). |
| `DiveToDepth` | StatefulAction | P-control on heave until odom.z ≈ target. |
| `IsObjectSeen` | Condition | Polls detections for an alias. |
| `Do360Turn` | StatefulAction | Spin at `TURN_RATE=0.4 rad/s`; SUCCESS early on object sight. |
| `Full360Scan` | StatefulAction | Same spin but always completes a full revolution. |
| `NavigateTo` | StatefulAction | P-controller surge + yaw + heave to a `Pose`. Must align (<0.4 rad) before surging. |
| `NavigateAround` | StatefulAction | Three-phase orbit: APPROACH → ORBIT (full 360°) → RETURN. |
| `NavigateBelowObject` | StatefulAction | Position directly below a tracked object. |
| `PlanPathTo` | StatefulAction | Calls `plan_path` service with a target Pose. |
| `WaitUntilReached` | Condition | True when pose error within `tolerance`. |
| `CalculateObjectTarget` | SyncAction | Computes a global pose at (forward, lateral, vertical) offsets from an object's pose. |
| `MoveStraight` | StatefulAction | Constant surge command. |
| `TrackObject` | StatefulAction | Spins until object found, writes its global pose to BB. |

### 5.4 Mission flow (from `prequalification.xml`)

1. `AllSystemsOK` → save current pose as `T0`.
2. `Full360Scan` (mapping spin).
3. `Do360Turn` searching for GATE → `TrackObject` writes `{GatePose}`.
4. Compute `T1` = GatePose + 0.5 m forward → `PlanPathTo(T1)` → wait.
5. Compute `T2` = GatePose − 2.5 m forward → `PlanPathTo(T2)` → wait.
6. `MoveStraight` until POLE seen → `TrackObject` writes `{PolePose}`.
7. Orbit pole at ±2 m lateral, +2 m forward (waypoints `P1`, `P2`, `P3`).
8. Return: `T2` → `T1` → `T0`.

Default seed values in `main.cpp`: `T1 = (5.0, 0, 0.5)`, `T2 = (1.5, 0, 0.5)`.

---

## 6. `custom_interfaces` package

Pure interface package. One service:

```
# srv/PlanPath.srv
geometry_msgs/Pose target
---
nav_msgs/Path trajectory
bool success
string message
```

Built with `rosidl_default_generators`. Used by `global_planner_node`
(server) and `prequalification_bt` (client).

---

## 7. Key topic / service inventory

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU plugin (sim) **or** `imu_node` (hw) → controllers, BT |
| `/altimeter` | `std_msgs/Float64` | custom Gazebo plugin → BT |
| `/odom` | `nav_msgs/Odometry` | `rgbd_odometry` (RTAB) → planner, LOS, BT |
| `/cloud_map` | `PointCloud2` | RTAB-Map → planner |
| `/planned_path` | `nav_msgs/Path` | planner → LOS / 6dof_pid |
| `/goal_pose` | `PoseStamped` | RViz → planner |
| `/cmd_vel` | `Twist` | LOS / 6dof_pid / BT / gamepad → `allocation_matrix` |
| `/hydrogen/<thruster>/cmd_thrust` | `Float64` | `allocation_matrix` / teleop → bridge → Gazebo |
| `/camera/RGB_image_raw/{front,down}` | `Image` | Gazebo → YOLO, depth fusion, RTAB |
| `/camera/depth_image_raw/{front,down}` | `Image` | Gazebo → depth fusion, RTAB |
| `/camera_info_{front,down}` | `CameraInfo` | Gazebo → depth fusion, RTAB |
| `/detections_2d` | `Detection2DArray` | `yolo_node` |
| `/detections_3d` | `Detection3DArray` | `data_distance_node` → BT |
| `plan_path` (srv) | `custom_interfaces/PlanPath` | BT (client) → planner (server) |

---

## 8. Tuning knob cheatsheet

| What | Where |
|---|---|
| AUV mass / inertia / collision-box (buoyancy) | `hydrogen/model/main_frame.xacro` |
| Hydrodynamic damping (Fossen coeffs) | `hydrogen/model/robot.gazebo` (`Hydrodynamics` plugin) |
| Thruster coefficient / prop diameter | `hydrogen/model/robot.gazebo` (one block per thruster) |
| Fluid density (sim) | `hydrogen/worlds/buoyant_pool.sdf` (`Buoyancy` plugin) |
| Camera mounting poses | `hydrogen/model/robot.xacro` (xyz/rpy on `zed_camera` macros) |
| Spawn pose defaults | `hydrogen/launch/model.launch.py` (`DeclareLaunchArgument`) |
| Allocation matrix geometry | `control_system/allocation_matrix.py` (`W`, `Lf`, `Lr`) |
| Voxel size / inflation / smoothing | `control_system/global_planner_node.py` (top of `__init__`) |
| VFG/ILOS gains | `control_system/los_control.py` (`PARAMS` list) + live Tk GUI / `~/.ros/vfg_params.yaml` |
| 6-DOF PID gains | `control_system/6dof_pid.py` (`declare_parameter` block) |
| BT motion gains | `prequalification_bt/bt_nodes.h` (`static constexpr K_…` in each action class) |
| HSV thresholds for orange detection | `hydrogen/hydrogen/data_distance_node.py` (`ORANGE_HSV_LO/HI`, `POLE_ASPECT_THRESHOLD`) |

---

## 9. Launching the stack

```bash
# Sim only
ros2 launch hydrogen model.launch.py            # Gazebo + bridge + allocation_matrix

# Spawn override
ros2 launch hydrogen model.launch.py x:=0 y:=0 z:=2.0 Y:=1.57

# Add SLAM + global planner on top
ros2 launch hydrogen RTAB_map.launch.py

# Path follower
ros2 run control_system los_controller          # opens Tk tuner by default
ros2 run control_system los_controller --ros-args -p gui:=false

# Vision
ros2 run hydrogen yolo_node                     # ML detector
ros2 run hydrogen data_distance_node            # HSV+depth detector (preq tasks)

# Mission
ros2 run prequalification_bt prequalification

# Hardware bring-up (Raspberry Pi)
ros2 launch hydrogen AUV.launch.py
```

> **Pre-flight:** `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` must include the directory
> with `libauv_absolute_depth.so` (the altimeter plugin). See top-level
> `ReadMe.md`.

---

## 10. Gotchas / TODOs worth knowing

- `dataset_collector.py` is malformed (duplicated header inside `__init__`)
  and won't import cleanly — fix before reusing.
- `6dof_pid copy.py` and `los_control copy.py` are unused backups; the
  active controllers are the un-suffixed versions.
- `controller_node` (attitude) and `los_controller` (path) both publish
  control commands but on **different** topics (`new_thrust_*` vs `/cmd_vel`).
  Don't run both as drivers of the same thruster set without checking the
  blender in `thruster_teleop.py`.
- The allocation matrix has **zero sway authority** by design — controllers
  must produce surge / yaw / heave commands only (sway is force-zeroed in
  `6dof_pid.py` line ~104, and absent from `los_control.py` outputs).
- `RTAB_map.launch.py` spawns `global_planner` as part of the SLAM stack —
  this is convenient but means the planner only exists when SLAM is up.
- BT's `IsObjectSeen` matches multiple class IDs per alias
  (`GATE` ⇢ `left_gate_pole | right_gate_pole | preq_gate`, etc.) — keep
  detector class names in sync.
- Top-level `ReadMe.md` still references *Hydrogen and Deuterium*. Deuterium
  is the next prototype; everything in `src/` currently targets Hydrogen.

---

## 11. Pointers to memory

For Claude: when re-entering this workspace, also check
`~/.claude/projects/-home-assemblex-Desktop-hydrogen/memory/`. Notably
`auv_buoyancy_tuning.md` documents how to keep net buoyancy at ~0.5 % positive
across changes to main-frame mass or collision-box dimensions.
