# Step 1 — Localization validation (do this BEFORE touching slam_toolbox)

Goal: confirm the EKF produces a smooth, low-drift `odom -> base_link` transform
that survives a few corners. If this is wrong, every downstream layer (SLAM map,
Heilmeier raceline, Frenet) will be wrong too.

## 0. Build & install
```bash
cd ~/your_ros2_ws
# clone rf2o once if you don't have it:
#   git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry src/rf2o_laser_odometry
colcon build --packages-select racing_localization rf2o_laser_odometry
source install/setup.bash
```
Install the two ROS packages if missing:
```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-tf2-tools
```

## 1. Launch
```bash
# Terminal A: Webots sim (your existing bring-up)
# Terminal B:
ros2 launch racing_localization localization.launch.py use_sim_time:=true
```

## 2. Sanity checks (in order)

### a) Topics alive
```bash
ros2 topic hz /yellow_car/odom_wheel    # ~50 Hz
ros2 topic hz /yellow_car/odom_rf2o     # ~20 Hz
ros2 topic hz /odometry/filtered        # ~50 Hz
```
If `/yellow_car/odom_rf2o` is silent: rf2o isn't getting `/yellow_car/scan` or
the base_link->laser TF is missing.

### b) TF tree has exactly ONE odom->base_link owner
```bash
ros2 run tf2_tools view_frames
# Open frames.pdf. You MUST see:  odom --(ekf_filter_node)--> base_link --> laser
# If you see TWO publishers of odom->base_link, rf2o publish_tf wasn't set false.
```

### c) No "extrapolation into the future" spam
All nodes must share sim time. Every node here sets use_sim_time:=true.
If you still see TF time errors, confirm Webots is publishing /clock and that
your other (existing) nodes also run with use_sim_time:=true.

## 3. The drift test (the real verdict)
1. In RViz: Fixed Frame = `odom`. Add: TF, LaserScan (`/yellow_car/scan`),
   Odometry (`/odometry/filtered`).
2. Drive ONE slow lap manually / at low speed and return to the start line.
3. Watch the filtered odom path. Compare start vs. end pose.

PASS criteria (sim, slow lap):
- Position drift on return-to-start < ~0.3 m
- Heading drift < ~5 deg
- Path is smooth (no jumps/teleports) through corners

If drift is large:
- Heading drifting in corners  -> rf2o vyaw not trusted enough. In ekf.yaml
  lower process yaw noise OR raise `var_vyaw` in odometry_node (distrust wheel more).
- Path too jumpy               -> raise rf2o `freq` to 30, or raise odom1
  twist_rejection_threshold.
- Speed/scale wrong            -> check wheelbase (0.257) and that
  /yellow_car/current_speed is in m/s.

## 4. Ground-truth overlay (optional, recommended)
You mentioned Webots can expose true pose via the supervisor. If you publish it
(e.g. /yellow_car/ground_truth), add it in RViz as a second Odometry/Path and
overlay against /odometry/filtered. That turns the drift test from "looks ok"
into a measured number.

## 5. MEASURED drift with Webots ground truth (recommended)
Instead of eyeballing, get hard numbers (ATE, return-to-start drift).

### One-time setup
Follow `webots_supervisor/README_GROUND_TRUTH.md` (3 small .wbt/launch edits;
does NOT touch your car or controller_violet). Confirm GT is live:
```bash
ros2 topic echo /yellow_car/ground_truth --once
```

### Run the evaluation
```bash
# with sim + localization.launch.py already running:
ros2 run racing_localization drift_evaluator.py
# optional: -p csv:=/tmp/run1.csv  -p align_samples:=40  -p return_radius_m:=0.5
```
Then drive a lap. The evaluator:
1. auto-aligns the raw Webots world pose to the EKF odom frame (first ~40 moving
   samples, rigid 2D fit),
2. prints a live line: `pos_err | yaw_err | ATE | path | return_drift`,
3. on Ctrl-C prints a FINAL REPORT and writes a CSV.

PASS (slow sim lap): ATE < ~0.30 m, heading drift < ~5 deg, smooth path.

### RViz overlay (visual + numeric together)
Fixed Frame = `odom`. Add two Odometry displays:
- `/odometry/filtered`            (EKF, e.g. red)
- `/yellow_car/ground_truth_aligned` (aligned GT, e.g. green)
They should sit on top of each other; the gap you see IS the drift.
(Use the *aligned* topic, not the raw `/yellow_car/ground_truth`, which is in
the Webots world frame and won't overlay.)

### Tuning from the numbers
- Heading error grows in corners -> trust rf2o yaw more: lower process yaw noise
  in ekf.yaml, or raise `var_vyaw` in odometry_node (distrust wheel yaw further).
- Position error grows on straights -> speed scale issue: re-check `wheelbase`
  and that `/yellow_car/current_speed` is m/s.
- Sudden jumps -> raise rf2o `freq` to 30 or raise odom twist_rejection_threshold.

## Only after PASS:
Proceed to slam_toolbox in MAPPING mode, feeding it /yellow_car/scan (RAW, walls
intact) + the EKF's odom->base_link. Then map_saver, then localization mode.
That is Step 2.
