# Step 2 — SLAM mapping & localization (slam_toolbox)

Prereq: Step 1 PASSED. The EKF must be publishing a stable odom->base_link.
slam_toolbox sits ON TOP of it and adds map->odom (drift correction).

## TF chain (must look exactly like this)
```
map --(slam_toolbox)--> odom --(robot_localization EKF)--> base_link --(static)--> laser
```
Verify with: `ros2 run tf2_tools view_frames`

## A. Build the map
```bash
# Terminal A: Webots sim
# Terminal B: localization (EKF + rf2o) -- the odom->base_link layer
ros2 launch racing_localization localization.launch.py
# Terminal C: SLAM mapping
ros2 launch racing_localization slam.launch.py            # mode:=mapping is default
```
In RViz: Fixed Frame = `map`, add the `Map` display on `/map`, plus `LaserScan`
`/yellow_car/scan` and TF. Drive 1-2 slow laps. Watch the walls fill in and the
loop close (the map will "snap" straight when loop closure fires — that's good).

GOOD map signs: walls are thin/crisp, the loop closes (start and end overlap),
no double walls / smearing.
BAD map signs: double walls or smearing -> the EKF odom feeding SLAM is too
drifty or scans are stale. Go re-check Step 1 (and rf2o scan rate).

## B. Save the map (save BOTH formats)
```bash
# 1) Pose-graph (to RESUME mapping or run slam_toolbox localization mode):
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/USER/maps/track_map'}"
#    -> creates track_map.data + track_map.posegraph

# 2) Occupancy grid pgm/yaml (for the Heilmeier raceline tool in Step 3):
ros2 run nav2_map_server map_saver_cli -f /home/USER/maps/track_map
#    -> creates track_map.pgm + track_map.yaml
```
(Adjust the service name prefix to match your node; with name "slam_toolbox"
the services are /slam_toolbox/serialize_map etc. Check `ros2 service list`.)

## C. Race-time localization (after mapping)
Edit `config/slam_localization.yaml`:
  - `map_file_name: /home/USER/maps/track_map`   (no extension)
  - `map_start_pose: [x, y, yaw]`                 (your grid start in map frame)
Then:
```bash
ros2 launch racing_localization slam.launch.py mode:=localization
```
Now slam_toolbox localizes against the fixed map instead of growing it — lower
CPU, stable map frame for the raceline.

## D. Does the ground-truth drift test still apply?
Yes, and it gets BETTER. With SLAM running, compare GROUND TRUTH against the
full map->base_link pose (not just odom->base_link). Two ways:
  - Quick: in RViz overlay /yellow_car/ground_truth_aligned (green) vs the
    car's TF in the `map` frame. With loop closure, drift should stay bounded
    (not grow unbounded like pure odometry).
  - The drift_evaluator currently compares GT vs /odometry/filtered (odom
    frame). That still measures the EKF layer. If you want it to grade the
    SLAM-corrected pose instead, tell me and I'll add a map-frame TF lookup so
    it scores map->base_link. That's the number that matters for racing.

## Pi-5 notes
- `map_update_interval: 1.0` and `resolution: 0.05` keep CPU sane. If the Pi
  struggles, raise map_update_interval to 2.0 and throttle_scans to 2.
- Localization mode is much lighter than mapping -> always race in localization
  mode, never keep mapping during a race.

## Next: Step 3 — offline Heilmeier minimum-curvature raceline
Feed track_map.pgm/.yaml to the raceline optimizer to get the global racing
line, then the Frenet planner tracks it. That's the next build.
