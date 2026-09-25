# Ground-Truth Supervisor — setup (3 small edits, no change to your car)

Goal: publish the car's TRUE pose so `drift_evaluator` can measure EKF drift as a
number. Read-only; it never moves anything and doesn't touch `controller_violet`.

## Step 1 — give your car a DEF name in the .wbt
Find your car's `Robot { ... }` node and prefix it with a DEF:
```
DEF YELLOW_CAR Robot {
  controller "<extern>"        # (your existing controller_violet driver)
  ...
}
```
If you use a different DEF, add it to `CAR_DEF_CANDIDATES` in
`ground_truth_supervisor.py`. The node prints what it tried if it fails.

## Step 2 — add a tiny Supervisor robot to the .wbt
Anywhere at the top level of the world, add:
```
Robot {
  name "ground_truth_supervisor"
  controller "<extern>"        # driven by webots_ros2 from your launch file
  supervisor TRUE              # <-- REQUIRED: gives it pose-reading powers
  children []                  # no body/sensors needed
}
```
(There can be many supervisor robots in modern Webots; this won't conflict with
the auto-spawned Ros2Supervisor that publishes /clock.)

## Step 3 — load the driver from your launch file
Add a WebotsController for it (mirrors how your car driver is launched):
```python
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController

ground_truth = WebotsController(
    robot_name="ground_truth_supervisor",
    parameters=[{"robot_description": "<path>/ground_truth.urdf",
                 "use_sim_time": True}],
)
# add `ground_truth` to your LaunchDescription
```
Make sure `ground_truth_supervisor.py` is importable on PYTHONPATH (install it in
the package, or set WEBOTS_CONTROLLER PYTHONPATH). The `<plugin type=...>` in
ground_truth.urdf points to `ground_truth_supervisor.GroundTruthDriver`.

### Alternative (no launch edits): classic controller
Set the supervisor robot's `controller "ground_truth_supervisor"` and drop
`ground_truth_supervisor.py` into that controller's folder. The `__main__`
block runs it standalone. (You still need ROS sourced in that terminal.)

## Verify
```bash
ros2 topic echo /yellow_car/ground_truth --once
```
You should see a pose. If the log says "Could NOT find the car node by DEF
name", revisit Step 1.

## Axis system
Default is ENU (z-up), correct for modern Webots. If your world is legacy NUE
(y-up) and yaw looks wrong, set `AXIS_SYSTEM = "NUE"` at the top of the script.
