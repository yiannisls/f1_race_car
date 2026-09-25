# F1TENTH Autonomous Racing — ROSCon France Hackathon

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Placement](https://img.shields.io/badge/Result-4th_Place-brightgreen)
![Hardware](https://img.shields.io/badge/Hardware-Raspberry_Pi_%7C_RPLidar_C1-lightgrey)

Autonomous 1/10th scale racing algorithms developed and deployed during the 2-day ROSCon France hackathon. The system was initially developed in a Webots simulation environment and successfully transferred to physical F1TENTH vehicle hardware, securing a 4th place finish.

![Race Day](IMG_2440.jpg)

---

## Architecture Overview

The repository features two primary Model Predictive Path Integral (MPPI) rollout controllers alongside baseline reactive methods and localization pipelines.

### 1. Pure Reactive Dynamic-Horizon MPPI (`mppi_car.py`)
* **LiDAR-Only Rollouts**: Operates strictly on 2D `LaserScan` ranges without dependencies on global maps, SLAM, or odometry.
* **Dynamic Prediction Horizon**: Dynamically expands rollout distance/time along open straights and contracts horizon depth in tight corners or near walls to avoid path collapse.
* **Candidate Evaluation**: Simulates kinematic bicycle arcs across sampled steering inputs, penalizing track wall clearance violations, heading error, and wrong-direction trajectories.

### 2. Global Raceline Grafted MPPI (`global_mppi.py`)
* **Optimal Line Fusion**: Blends offline minimum-time/minimum-curvature racelines (`raceline.csv`) as a soft lateral bias into the reactive target heading.
* **Curvature Speed Anticipation**: Calculates upcoming curvature ($\kappa$) from the global profile to compute dynamic braking targets ($v = \sqrt{a_y / \kappa}$) before corners enter LiDAR line-of-sight.
* **Graceful Fallback**: Automatically reverts to pure reactive LiDAR rollouts if the TF transform (`map -> base_link`) is degraded or unavailable.

### 3. Auxiliary & Research Modules
* **`racing_localization`**: Nav2 AMCL map-based localization, EKF fusion, and ground-truth validation supervisors.
* **Perception & Baselines (`racer_cpp/src/`)**: Pure pursuit, disparity extender, reactive gap follower, wall-following controller, DBSCAN obstacle clustering, and camera wall segmentation.

---

## Track & Simulation

The algorithms were tested on the official hackathon track layout before hardware deployment. 

![Track Map](IMG_2428.jpg)

*(Insert Webots Screencast Here: e.g., `![Webots Sim](webots_screencast.gif)`)*

**Note on Simulation:** The Webots environment was provided via a locked Docker container by the competition organizers. While the ROS 2 algorithms in this repository are standalone, replicating the exact simulation visualizer requires the competition Docker image.

---

## Repository Structure

```text
.
├── dependencies.repos
├── README.md
├── racer_cpp/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── data/
│   │   └── raceline.csv
│   ├── launch/
│   │   ├── reactive_racer.launch.py
│   │   └── grafted_raceline.launch.py
│   ├── maps/
│   │   ├── track_map.yaml
│   │   └── track_map.pgm
│   └── src/
│       ├── baselines/          # Standard F1TENTH algorithms
│       ├── experimental/       # Perception and clustering nodes
│       └── scripts/
│           ├── global_mppi.py  # Grafted raceline controller
│           └── mppi_car.py     # Pure reactive controller
└── racing_localization/
    ├── config/
    ├── launch/
    └── src/
```

---

## Installation & Setup

### 1. Workspace Preparation
Clone this repository into a ROS 2 Jazzy workspace. 

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/](https://github.com/)<your_username>/<your_repo_name>.git
```

### 2. External Dependencies
The localization stack relies on `rf2o_laser_odometry`. Import it using `vcs`:

```bash
cd ~/ros2_ws
vcs import src < src/<your_repo_name>/dependencies.repos
```

### 3. Build
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Execution Commands

### Run Reactive MPPI (Hardware / Custom Sim)
Executes the pure LiDAR rollout controller.

```bash
ros2 launch racer_cpp reactive_racer.launch.py scan_topic:=/scan cmd_topic:=/drive
```

### Run Global Line Grafted MPPI
Executes the rollout controller with raceline tracking and curvature anticipation.

```bash
ros2 launch racer_cpp grafted_raceline.launch.py \
  raceline_csv:=$(ros2 pkg prefix racer_cpp)/share/racer_cpp/data/raceline.csv \
  line_bias:=0.15 \
  ay_grip:=2.5 \
  scan_topic:=/scan \
  cmd_topic:=/drive
```

---

## Team

![Team Photo](IMG_2442.jpg)


