# 🏎️ 48 Hours of Full Throttle: F1TENTH @ ROSCon France

<p align="center">
    <img src="https://github.com/user-attachments/assets/d728106c-609e-4dbc-8afa-7903e97af3c5" width="400" alt="Our Team with the Car" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros" alt="ROS 2 Jazzy" />
  <img src="https://img.shields.io/badge/Placement-4th_Place_🏆-brightgreen?style=for-the-badge" alt="4th Place" />
  <img src="https://img.shields.io/badge/Compute-Raspberry_Pi_4-C51A4A?style=for-the-badge&logo=raspberrypi" alt="Raspberry Pi" />
  <img src="https://img.shields.io/badge/Perception-RPLidar_C1-lightgrey?style=for-the-badge" alt="RPLidar C1" />
  <img src="https://img.shields.io/badge/Simulation-Webots_in_Docker-darkblue?style=for-the-badge" alt="Webots" />
</p>

---

## 📖 The Story: Two Days in the Sorbonne University "Cellars"

What happens when you lock a team of robotics students in the Sorbonne University stone-vaulted looking dungeon cellars for 48 hours with a fleet of 1/10th scale autonomous racecars, infinite coffee, and a competition deadline? 

This repository houses our complete autonomous racing stack developed for the **ROSCon France F1TENTH Hackathon**. Starting from zero baseline in a customized Webots Docker container, we designed, iterated, broke, fixed, and finally deployed our algorithms onto the physical 1/10th-scale car (powered by an onboard **Raspberry Pi** and an **RPLidar C1**).

After endless tuning runs, close calls with the track walls, and wheel-to-wheel qualifying heats, **we placed 4th overall**! 

<p align="center">
    <img src="https://github.com/user-attachments/assets/4e7708fc-de6b-46a1-9135-a2b9c6649d89" width="350" alt="ROSCon France F1TENTH Grid" />
</p>

---

## 🛠️ The Track & Simulation Playground

Before touching the physical hardware, everything lived in simulation. The organizers provided a locked **Webots** Docker environment reproducing the exact track topology and physical dimensions of the venue.


### The Webots Tuning Grind
We spent hours in Webots dialing in vehicle model parameters, latencies, and steering reaction delays:
* **The Screencast**: We recorded our sim runs to debug edge cases where the vehicle clipped corners or lost track boundary line-of-sight.
* **Sim-to-Real Friction**: In Webots, tire grip was predictable. On the real dungeon floor (smooth concrete/linoleum), sudden throttle spikes caused immediate spin-outs. We had to tune our rollout costs specifically to penalize excessive steering angles at higher velocities.

<p align="center">
  <video src="https://github.com/user-attachments/assets/3fb6a182-a385-4403-8c25-e6d220143429" controls="controls" width="700">
  </video>
</p>

---

## 🧠 Algorithmic Journey: How We Built the Racers

We started where most F1TENTH teams start: classical baselines like **Follow the Gap (FTG)** and **Disparity Extender**. While these reactive algorithms safely kept the car off the walls, they were fundamentally greedy: they couldn't plan ahead, they oscillated in tight chicanes, and they carried almost zero momentum through the bends.

We needed a controller that could simulate multiple candidate futures simultaneously in real-time. Enter **Model Predictive Path Integral (MPPI)**.

---

### Step 1: Pure Reactive Dynamic-Horizon MPPI (`mppi_car.py`)

Our first major breakthrough was building a **purely reactive MPPI** controller running at ~30–40 Hz. The goal: zero reliance on SLAM, zero map files, zero odometry drift vulnerability. Just raw LiDAR ranges transformed into candidate trajectory rollouts.

#### 1. Kinematic Rollouts
At each control step, the controller samples $K$ perturbations around a base steering input $\mathbf{u} = [\delta, v]^T$:

$$\delta_k = \delta_{\text{nominal}} + \epsilon_k, \quad \epsilon_k \sim \mathcal{N}(0, \sigma^2)$$

For each candidate $k$, we project the car forward over a discrete prediction horizon $N$ using a kinematic bicycle model:

$$\begin{aligned} x_{t+1} &= x_t + v_t \cos(\theta_t) \Delta t \\ y_{t+1} &= y_t + v_t \sin(\theta_t) \Delta t \\ \theta_{t+1} &= \theta_t + \frac{v_t}{L} \tan(\delta_t) \Delta t \end{aligned}$$

#### 2. Dynamic Prediction Horizon
A static horizon causes problems:
* Too long $\rightarrow$ trajectories project through walls on sharp 90° bends, triggering false collisions.
* Too short $\rightarrow$ the car can't see the end of a long straight and brakes prematurely.

We implemented an adaptive horizon $N_{\text{dyn}}$ scaled directly by the open space reported by the forward LiDAR beams:

$$N_{\text{dyn}} = \text{clamp}\left(\alpha \cdot d_{\text{forward}}, \, N_{\min}, \, N_{\max}\right)$$

#### 3. Trajectory Cost Formulation
Each rollout candidate is scored against the processed LiDAR scan:

$$J_k = \sum_{t=1}^{N} \left( w_{\text{wall}} \cdot \mathcal{C}_{\text{clearance}}(x_t, y_t) + w_{\text{heading}} \cdot \vert{}\theta_t - \theta_{\text{target}}\vert{} - w_{\text{speed}} \cdot v_t \right)$$

* $\mathcal{C}_{\text{clearance}}$ severely penalizes trajectories encroaching within car radius $r_{\text{safe}}$ of any LiDAR hit point.
* Trajectories predicting collisions receive an infinite cost penalty.

#### 4. Path Integral Action
The optimal steering command is the softmax-weighted sum across all $K$ candidates:

$$w_k = \frac{\exp\left(-\frac{1}{\lambda} (J_k - \min_j J_j)\right)}{\sum_{i=1}^K \exp\left(-\frac{1}{\lambda} (J_i - \min_j J_j)\right)}$$

$$u^* = \sum_{k=1}^K w_k u_k$$

> **The Result:** The car drove smoothly, carved through chicanes without oscillation, and could dynamically avoid rogue obstacles or stalled cars without any prior map knowledge.

---

### Step 2: Going Faster with Global Grafted MPPI (`global_mppi.py`)

Pure reactive driving is safe, but it doesn't take the racing line. In racing, you don't stay in the center of the track—you enter wide, clip the apex, and exit wide.

For Day 2, we asked: **Can we keep the bulletproof collision-avoidance of MPPI while grafting a global optimal raceline onto it?**

```
             ┌─────────────────────────────┐
             │      Offline Raceline       │
             │   (x, y, target_v, kappa)   │
             └──────────────┬──────────────┘
                            │ (Map Frame)
                            ▼
 ┌──────────────┐    ┌─────────────────────┐    ┌───────────────────────────┐
 │ AMCL / State ├───►│  Coordinate Graft   ├───►│  Global Grafted MPPI      │
 │  Estimation  │    │ (Transform to Base) │    │  - Curvature Braking      │
 └──────────────┘    └─────────────────────┘    │  - Lateral Raceline Bias  ├──► Drive Cmd
                                                │  - LiDAR Collision Veto   │
      Raw LiDAR Scan ──────────────────────────►│                           │
                                                └───────────────────────────┘
```

#### 1. Curvature-Based Speed Anticipation
Rather than reacting to corners when the LiDAR sees the wall approaching, we compute the prospective curvature $\kappa$ from the offline raceline waypoints ahead. We bound target velocity by tire lateral grip limit $a_{y,\text{grip}}$:

$$v_{\text{target}}(s) = \min\left(v_{\max}, \; \sqrt{\frac{a_{y,\text{grip}}}{\kappa(s + d_{\text{lookahead}})}}\right)$$

This allows the vehicle to brake **before** entering the turn while traveling flat out down the straight.

#### 2. Raceline Heading Bias (The "Graft")
Instead of forcing a rigid path tracker (like Pure Pursuit) which crashes if an obstacle blocks the line, we inject the raceline heading $\theta_{\text{raceline}}$ as a soft guidance bias into the MPPI cost:

$$J_{\text{grafted}} = J_{\text{reactive}} + w_{\text{line}} \cdot \Vert{}\mathbf{p}_{\text{rollout}} - \mathbf{p}_{\text{raceline}}\Vert{}^2$$

#### 3. Graceful Fallback
If AMCL localization lost confidence or the TF tree lagged, the node gracefully clamped the raceline bias weight $w_{\text{line}} \to 0$ and reverted to pure reactive LiDAR MPPI. Zero crashes due to localization loss!

---

## 📂 Repository Layout

```text
.
├── dependencies.repos          # VCS import file for rf2o laser odometry
├── README.md                   # You are here!
├── racer_cpp/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── data/
│   │   └── raceline.csv        # Global raceline coordinates & curvature
│   ├── launch/
│   │   ├── reactive_racer.launch.py   # Pure LiDAR MPPI launch
│   │   └── grafted_raceline.launch.py # Raceline-assisted MPPI launch
│   ├── maps/
│   │   ├── track_map.yaml      # Competition map config
│   │   └── track_map.pgm       # Occupancy grid
│   └── src/
│       ├── baselines/          # Disparity extender, reactive gap, pure pursuit
│       ├── experimental/       # PCL DBSCAN clustering, OpenCV wall segmentation
│       └── scripts/
│           ├── mppi_car.py     # Reactive dynamic-horizon MPPI node
│           └── global_mppi.py  # Grafted global raceline MPPI node
└── racing_localization/
    ├── config/                 # EKF, AMCL, and RF2O parameter profiles
    ├── launch/                 # Localization and SLAM launch scripts
    └── src/                    # Custom wheel/joint odometry conversions
```

---

## 🚀 Quickstart & Setup

### 1. Clone into your ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/](https://github.com/)<your_username>/<your_repo_name>.git
```

### 2. Import External Dependencies
We use `rf2o_laser_odometry` to compute planar odometry from laser scans:
```bash
cd ~/ros2_ws
vcs import src < src/<your_repo_name>/dependencies.repos
```

### 3. Build Packages
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 🏎️ Running the Controllers

### Option A: Reactive MPPI (No map needed, robust)
```bash
ros2 launch racer_cpp reactive_racer.launch.py \
  scan_topic:=/scan \
  cmd_topic:=/drive
```

### Option B: Global Raceline MPPI (High speed, raceline tracking)
```bash
ros2 launch racer_cpp grafted_raceline.launch.py \
  raceline_csv:=$(ros2 pkg prefix racer_cpp)/share/racer_cpp/data/raceline.csv \
  line_bias:=0.15 \
  ay_grip:=2.5 \
  scan_topic:=/scan \
  cmd_topic:=/drive
```

---

## 🏆 Takeaways & Hackathon Reflections

* **Sim-to-Real is never 1:1**: The latency between command publication and steering servo response on the physical car was significantly higher than in Webots. Introducing a 1-step input buffer in our kinematic predictions made the physical car substantially smoother.
* **Fallback architectures win races**: Several teams suffered catastrophic crashes when their localization drifted. Because our grafted controller fell back to local reactive LiDAR rollouts the instant localization variance spiked, our car never hit a wall during the final rounds.
* **Huge thanks** to the ROSCon France organizers for an unforgettable hackathon experience!
