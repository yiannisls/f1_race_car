#!/usr/bin/env python3
"""
reactive_plus_line_node.py
==========================
PATH 2: the proven reactive MPPI navigation (from mppi_car.py, ~15s laps) with
the global raceline grafted on as a SOFT bias + speed anticipation.

DESIGN
  - Reactive navigation is PRIMARY and unchanged: the LiDAR rollout/scoring that
    already drives the whole track stays exactly as-is (compute_target, rollouts,
    clearance scoring, choose_best_rollout, escape).
  - The global raceline (raceline.csv) is added in TWO soft, additive ways:
      1. STEERING BIAS: the reactive target_y is gently nudged toward where the
         global line says to be:  target_y = (1-bias)*reactive + bias*line.
         `line_bias` is small (default 0.15). At 0 -> pure reactive.
      2. SPEED ANTICIPATION: the global line's UPCOMING curvature sets a speed cap
         (v = sqrt(ay_grip/kappa_ahead)) so the car brakes BEFORE a corner the
         LiDAR cannot see yet. This only ever LOWERS speed (a safety cap).
  - GRACEFUL FALLBACK: if map->base_link TF is missing (localization lost), the
    bias and speed cap switch off and the controller is exactly the proven
    reactive node.

Vehicle model params are LEFT AS-IS from mppi_car.py (wheelbase 0.30, max_steer
0.50) to preserve the navigation that already works -- the line bias is soft
enough that the rollout model does not need to be exact.

RUN
  ros2 run racer_cpp reactive_plus_line_node --ros-args \
    -p raceline_csv:=/home/turtle/maps/raceline.csv \
    -p line_bias:=0.15 -p ay_grip:=2.5
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import tf2_ros


def yaw_from_quat(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


class ReactivePlusLinePlanner(Node):
    def __init__(self):
        super().__init__('reactive_plus_line_planner')
        
        # Resolve default raceline path cleanly
        try:
            pkg_share = get_package_share_directory('racer_cpp')
            default_csv = os.path.join(pkg_share, 'data', 'raceline.csv')
        except Exception:
            default_csv = ''

        self.declare_parameter('raceline_csv', default_csv)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/drive')

        self.csv_path = self.get_parameter('raceline_csv').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.scan_topic = '/yellow_car/scan'
        self.cmd_topic = '/yellow_car/cmd_ackermann'

        # ==========================================================
        # Vehicle model  (LEFT AS-IS from mppi_car.py -- do not change)
        # ==========================================================
        self.wheelbase = 0.30
        self.max_steer = 0.50
        self.rollout_steer_rate = 6.8

        self.num_steers = 111
        self.dt = 0.055

        self.min_horizon_time = 0.95
        self.mid_horizon_time = 1.45
        self.max_horizon_time = 2.35
        self.min_preview_distance = 1.05
        self.max_preview_distance = 4.15

        self.max_obstacle_range = 5.0
        self.front_angle_limit_deg = 120.0
        self.collision_radius = 0.14
        self.max_side_distance = 2.5

        self.smoothed_left = self.max_side_distance
        self.smoothed_right = self.max_side_distance
        self.have_sides = False
        self.side_smoothing_alpha = 0.22
        self.smoothed_far_front = self.max_obstacle_range
        self.have_far_front = False
        self.far_front_alpha = 0.30
        self.smoothed_target_y = 0.0
        self.have_target = False

        self.side_push_start = 0.48
        self.side_push_gain = 0.74
        self.max_side_push_y = 0.12

        self.corner_front_start = 3.05
        self.corner_front_full = 0.70
        self.corner_bias_gain = 0.15
        self.max_corner_bias_y = 0.18

        self.turn_sign = 0.0
        self.turn_timer = 0.0
        self.turn_hold_time = 0.36
        self.turn_start_strength = 0.26
        self.turn_flip_strength = 0.44

        self.racing_wall_buffer = 0.60
        self.racing_max_offset = 0.18
        self.racing_min_offset = 0.035
        self.center_limit = 0.13

        self.max_speed = 3.65
        self.min_speed = 0.25
        self.lateral_accel_limit = 4.55
        self.max_accel_step = 0.38
        self.straight_accel_step = 0.72
        self.max_decel_step = 0.60
        self.emergency_decel_step = 1.05
        self.max_steer_step = 0.42
        self.min_planning_speed = 1.00
        self.max_planning_speed = 3.10
        self.preview_boost = 0.58
        self.high_speed_horizon_speed = 2.20
        self.min_high_speed_horizon_time = 1.38
        self.turn_lookahead_min_time = 1.12
        self.turn_lookahead_mid_time = 1.30
        self.turn_lookahead_max_time = 1.55
        self.turn_lookahead_front_gate = 1.75
        self.pre_turn_front_start = 3.05
        self.pre_turn_front_mid = 1.55
        self.pre_turn_front_near = 0.85
        self.max_good_overshoot = 0.11
        self.max_relaxed_overshoot = 0.18
        self.straight_preferred_lat = 0.24
        self.straight_safe_lat = 0.34
        self.straight_relaxed_lat = 0.44
        self.no_opposite_target_strength = 0.55
        self.min_same_turn_target = 0.020
        self.escape_front_threshold = 0.20
        self.escape_clearance_threshold = 0.090
        self.escape_speed = 0.30

        self.previous_steer = 0.0
        self.previous_speed = self.min_speed
        self.previous_desired_steer = 0.0
        self.callback_count = 0

        # ==========================================================
        # GLOBAL-LINE GRAFT params (the only additions)
        # ==========================================================
        self.csv_path   = self.declare_parameter('raceline_csv', '').value
        self.line_bias  = self.declare_parameter('line_bias', 0.15).value      # 0=pure reactive
        self.map_frame  = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.line_lookahead_m = self.declare_parameter('line_lookahead_m', 1.0).value
        self.max_line_y = self.declare_parameter('max_line_y', 0.30).value     # clamp line target
        # speed anticipation
        self.use_line_speed = bool(self.declare_parameter('use_line_speed', True).value)
        self.ay_grip = self.declare_parameter('ay_grip', 2.5).value
        self.speed_lookahead_pts = int(self.declare_parameter('speed_lookahead_pts', 45).value)
        self.curv_floor = self.declare_parameter('curv_floor', 0.05).value
        self.line_speed_scale = self.declare_parameter('line_speed_scale', 1.0).value
        self.nn_back = int(self.declare_parameter('nn_back', 5).value)
        self.nn_fwd  = int(self.declare_parameter('nn_fwd', 30).value)

        # speed source mode: 'curvature' (C: curvature-based feasible speed primary,
        # real grip, anticipates), 'raceline' (B: raceline vx primary, optimistic),
        # or 'cap' (reactive primary, line only slows).
        self.speed_mode = self.declare_parameter('speed_mode', 'curvature').value
        # EMA smoothing of the target speed (reduces the jumpy binding-cap switching).
        # alpha in (0,1]: lower = smoother but laggier. Emergencies bypass this.
        self.speed_ema_alpha = self.declare_parameter('speed_ema_alpha', 0.3).value
        self.smoothed_target_speed = None

        self.have_line = False
        self.prev_idx = None
        if self.csv_path:
            self.load_raceline(self.csv_path)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn('no raceline_csv -> running PURE REACTIVE (no line graft).')

        # io
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yellow_car/rollouts', 10)
        self.best_path_pub = self.create_publisher(Path, '/yellow_car/best_path', 10)
        self.line_marker_pub = self.create_publisher(Marker, '/yellow_car/line_target', 10)
        # full raceline drawn as a blue line in the MAP frame (overlays the track)
        self.raceline_pub = self.create_publisher(Marker, '/yellow_car/raceline', 10)
        if self.have_line:
            # republish periodically so RViz always has it (latched-like)
            self.raceline_timer = self.create_timer(1.0, self.publish_raceline)

        self.get_logger().info(
            f'REACTIVE+LINE started. line_bias={self.line_bias} (0=pure reactive), '
            f'ay_grip={self.ay_grip}, line={"on" if self.have_line else "OFF"}')

    # ==========================================================
    # GLOBAL LINE helpers (additions)
    # ==========================================================
    def load_raceline(self, path):
        d = np.loadtxt(path, delimiter=',', comments='#')
        self.rl_x = d[:, 1]; self.rl_y = d[:, 2]
        self.rl_psi = d[:, 3]; self.rl_kappa = np.abs(d[:, 4]); self.rl_vx = d[:, 5]
        self.N = len(self.rl_x)
        self.rl_seg = np.hypot(np.diff(self.rl_x, append=self.rl_x[0]),
                               np.diff(self.rl_y, append=self.rl_y[0]))
        self.have_line = True
        self.get_logger().info(f'raceline loaded: {self.N} pts, lap {self.rl_seg.sum():.2f} m')

    def nearest_index(self, cx, cy):
        if self.prev_idx is None:
            d2 = (self.rl_x - cx) ** 2 + (self.rl_y - cy) ** 2
            idx = int(np.argmin(d2))
        else:
            offs = np.arange(-self.nn_back, self.nn_fwd + 1)
            cand = (self.prev_idx + offs) % self.N
            d2 = (self.rl_x[cand] - cx) ** 2 + (self.rl_y[cand] - cy) ** 2
            idx = int(cand[int(np.argmin(d2))])
            fwd = (idx - self.prev_idx) % self.N
            if fwd > self.nn_fwd and fwd < self.N - self.nn_back:
                idx = self.prev_idx
        self.prev_idx = idx
        return idx

    def global_line_query(self):
        """Returns (line_y, v_curv, v_ref, ok). line_y = lateral offset (base_link,
        left+) of a lookahead point on the global line. v_curv = curvature-based
        feasible speed. v_ref = raceline's own planned speed (primary for mode B).
        ok=False if TF unavailable (-> caller falls back to pure reactive)."""
        if not self.have_line:
            return 0.0, None, None, False
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return 0.0, None, None, False

        cx = tf.transform.translation.x
        cy = tf.transform.translation.y
        cyaw = yaw_from_quat(tf.transform.rotation.z, tf.transform.rotation.w)
        idx = self.nearest_index(cx, cy)

        # lookahead point along the line
        acc = 0.0; i = idx
        while acc < self.line_lookahead_m:
            acc += self.rl_seg[i]; i = (i + 1) % self.N
            if i == idx:
                break
        lx, ly = self.rl_x[i], self.rl_y[i]
        # transform into base_link, take lateral (left +)
        dx, dy = lx - cx, ly - cy
        c, s = math.cos(cyaw), math.sin(cyaw)
        line_y = -s * dx + c * dy
        line_y = max(-self.max_line_y, min(self.max_line_y, line_y))

        # speed anticipation from upcoming curvature (feasible, real grip)
        v_curv = None
        v_ref = None
        if self.use_line_speed:
            ahead = [(idx + j) % self.N for j in range(self.speed_lookahead_pts)]
            kap = max(float(np.max(self.rl_kappa[ahead])), self.curv_floor)
            v_curv = math.sqrt(self.ay_grip / kap) * self.line_speed_scale
            # raceline's OWN planned speed (vx), braking for the slowest upcoming
            # point (anticipation). This is the "v_ref" primary target for mode B.
            v_min_ahead = float(np.min(self.rl_vx[ahead]))
            v_ref = min(float(self.rl_vx[idx]), v_min_ahead) * self.line_speed_scale

        self._last_line_y = line_y
        return line_y, v_curv, v_ref, True

    # ==========================================================
    # (UNCHANGED reactive helpers from mppi_car.py)
    # ==========================================================
    def clamp(self, v, lo, hi): return max(lo, min(hi, v))

    def sign_or_zero(self, v, db=1e-6):
        if v > db: return 1.0
        if v < -db: return -1.0
        return 0.0

    def valid_range(self, scan, r):
        if math.isnan(r) or math.isinf(r): return False
        if r < scan.range_min or r > scan.range_max: return False
        return True

    def sector_values(self, scan, mn, mx):
        out = []
        for i, r in enumerate(scan.ranges):
            if not self.valid_range(scan, r): continue
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if mn <= a <= mx: out.append(min(r, self.max_obstacle_range))
        return out

    def sector_percentile(self, scan, mn, mx, p):
        v = self.sector_values(scan, mn, mx)
        if not v: return self.max_obstacle_range
        v.sort(); return v[int(self.clamp(p, 0, 1) * (len(v) - 1))]

    def sector_mean_top(self, scan, mn, mx):
        v = self.sector_values(scan, mn, mx)
        if not v: return self.max_obstacle_range
        v.sort(reverse=True); n = max(1, len(v) // 3)
        return sum(v[:n]) / n

    def make_steering_samples(self):
        s = set()
        for i in range(self.num_steers):
            r = i / max(1, self.num_steers - 1)
            s.add(round(-self.max_steer + 2.0 * self.max_steer * r, 5))
        for st in [-0.50,-0.44,-0.38,-0.32,-0.26,-0.20,-0.14,-0.08,-0.04,0.0,
                   0.04,0.08,0.14,0.20,0.26,0.32,0.38,0.44,0.50]:
            s.add(round(st, 5))
        return sorted(s)

    def scan_to_obstacle_points(self, scan):
        obs = []
        for i in range(0, len(scan.ranges), 2):
            r = scan.ranges[i]
            if not self.valid_range(scan, r): continue
            r = min(r, self.max_obstacle_range)
            a = scan.angle_min + i * scan.angle_increment
            if abs(math.degrees(a)) > self.front_angle_limit_deg: continue
            x = r * math.cos(a); y = r * math.sin(a)
            if x < -0.10: continue
            obs.append((x, y))
        return obs

    def get_side_distances(self, scan):
        rl = min(self.sector_percentile(scan, 28, 108, 0.20), self.max_side_distance)
        rr = min(self.sector_percentile(scan, -108, -28, 0.20), self.max_side_distance)
        if not self.have_sides:
            self.smoothed_left = rl; self.smoothed_right = rr; self.have_sides = True
        else:
            a = self.side_smoothing_alpha
            self.smoothed_left = (1-a)*self.smoothed_left + a*rl
            self.smoothed_right = (1-a)*self.smoothed_right + a*rr
        return self.smoothed_left, self.smoothed_right, rl, rr

    def get_far_front(self, scan):
        raw = self.sector_percentile(scan, -22, 22, 0.12)
        if not self.have_far_front:
            self.smoothed_far_front = raw; self.have_far_front = True
        else:
            a = self.far_front_alpha
            self.smoothed_far_front = (1-a)*self.smoothed_far_front + a*raw
        return self.smoothed_far_front

    def compute_side_push(self, rl, rr):
        p = 0.0
        if rl < self.side_push_start: p -= self.side_push_gain*(self.side_push_start-rl)
        if rr < self.side_push_start: p += self.side_push_gain*(self.side_push_start-rr)
        return self.clamp(p, -self.max_side_push_y, self.max_side_push_y)

    def update_turn_lock(self, raw_turn_y, front, ts):
        rs = self.sign_or_zero(raw_turn_y, 0.030)
        real = rs != 0.0 and ts > self.turn_start_strength and front < 2.85
        if real:
            if (self.turn_sign != 0.0 and rs != self.turn_sign
                    and ts > self.turn_flip_strength and abs(raw_turn_y) > 0.060):
                self.turn_sign = rs; self.turn_timer = self.turn_hold_time
            elif self.turn_sign == 0.0:
                self.turn_sign = rs; self.turn_timer = self.turn_hold_time
            elif rs == self.turn_sign:
                self.turn_timer = self.turn_hold_time
            else:
                self.turn_timer = max(0.0, self.turn_timer - 2.0*self.dt)
        else:
            self.turn_timer = max(0.0, self.turn_timer - self.dt)
        if self.turn_timer <= 0.0: self.turn_sign = 0.0
        return self.turn_sign

    def compute_target(self, scan):
        left_dist, right_dist, raw_left, raw_right = self.get_side_distances(scan)
        front = self.sector_percentile(scan, -14, 14, 0.10)
        far_front = self.get_far_front(scan)
        left_open = self.sector_mean_top(scan, 22, 100)
        right_open = self.sector_mean_top(scan, -100, -22)
        min_side = min(left_dist, right_dist)
        wall_prox = self.clamp(1.0 - (min_side - 0.48)/1.10, 0.0, 1.0)
        center_y = self.clamp(0.30*(left_dist-right_dist)*wall_prox,
                              -self.center_limit, self.center_limit)
        side_push_y = self.compute_side_push(raw_left, raw_right)
        corner_strength = self.clamp(
            (self.corner_front_start-front)/max(self.corner_front_start-self.corner_front_full,1e-6),
            0.0, 1.0)
        open_diff = self.clamp(left_open-right_open, -1.15, 1.15)
        raw_turn_y = self.clamp(self.corner_bias_gain*open_diff*corner_strength,
                                -self.max_corner_bias_y, self.max_corner_bias_y)
        turn_strength = self.clamp(max(abs(raw_turn_y)/max(self.max_corner_bias_y,1e-6),
                                       0.78*corner_strength), 0.0, 1.0)
        turn_sign = self.update_turn_lock(raw_turn_y, front, turn_strength)
        if turn_sign == 0.0 and turn_strength > 0.40:
            turn_sign = self.sign_or_zero(raw_turn_y, 0.035)

        if turn_sign != 0.0 and turn_strength > 0.30:
            swd = left_dist if turn_sign > 0.0 else right_dist
            safe = self.clamp(swd - self.racing_wall_buffer, 0.0, self.racing_max_offset)
            racing_y = 0.0 if safe <= self.racing_min_offset else \
                turn_sign*self.clamp(0.42*safe, self.racing_min_offset, self.racing_max_offset)
            rb = self.clamp((turn_strength-0.25)/0.65, 0.0, 1.0)
            cry = self.clamp(center_y+side_push_y, -self.center_limit, self.center_limit)
            tr = (1.0-rb)*cry + rb*racing_y
            target_raw = self.clamp(tr, -self.racing_max_offset, self.racing_max_offset)
        else:
            racing_y = 0.0; rb = 0.0
            target_raw = self.clamp(0.70*center_y+side_push_y, -self.center_limit, self.center_limit)

        if not self.have_target:
            self.smoothed_target_y = target_raw; self.have_target = True
        else:
            alpha = 0.38 if (turn_sign != 0.0 or turn_strength > 0.35) else 0.45
            if turn_sign != 0.0 and self.smoothed_target_y*turn_sign < -0.02: alpha = 0.90
            self.smoothed_target_y = (1.0-alpha)*self.smoothed_target_y + alpha*target_raw
        target_y = self.clamp(self.smoothed_target_y, -0.24, 0.24)

        if turn_sign != 0.0 and turn_strength > self.no_opposite_target_strength:
            if target_y*turn_sign < 0.0:
                target_y = turn_sign*max(self.min_same_turn_target, 0.50*abs(racing_y)) \
                    if abs(racing_y) >= self.racing_min_offset else 0.0
                self.smoothed_target_y = target_y
            elif abs(racing_y) >= self.racing_min_offset and target_y*turn_sign < self.min_same_turn_target:
                target_y = turn_sign*self.min_same_turn_target
                self.smoothed_target_y = target_y

        return {'target_y': target_y, 'raw_turn_y': raw_turn_y, 'turn_sign': turn_sign,
                'turn_strength': turn_strength, 'front': front, 'far_front': far_front,
                'left_dist': left_dist, 'right_dist': right_dist, 'raw_left': raw_left,
                'raw_right': raw_right, 'left_open': left_open, 'right_open': right_open,
                'side_push_y': side_push_y, 'center_y': center_y, 'racing_y': racing_y,
                'race_blend': rb, 'min_side': min(raw_left, raw_right)}

    def compute_dynamic_horizon(self, ti):
        ts = ti['turn_strength']; front = ti['front']; ff = ti['far_front']; ms = ti['min_side']
        ofs = self.clamp((ff-0.90)/2.70, 0, 1); oss = self.clamp((ms-0.42)/0.48, 0, 1)
        ss = self.clamp(1.0-ts, 0, 1)
        ops = 0.50*ofs + 0.25*oss + 0.25*ss
        dd = self.min_preview_distance + ops*(self.max_preview_distance-self.min_preview_distance)
        if ts > 0.45 or ti['turn_sign'] != 0.0: dd = min(dd, 2.20-0.45*ts)
        if front < 1.10: dd = min(dd, max(0.90, front+0.70))
        ps = self.clamp(self.previous_speed+self.preview_boost, self.min_planning_speed, self.max_planning_speed)
        ht = self.clamp(dd/max(ps, 0.30), self.min_horizon_time, self.max_horizon_time)
        if self.previous_speed > self.high_speed_horizon_speed and front > 1.35:
            ht = max(ht, self.min_high_speed_horizon_time)
        if ops > 0.75 and ts < 0.25:
            ps = min(ps+0.25, self.max_planning_speed); ht = max(ht, 1.85)
        near_wall = ms < 0.42
        turn_ctx = ts > 0.42 or ti['turn_sign'] != 0.0
        if turn_ctx:
            if front > self.turn_lookahead_front_gate: ht = max(ht, self.turn_lookahead_max_time)
            elif front > 1.05: ht = max(ht, self.turn_lookahead_mid_time)
            else: ht = max(ht, self.turn_lookahead_min_time)
        if ts > 0.55 or near_wall:
            ps = max(min(ps, 1.62-0.20*ts), self.min_planning_speed)
            if self.previous_speed > self.high_speed_horizon_speed and front > 1.35:
                ht = min(max(ht, 1.42), 1.70)
            elif front > 1.10: ht = min(max(ht, self.turn_lookahead_mid_time), 1.55)
            else: ht = min(max(ht, self.turn_lookahead_min_time), 1.36)
        steps = int(self.clamp(round(ht/self.dt), 14, 48))
        ah = steps*self.dt; pd = ps*ah
        hm = 'LONG' if ah >= 1.80 else ('SHORT' if ah <= 1.18 else 'MID')
        return ps, steps, ah, pd, hm

    def simulate_rollout(self, ts, ps, steps):
        x=y=yaw=0.0; ss=self.previous_steer; path=[]
        for _ in range(steps):
            d = self.clamp(ts-ss, -self.rollout_steer_rate*self.dt, self.rollout_steer_rate*self.dt)
            ss = self.clamp(ss+d, -self.max_steer, self.max_steer)
            x += ps*math.cos(yaw)*self.dt; y += ps*math.sin(yaw)*self.dt
            yaw += (ps/self.wheelbase)*math.tan(ss)*self.dt
            path.append((x, y, yaw))
        return path

    def min_clearance_along_path(self, path, obs):
        mc = self.max_obstacle_range
        for px, py, _ in path[::2]:
            for ox, oy in obs:
                dx=px-ox; dy=py-oy; d2=dx*dx+dy*dy
                if d2 < mc*mc: mc = math.sqrt(d2)
        return mc

    def average_path_y(self, path):
        t=0.0; w=0.0; n=max(1, len(path)-1)
        for i,(_x,y,_yaw) in enumerate(path):
            wt = 1.0+0.15*(i/n); t += wt*y; w += wt
        return t/max(w, 1e-6)

    def score_rollout(self, path, steer, obs, ti, ps, pd):
        ty=ti['target_y']; tsn=ti['turn_sign']; tst=ti['turn_strength']; rty=ti['raw_turn_y']
        mc = self.min_clearance_along_path(path, obs); col = mc < self.collision_radius
        o3=path[len(path)//3][1]; mid=path[len(path)//2][1]
        fx,fy,fyaw=path[-1]; avg=self.average_path_y(path)
        ae=avg-ty; me=mid-ty; fe=fy-ty
        cost=0.0; reward=0.0; wws=0.0; ovs=0.0
        if col: cost += 18000.0
        if mc<0.42: cost += 44.0*(0.42-mc)**2/max(mc,0.03)
        if mc<0.29: cost += 220.0*(0.29-mc)**2/max(mc,0.03)
        if mc<0.18: cost += 1100.0*(0.18-mc)**2/max(mc,0.03)
        cost += (18.0+24.0*tst)*ae**2 + (12.0+14.0*tst)*me**2 + (14.0+16.0*tst)*fe**2
        if tst<0.32: cost += 18.0*avg**2 + 12.0*fy**2
        if tsn != 0.0 and tst > 0.30:
            wa=max(0.0,-(avg*tsn)+0.018); wm=max(0.0,-(mid*tsn)+0.018); we=max(0.0,-(o3*tsn)+0.012)
            wws=wa+0.70*wm+0.50*we
            cost += (650.0+480.0*tst)*wws**2
            lr=self.clamp(0.14-abs(ae),0.0,0.14); reward += (0.9+1.4*tst)*lr
            tsy=max(0.0,ty*tsn); ao=0.20+0.07*(1.0-tst)
            oa=max(0.0,avg*tsn-tsy-ao); om=max(0.0,mid*tsn-tsy-ao-0.04); of=max(0.0,fy*tsn-tsy-ao-0.12)
            ovs=oa+0.70*om+0.40*of
            cost += (240.0+230.0*tst)*ovs**2
            if steer*tsn<0.0 and abs(steer)>0.045:
                cost += (95.0+135.0*tst)*abs(steer); wws += abs(steer)
        if abs(rty)>0.020:
            dfy=self.clamp(1.50*rty,-0.42,0.42); cost += (3.0+5.0*tst)*(fyaw-dfy)**2
        mf=0.54*max(0.20,pd)
        if fx<mf: cost += 58.0*(mf-fx)**2
        if self.previous_steer*steer<0.0 and abs(self.previous_steer)>0.12 and abs(steer)>0.12:
            cost += 7.0*(abs(self.previous_steer)+abs(steer))
        cost += 5.0*(steer-self.previous_steer)**2 + 0.10*steer**2
        cost -= self.clamp(reward,0.0,0.35)
        return {'cost':cost,'reward':reward,'wrong_way_score':wws,'overshoot_score':ovs,
                'clearance':mc,'collision':col,'avg_y':avg,'lateral_offset':avg-ty,
                'path':path,'steer':steer,'final_x':fx}

    def choose_best_rollout(self, results, ti):
        tst=ti['turn_strength']; tsn=ti['turn_sign']
        if tsn==0.0 and tst<0.28:
            pl,sl,rl=self.straight_preferred_lat,self.straight_safe_lat,self.straight_relaxed_lat
        else:
            pl=self.clamp(0.44-0.14*tst,0.28,0.44); sl=self.clamp(0.56-0.17*tst,0.36,0.56)
            rl=self.clamp(0.68-0.20*tst,0.46,0.68)
        def gd(r):
            if tsn==0.0 or tst<0.30:
                return r['overshoot_score']<0.18 and abs(r['lateral_offset'])<=rl
            return r['wrong_way_score']<0.045 and r['overshoot_score']<self.max_good_overshoot
        pref=[i for i,r in enumerate(results) if (not r['collision']) and r['clearance']>=0.33
              and abs(r['lateral_offset'])<=pl and gd(r)]
        safe=[i for i,r in enumerate(results) if (not r['collision']) and r['clearance']>=0.25
              and abs(r['lateral_offset'])<=sl and gd(r)]
        rel=[i for i,r in enumerate(results) if (not r['collision']) and r['clearance']>=0.18
             and abs(r['lateral_offset'])<=rl and r['wrong_way_score']<0.12
             and r['overshoot_score']<self.max_relaxed_overshoot]
        if pref: return min(pref,key=lambda i:results[i]['cost']),'PREFERRED'
        if safe: return min(safe,key=lambda i:results[i]['cost']),'SAFE'
        if rel: return min(rel,key=lambda i:results[i]['cost']),'RELAXED'
        fb=[i for i,r in enumerate(results) if (not r['collision']) and r['clearance']>=0.15
            and r['wrong_way_score']<0.22 and r['overshoot_score']<0.42]
        if fb:
            return min(fb,key=lambda i:(results[i]['cost']+10.0*abs(results[i]['lateral_offset'])
                +16.0*results[i]['overshoot_score']+18.0*results[i]['wrong_way_score']
                -2.0*results[i]['clearance']+1.5*abs(results[i]['steer']-self.previous_steer))),'FALLBACK_LINE'
        bi=max(range(len(results)),key=lambda i:(results[i]['clearance']
            -0.72*abs(results[i]['lateral_offset'])-0.95*results[i]['wrong_way_score']
            -1.05*results[i]['overshoot_score']-0.10*abs(results[i]['steer']-self.previous_steer)))
        return bi,'MAX_CLEARANCE'

    def smooth_steering(self, ds, clr, tst, tsn=0.0, front=5.0):
        urg=self.clamp((0.36-clr)/0.24,0.0,1.0)
        ms=self.max_steer_step*(1.0+0.55*urg+0.28*tst)
        if tsn!=0.0 and tst>0.55 and front<1.85 and ds*tsn>0.035:
            mts=0.20+0.14*self.clamp((1.55-front)/0.85,0.0,1.0)
            ds=tsn*max(abs(ds),mts); ms=max(ms,0.78)
        d=self.clamp(ds-self.previous_steer,-ms,ms)
        return self.clamp(self.previous_steer+d,-self.max_steer,self.max_steer)

    def choose_speed(self, steer, best, ti, mode, hm, line_speed_cap=None, v_ref=None):
        clr=best['clearance']; col=best['collision']; tst=ti['turn_strength']
        front=ti['front']; ff=ti['far_front']; le=abs(best['lateral_offset'])
        wws=best['wrong_way_score']; ovs=best['overshoot_score']
        a=abs(steer); ts=abs(math.tan(a))
        cs=self.max_speed if ts<1e-4 else math.sqrt(self.lateral_accel_limit*self.wheelbase/ts)

        # ---- speed source (primary target) ----
        #   'curvature' (C): curvature-based feasible speed sqrt(ay_grip/kappa_ahead)
        #                    is PRIMARY -> anticipates corners/straights, real grip.
        #   'raceline'  (B): raceline's planned vx is PRIMARY (optimistic).
        #   'cap'          : reactive curvature speed primary, line only slows.
        # In all modes the reactive caps below limit the target for safety, and the
        # per-steer grip cap (cs) always applies.
        if self.speed_mode == 'curvature' and line_speed_cap is not None:
            target = min(line_speed_cap, self.max_speed)
        elif self.speed_mode == 'raceline' and v_ref is not None:
            target = min(v_ref, self.max_speed)
        else:
            target = min(self.max_speed, cs)
        # grip cap always applies (can't exceed what the steer allows)
        target = min(target, cs)
        if clr<=self.collision_radius: clsp=self.min_speed
        elif clr>=0.42: clsp=self.max_speed
        else:
            r=(clr-self.collision_radius)/max(0.42-self.collision_radius,1e-6)
            clsp=self.min_speed+r*(self.max_speed-self.min_speed)
        target=min(target,clsp)
        warn=self.clamp(1.55-0.52*tst,0.92,1.55); stop=0.34
        if ff<=stop: fsp=self.min_speed
        elif ff>=warn: fsp=self.max_speed
        else:
            r=(ff-stop)/max(warn-stop,1e-6); fsp=self.min_speed+r*(self.max_speed-self.min_speed)
        if tst>0.35 and clr>0.30 and not col: fsp=max(fsp,1.30)
        target=min(target,fsp)
        if tst>0.25:
            tc=self.clamp(self.max_speed-1.30*tst,1.35,self.max_speed); target=min(target,tc)
        fm=min(front,ff); ms=ti['min_side']
        if ms<0.36: target=min(target,1.10)
        elif ms<0.46 and self.previous_speed>1.85: target=min(target,1.55)
        elif ms<0.54 and self.previous_speed>2.30: target=min(target,2.05)
        stp=(tst>0.55 and clr>0.30 and le<0.26 and wws<0.04 and ovs<0.12 and not col)
        if fm<self.pre_turn_front_near: target=min(target,1.18 if stp else 0.90)
        elif fm<self.pre_turn_front_mid: target=min(target,1.55 if stp else 1.28)
        elif fm<self.pre_turn_front_start and self.previous_speed>1.90: target=min(target,2.05 if stp else 1.82)
        if le>0.70: target=min(target,1.15)
        elif le>0.55: target=min(target,1.55)
        elif le>0.42: target=min(target,1.95)
        if wws>0.12: target=min(target,0.95)
        elif wws>0.05: target=min(target,1.35)
        if ovs>0.28: target=min(target,1.05)
        elif ovs>0.16: target=min(target,1.35)
        if mode=='MAX_CLEARANCE': target=min(target,0.82 if clr<0.20 else 1.45)
        if hm=='SHORT' and clr<0.26: target=min(target,1.15)
        if col: target=min(target,self.min_speed)

        # ---- GLOBAL-LINE speed anticipation (only ever LOWERS speed) ----
        if line_speed_cap is not None:
            target = min(target, line_speed_cap)

        emer=col or clr<0.18 or front<0.32

        # ---- EMA smoothing of the target speed (reduces jumpy cap-switching) ----
        # Emergencies BYPASS the filter so hard braking is never delayed.
        if emer:
            self.smoothed_target_speed = target          # snap to (low) target now
        else:
            if self.smoothed_target_speed is None:
                self.smoothed_target_speed = target
            else:
                a_ema = self.speed_ema_alpha
                self.smoothed_target_speed = (1.0 - a_ema) * self.smoothed_target_speed + a_ema * target
            target = self.smoothed_target_speed

        if target>self.previous_speed:
            sf=(hm=='LONG' and tst<0.20 and a<0.10 and le<0.22 and clr>0.44 and front>2.00)
            st=self.straight_accel_step if sf else self.max_accel_step
            st*=self.clamp(1.0-0.10*tst,0.78,1.0)
            sp=min(target,self.previous_speed+st)
        else:
            ds=self.emergency_decel_step if emer else self.max_decel_step
            sp=max(target,self.previous_speed-ds)
        return self.clamp(sp,self.min_speed,self.max_speed)

    def publish_command(self, steer, speed):
        c=AckermannDrive(); c.steering_angle=float(steer); c.speed=float(speed)
        self.cmd_pub.publish(c)

    def publish_raceline(self):
        """Draw the full raceline CSV as a blue LINE_STRIP in the MAP frame, so it
        overlays the saved map / track in RViz."""
        if not self.have_line:
            return
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'raceline'; m.id = 0; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.04
        m.color.r = 0.1; m.color.g = 0.4; m.color.b = 1.0; m.color.a = 0.9
        m.pose.orientation.w = 1.0
        for i in range(self.N):
            p = Point(); p.x = float(self.rl_x[i]); p.y = float(self.rl_y[i]); p.z = 0.02
            m.points.append(p)
        # close the loop
        p = Point(); p.x = float(self.rl_x[0]); p.y = float(self.rl_y[0]); p.z = 0.02
        m.points.append(p)
        self.raceline_pub.publish(m)

    def publish_line_marker(self, scan, line_y):
        m=Marker(); m.header.frame_id=scan.header.frame_id; m.header.stamp=scan.header.stamp
        m.ns='line_target'; m.id=0; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.x=float(self.line_lookahead_m); m.pose.position.y=float(line_y); m.pose.position.z=0.05
        m.scale.x=m.scale.y=m.scale.z=0.12
        m.color.b=1.0; m.color.g=0.4; m.color.a=0.9
        self.line_marker_pub.publish(m)

    def publish_visualization(self, scan, results, best_index):
        # candidate rollouts as a MarkerArray (best = green/thick, others faint)
        ma = MarkerArray()
        clr = Marker(); clr.header.frame_id = scan.header.frame_id
        clr.header.stamp = scan.header.stamp; clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        for i, r in enumerate(results):
            if i % 2 != 0 and i != best_index:
                continue
            m = Marker(); m.header.frame_id = scan.header.frame_id; m.header.stamp = scan.header.stamp
            m.ns = 'candidate_rollouts'; m.id = i; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            if i == best_index:
                m.scale.x = 0.07; m.color.g = 1.0; m.color.a = 1.0
            elif r['collision']:
                m.scale.x = 0.018; m.color.r = 1.0; m.color.a = 0.14
            else:
                m.scale.x = 0.018; m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 0.12
            for x, y, _yaw in r['path']:
                p = Point(); p.x = float(x); p.y = float(y); p.z = 0.05
                m.points.append(p)
            ma.markers.append(m)
        self.marker_pub.publish(ma)

        # best path as nav_msgs/Path
        path = Path(); path.header.frame_id = scan.header.frame_id; path.header.stamp = scan.header.stamp
        for x, y, yaw in results[best_index]['path']:
            ps = PoseStamped(); ps.header.frame_id = scan.header.frame_id; ps.header.stamp = scan.header.stamp
            ps.pose.position.x = float(x); ps.pose.position.y = float(y); ps.pose.position.z = 0.05
            ps.pose.orientation.z = math.sin(yaw/2.0); ps.pose.orientation.w = math.cos(yaw/2.0)
            path.poses.append(ps)
        self.best_path_pub.publish(path)

    # ==========================================================
    # main callback
    # ==========================================================
    def scan_callback(self, scan):
        self.callback_count += 1
        obstacles = self.scan_to_obstacle_points(scan)
        target_info = self.compute_target(scan)

        # ---- GLOBAL-LINE GRAFT: soft steering bias + speed (cap or raceline) ----
        line_speed_cap = None
        v_ref = None
        if self.have_line and self.line_bias > 0.0:
            line_y, v_curv, v_ref, ok = self.global_line_query()
            if ok:
                # soft blend: reactive target nudged toward the global line
                blended = (1.0 - self.line_bias) * target_info['target_y'] + self.line_bias * line_y
                target_info['target_y'] = self.clamp(blended, -0.30, 0.30)
                line_speed_cap = v_curv
                if self.callback_count % 2 == 0:
                    self.publish_line_marker(scan, line_y)
        elif self.have_line and self.use_line_speed:
            # bias off but still want speed (cap and/or raceline primary)
            _, v_curv, v_ref, ok = self.global_line_query()
            if ok: line_speed_cap = v_curv

        ps, steps, ht, pd, hm = self.compute_dynamic_horizon(target_info)
        results = []
        for sc in self.make_steering_samples():
            path = self.simulate_rollout(sc, ps, steps)
            results.append(self.score_rollout(path, sc, obstacles, target_info, ps, pd))
        bi, mode = self.choose_best_rollout(results, target_info)
        best = results[bi]
        desired = best['steer']

        if (target_info['turn_sign'] != 0.0 and target_info['turn_strength'] > 0.62
                and target_info['front'] < 1.55 and desired*target_info['turn_sign'] >= -0.02):
            mes = 0.20+0.14*self.clamp((1.45-target_info['front'])/0.90,0.0,1.0)
            desired = target_info['turn_sign']*max(abs(desired), mes)

        steer = self.smooth_steering(desired, best['clearance'], target_info['turn_strength'],
                                     target_info['turn_sign'], target_info['front'])
        speed = self.choose_speed(steer, best, target_info, mode, hm, line_speed_cap, v_ref)

        esc = (best['clearance'] < self.escape_clearance_threshold
               or target_info['front'] < self.escape_front_threshold
               or (best['collision'] and best['clearance'] < 0.10))
        if esc:
            if target_info['turn_sign'] != 0.0: es=target_info['turn_sign']
            elif abs(desired)>0.05: es=self.sign_or_zero(desired,0.01)
            else: es=1.0 if target_info['left_dist']>target_info['right_dist'] else -1.0
            steer=self.clamp(0.42*es,-self.max_steer,self.max_steer); speed=self.escape_speed

        self.publish_command(steer, speed)
        self.previous_steer=steer; self.previous_speed=speed; self.previous_desired_steer=desired

        # publish rollouts + best path viz (every other cycle to keep CPU light)
        if self.callback_count % 2 == 0:
            self.publish_visualization(scan, results, bi)

        if self.callback_count % 8 == 0:
            lc = f'{line_speed_cap:.2f}' if line_speed_cap is not None else 'off'
            vr = f'{v_ref:.2f}' if v_ref is not None else 'off'
            self.get_logger().info(
                f'spd_mode={self.speed_mode} mode={mode} hm={hm} steer={steer:.2f} '
                f'speed={speed:.2f} v_curv={lc} v_ref={vr} '
                f'target_y={target_info["target_y"]:.3f} turn_sign={target_info["turn_sign"]:.0f} '
                f'front={target_info["front"]:.2f} clr={best["clearance"]:.2f} bias={self.line_bias}')


def main(args=None):
    rclpy.init(args=args)
    node = ReactivePlusLinePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
