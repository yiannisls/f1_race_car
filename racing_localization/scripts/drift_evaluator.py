#!/usr/bin/env python3
"""
drift_evaluator.py
==================
Turns the localization drift test into MEASURED NUMBERS.

Subscribes:
  /yellow_car/ground_truth   nav_msgs/Odometry   (raw Webots world frame)
  /odometry/filtered         nav_msgs/Odometry   (EKF output, odom frame)

What it does:
  1. ALIGNMENT: GT is in the Webots world frame; the EKF starts at (0,0,0) in
     odom. These differ by a fixed rigid transform (rotation + translation).
     We estimate that transform once, from the first ALIGN_SAMPLES matched
     pairs, using a 2D Umeyama (Kabsch) fit. After that, GT is mapped into the
     odom frame so the two are directly comparable.
  2. METRICS (continuous):
       - instantaneous position error  ||p_ekf - p_gt||      [m]
       - instantaneous heading error   wrap(yaw_ekf - yaw_gt) [deg]
       - running ATE (RMSE of position error over the run)   [m]
       - path length travelled (from GT)                     [m]
       - return-to-start drift: error vs the START pose when the car comes
         back near its starting point (closes a lap)
  3. OUTPUTS:
       - prints a live one-line summary at ~2 Hz
       - logs every sample to CSV (--csv, default /tmp/drift_log.csv)
       - republishes ALIGNED GT on /yellow_car/ground_truth_aligned (odom frame)
         so you can overlay it on the EKF path in RViz
       - prints a FINAL REPORT on Ctrl-C

Run:
  ros2 run racing_localization drift_evaluator
  # or: python3 drift_evaluator.py --ros-args -p csv:=/tmp/drift_log.csv
"""

import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def umeyama_2d(src, dst):
    """Least-squares similarity (here rigid, scale fixed to 1) mapping src->dst.
    src, dst: (N,2). Returns (R 2x2, t 2,)."""
    src = np.asarray(src); dst = np.asarray(dst)
    mu_s = src.mean(axis=0); mu_d = dst.mean(axis=0)
    S = src - mu_s; D = dst - mu_d
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:           # reflection guard
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    return R, t


class DriftEvaluator(Node):
    def __init__(self):
        super().__init__("drift_evaluator")
        self.align_samples = self.declare_parameter("align_samples", 40).value
        self.csv_path = self.declare_parameter("csv", "/tmp/drift_log.csv").value
        self.return_radius = self.declare_parameter("return_radius_m", 0.5).value

        self.gt_sub = Subscriber(self, Odometry, "/yellow_car/ground_truth")
        self.ekf_sub = Subscriber(self, Odometry, "/odometry/filtered")
        self.sync = ApproximateTimeSynchronizer(
            [self.gt_sub, self.ekf_sub], queue_size=50, slop=0.05)
        self.sync.registerCallback(self.cb)

        self.aligned_pub = self.create_publisher(
            Odometry, "/yellow_car/ground_truth_aligned", 10)

        # alignment state
        self.R = None; self.t = None
        self.gt_buf = []; self.ekf_buf = []

        # metrics state
        self.errors = []           # position errors after alignment
        self.start_gt = None       # aligned GT start pose (x,y,yaw)
        self.path_len = 0.0
        self.last_gt_xy = None
        self.moved_away = False    # set once car leaves return_radius
        self.return_drift = None   # best-known return-to-start error
        self.n = 0

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t", "gt_x", "gt_y", "gt_yaw_deg",
                           "ekf_x", "ekf_y", "ekf_yaw_deg",
                           "pos_err_m", "yaw_err_deg", "ate_m"])

        self.create_timer(0.5, self.print_live)
        self.get_logger().info(
            f"drift_evaluator up. Aligning on first {self.align_samples} pairs. "
            f"CSV -> {self.csv_path}")

    def cb(self, gt: Odometry, ekf: Odometry):
        gx, gy = gt.pose.pose.position.x, gt.pose.pose.position.y
        gyaw = yaw_of(gt.pose.pose.orientation)
        ex, ey = ekf.pose.pose.position.x, ekf.pose.pose.position.y
        eyaw = yaw_of(ekf.pose.pose.orientation)

        # --- phase 1: collect for alignment ---
        if self.R is None:
            self.gt_buf.append([gx, gy]); self.ekf_buf.append([ex, ey])
            if len(self.gt_buf) >= self.align_samples:
                # need some motion for a stable fit
                spread = np.ptp(np.asarray(self.ekf_buf), axis=0).max()
                if spread < 0.2:
                    self.gt_buf.pop(0); self.ekf_buf.pop(0)  # slide until moving
                    return
                self.R, self.t = umeyama_2d(self.gt_buf, self.ekf_buf)
                yaw_align = math.atan2(self.R[1, 0], self.R[0, 0])
                self.yaw_offset = yaw_align
                self.get_logger().info(
                    f"Alignment locked. rot={math.degrees(yaw_align):.1f} deg, "
                    f"t=({self.t[0]:.2f},{self.t[1]:.2f}).")
            return

        # --- phase 2: map GT into odom frame and score ---
        p = self.R @ np.array([gx, gy]) + self.t
        gax, gay = float(p[0]), float(p[1])
        gayaw = wrap(gyaw + self.yaw_offset)

        pos_err = math.hypot(ex - gax, ey - gay)
        yaw_err = math.degrees(abs(wrap(eyaw - gayaw)))
        self.errors.append(pos_err)
        ate = float(np.sqrt(np.mean(np.square(self.errors))))
        self.n += 1
        self.last_pos_err = pos_err; self.last_yaw_err = yaw_err; self.last_ate = ate

        # path length + return-to-start tracking (on aligned GT)
        if self.start_gt is None:
            self.start_gt = (gax, gay, gayaw)
        if self.last_gt_xy is not None:
            self.path_len += math.hypot(gax - self.last_gt_xy[0],
                                        gay - self.last_gt_xy[1])
        self.last_gt_xy = (gax, gay)

        d_start = math.hypot(gax - self.start_gt[0], gay - self.start_gt[1])
        if d_start > self.return_radius:
            self.moved_away = True
        elif self.moved_away:   # came back near start after leaving -> lap closed
            self.return_drift = pos_err

        # republish aligned GT for RViz overlay
        a = Odometry()
        a.header = ekf.header
        a.pose.pose.position.x = gax; a.pose.pose.position.y = gay
        a.pose.pose.orientation.z = math.sin(gayaw / 2.0)
        a.pose.pose.orientation.w = math.cos(gayaw / 2.0)
        self.aligned_pub.publish(a)

        t = ekf.header.stamp.sec + ekf.header.stamp.nanosec * 1e-9
        self.csv.writerow([f"{t:.3f}", f"{gax:.4f}", f"{gay:.4f}",
                           f"{math.degrees(gayaw):.2f}", f"{ex:.4f}", f"{ey:.4f}",
                           f"{math.degrees(eyaw):.2f}", f"{pos_err:.4f}",
                           f"{yaw_err:.3f}", f"{ate:.4f}"])

    def print_live(self):
        if self.R is None or self.n == 0:
            return
        rd = f"{self.return_drift:.3f} m" if self.return_drift is not None else "—"
        self.get_logger().info(
            f"pos_err={self.last_pos_err:.3f} m | yaw_err={self.last_yaw_err:.2f}° "
            f"| ATE={self.last_ate:.3f} m | path={self.path_len:.1f} m "
            f"| return_drift={rd}")

    def final_report(self):
        if not self.errors:
            self.get_logger().warn("No data scored — was GT publishing? Check DEF name.")
            return
        e = np.asarray(self.errors)
        print("\n================ DRIFT REPORT ================")
        print(f" samples scored      : {len(e)}")
        print(f" path length (GT)    : {self.path_len:.2f} m")
        print(f" ATE (RMSE)          : {np.sqrt(np.mean(e**2)):.3f} m")
        print(f" mean position error : {e.mean():.3f} m")
        print(f" max position error  : {e.max():.3f} m")
        if self.return_drift is not None:
            print(f" return-to-start drift: {self.return_drift:.3f} m   "
                  f"({100*self.return_drift/max(self.path_len,1e-6):.2f}% of path)")
        else:
            print(" return-to-start drift: (no full loop detected)")
        print(f" CSV log             : {self.csv_path}")
        print("=============================================")
        print(" PASS (slow sim lap): ATE < ~0.30 m and heading drift < ~5°.")


def main():
    rclpy.init()
    node = DriftEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.final_report()
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
