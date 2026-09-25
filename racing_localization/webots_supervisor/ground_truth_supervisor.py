#!/usr/bin/env python3
"""
ground_truth_supervisor.py
==========================
Publishes the TRUE world-frame pose of the car from Webots, for benchmarking the
EKF (/odometry/filtered) against reality. Read-only: it never moves anything.

WHY A SEPARATE SUPERVISOR?
Your car runs an extern webots_ros2 driver ("controller_violet"), which is NOT a
Supervisor and therefore cannot read true pose. The clean fix is a tiny extra
Supervisor robot in the world whose ONLY job is to read the car's pose by DEF
name and publish it. This does not touch your existing car driver.

TWO WAYS TO RUN THIS (pick one):

  (A) As a webots_ros2 driver plugin (recommended, matches your launch setup)
      - Add a Supervisor robot to the .wbt (see ground_truth.urdf + README)
      - webots_ros2_driver loads this class via the <plugin> tag.
      - `init(self, webots_node, properties)` is the entry point; webots_node.robot
        is a Supervisor instance because the Robot node has supervisor TRUE.

  (B) As a classic standalone Webots controller
      - Set the Supervisor robot's controller field to this file.
      - The `__main__` block at the bottom handles that case.

OUTPUT:
  /yellow_car/ground_truth   nav_msgs/Odometry   (RAW Webots world frame)
      - frame_id = "world", child_frame_id = "base_link"
      - pose = true position + yaw (Webots axis convention handled below)
      - twist = true linear/angular velocity from node.getVelocity()

The drift_evaluator node aligns this raw world pose to the EKF's odom frame, so
you do NOT need to zero it here.

WEBOTS AXIS NOTE (ENU world, default in modern Webots):
  position = [x, y, z]   -> we use x, y directly (z is up)
  getVelocity() = [vx, vy, vz, wx, wy, wz] in world frame
  yaw is extracted from the 3x3 orientation matrix (getOrientation()).
If your world uses the legacy NUE axis system, see the AXIS_SYSTEM flag below.
"""

import math
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ---- CONFIG ---------------------------------------------------------------
# DEF name of the car in the .wbt. If getFromDef fails the node prints every
# candidate it tried plus instructions. Order = priority.
CAR_DEF_CANDIDATES = ['YELLOW_CAR', 'yellow_car', 'TT02_2025a_camera', 'yellow_car"TT02_2025a_camera"']
AXIS_SYSTEM = "ENU"   # "ENU" (z-up, modern default) or "NUE" (y-up, legacy)
PUBLISH_TOPIC = "/yellow_car/ground_truth"


def yaw_from_orientation(R, axis_system):
    """Extract planar yaw from Webots 3x3 row-major orientation matrix R (len 9)."""
    if axis_system == "ENU":
        # heading in the world XY plane: forward axis is column 0 (x-axis of robot)
        # R = [r0 r1 r2; r3 r4 r5; r6 r7 r8], robot x-axis world = (R[0],R[3],R[6])
        return math.atan2(R[3], R[0])
    else:  # NUE legacy (y up). Heading in world XZ plane.
        return math.atan2(R[2], R[0])


class GroundTruthPublisher:
    """Shared logic used by both the driver-plugin and standalone entry points."""

    def __init__(self, supervisor, ros_node):
        self.supervisor = supervisor
        self.node = ros_node
        self.timestep = int(supervisor.getBasicTimeStep())
        self.car = None

        for name in CAR_DEF_CANDIDATES:
            n = supervisor.getFromDef(name)
            if n is not None:
                self.car = n
                self.node.get_logger().info(
                    f"[ground_truth] Found car via DEF '{name}'.")
                break

        if self.car is None:
            self.node.get_logger().error(
                "[ground_truth] Could NOT find the car node by DEF name.\n"
                f"  Tried: {CAR_DEF_CANDIDATES}\n"
                "  FIX: open your .wbt, find the car Robot node, and give it a DEF, e.g.:\n"
                "       DEF YELLOW_CAR Robot { ... }\n"
                "  Then add the name to CAR_DEF_CANDIDATES if different.\n"
                "  (Also ensure THIS supervisor robot has 'supervisor TRUE'.)")

        self.pub = self.node.create_publisher(Odometry, PUBLISH_TOPIC, 10)
        self.child_frame = "base_link"

    def step(self):
        if self.car is None:
            return
        pos = self.car.getPosition()          # [x, y, z] world
        R = self.car.getOrientation()         # 9-element row-major matrix
        vel = self.car.getVelocity()          # [vx,vy,vz, wx,wy,wz] world

        if AXIS_SYSTEM == "ENU":
            x, y = pos[0], pos[1]
            vx, vy = vel[0], vel[1]
            wz = vel[5]
        else:  # NUE
            x, y = pos[0], pos[2]
            vx, vy = vel[0], vel[2]
            wz = vel[4]

        yaw = yaw_from_orientation(R, AXIS_SYSTEM)

        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.angular.z = float(wz)
        # Ground truth is exact: tiny covariance so it can be a hard reference.
        for i in (0, 7, 35):
            msg.pose.covariance[i] = 1e-6
        self.pub.publish(msg)


# ===========================================================================
# (A) webots_ros2 driver-plugin entry point
# ===========================================================================
class GroundTruthDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot          # Supervisor (supervisor TRUE)
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros_node = rclpy.create_node("ground_truth_supervisor")
        self.gt = GroundTruthPublisher(self.robot, self.ros_node)

    def step(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.gt.step()


# ===========================================================================
# (B) standalone Webots controller entry point
# ===========================================================================
if __name__ == "__main__":
    from controller import Supervisor
    supervisor = Supervisor()
    rclpy.init(args=sys.argv)
    ros_node = rclpy.create_node("ground_truth_supervisor")
    gt = GroundTruthPublisher(supervisor, ros_node)
    ts = int(supervisor.getBasicTimeStep())
    while supervisor.step(ts) != -1:
        rclpy.spin_once(ros_node, timeout_sec=0)
        gt.step()
    rclpy.shutdown()
