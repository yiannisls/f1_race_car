#!/usr/bin/env python3
"""
Localization bring-up for the F1/10 Webots racing stack (Step 1: drift-free odometry).

Brings up:
  1. wheel odometry source   -> /yellow_car/odom_wheel  (twist: vx, no TF)
  2. rf2o laser odometry      -> /yellow_car/odom_rf2o   (twist: vx,vy,vyaw, no TF)
  3. robot_localization EKF   -> /odometry/filtered + odom->base_link TF (single owner)
  4. static TF base_link->laser (REPLACE with your real URDF extrinsics, or
     remove if your robot_state_publisher/URDF already provides it)

Run:
  ros2 launch racing_localization localization.launch.py

Then validate (see VALIDATION.md):
  ros2 run tf2_tools view_frames
  ros2 topic echo /odometry/filtered
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg = FindPackageShare("racing_localization")

    ekf_cfg  = PathJoinSubstitution([pkg, "config", "ekf.yaml"])
    rf2o_cfg = PathJoinSubstitution([pkg, "config", "rf2o.yaml"])

    # Wheel odom is OFF by default: the sim's /yellow_car/current_speed is dead,
    # so rf2o is the sole motion source. Flip to "true" once real wheel speed
    # exists AND you've re-enabled odom0's vx slot in ekf.yaml.
    enable_wheel_odom = LaunchConfiguration("enable_wheel_odom")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_wheel_odom", default_value="true"),

        # --- (4) Static transform: base_link -> laser ---------------------------
        # REPLACE the xyz/rpy with your real LiDAR mount from the URDF.
        # Your occupancy_grid_list assumes the lidar is ~0.21 m forward of base_link.
        # If robot_state_publisher already publishes this from the URDF, DELETE this node.
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="base_to_laser",
        #     arguments=["0.21", "0.0", "0.0", "0.0", "0.0", "0.0",
        #                "base_link", "laser_link"],
        #     parameters=[{"use_sim_time": use_sim_time}],
        # ),

        # --- (1) Wheel odometry source (OFF by default; no measured speed) ------
        Node(
            package="racing_localization",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            condition=IfCondition(enable_wheel_odom),
            parameters=[{
                "use_sim_time": use_sim_time,
                "wheelbase": 0.257,
                "publish_rate_hz": 50.0,
            }],
        ),

        # --- (2) rf2o laser odometry -------------------------------------------
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[rf2o_cfg,{"publish_tf": False, "use_sim_time": use_sim_time}],
        ),

        # Joint odometry node (wheel + rf2o) for twist-only output to EKF. This node is
        Node(
            package="racing_localization",
            executable="joint_odometry_node",
            name="joint_odometry_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                
                "wheel_radius": 0.04,
                "wheelbase": 0.257,
                "publish_pose": False,   # EKF integrates pose; node sends twist only
            }],
        ),
        # # --- (3) robot_localization EKF (owns odom->base_link TF) --------------
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_cfg, {"use_sim_time": use_sim_time}],
        ),
    ])
