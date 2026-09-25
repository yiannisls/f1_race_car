from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_yaml",      default_value="/home/turtle/maps/new_track_map_clean.yaml"),

        # nav2_map_server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            }],
        ),

        # nav2_amcl
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "global_frame_id": "map",
                "scan_topic": "/yellow_car/scan",
                "max_particles": 5000,
                "min_particles": 1000,
                "update_min_d": 0.05,
                "update_min_a": 0.05,
                "laser_max_range": 12.0,
                "laser_min_range": 0.15,
                "max_beams": 180,
                "alpha1": 0.05,
                "alpha2": 0.05,
                "alpha3": 0.1,
                "alpha4": 0.05,
                "laser_model_type": "likelihood_field",
                "z_hit": 0.8,
                "z_rand": 0.2,
                "sigma_hit": 0.08,
                "resample_interval": 1,
                "transform_tolerance": 0.5,
                "recovery_alpha_slow": 0.001,
                "recovery_alpha_fast": 0.1,
            }],
        ),
    ])