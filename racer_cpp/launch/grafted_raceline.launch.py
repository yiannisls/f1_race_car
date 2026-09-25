import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('racer_cpp')
    default_csv = os.path.join(pkg_share, 'data', 'raceline.csv')

    return LaunchDescription([
        DeclareLaunchArgument('raceline_csv', default_value=default_csv),
        DeclareLaunchArgument('line_bias', default_value='0.15'),
        DeclareLaunchArgument('ay_grip', default_value='2.5'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_topic', default_value='/drive'),

        Node(
            package='racer_cpp',
            executable='global_mppi.py',
            name='mppi_global_racer',
            output='screen',
            parameters=[{
                'raceline_csv': LaunchConfiguration('raceline_csv'),
                'line_bias': LaunchConfiguration('line_bias'),
                'ay_grip': LaunchConfiguration('ay_grip'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
            }]
        )
    ])
