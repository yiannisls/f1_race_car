from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_topic', default_value='/drive'),

        Node(
            package='racer_cpp',
            executable='mppi_car.py',
            name='mppi_reactive_racer',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
            }]
        )
    ])