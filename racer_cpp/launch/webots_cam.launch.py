import os
import launch
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    # 1. Grab the locked URDF file from the organizers
    rosconfr_dir = get_package_share_directory('webots_rosconfr')
    robot_description_path = os.path.join(rosconfr_dir, 'resource', 'TT02_jaune_cam.urdf')
    with open(robot_description_path, 'r') as file:
        robot_description = file.read()

    # 2. Point DIRECTLY to your custom world on your Desktop!
    # (Make sure this path perfectly matches where you saved it)
    webots_world_path = os.path.join(
        get_package_share_directory('webots_rosconfr'),
        'worlds',
        'Piste_CoVAPSy_2025a_camera.wbt'   # verify exact filename below
    )

    webots = WebotsLauncher(
        world=webots_world_path,
        mode='realtime',
        gui=True,
    )

    # 3. The driver for the Yellow Car
    webots_bridge = WebotsController(
        robot_name='yellow_car',
        parameters=[{'robot_description': robot_description_path}],
        remappings=[('__node', 'webots_bridge')],
        respawn=True,
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},
        ],
        output='screen',
    )

    # 5. YOUR Ground Truth Supervisor! (Using the safe Standalone Node method)
    supervisor_driver = Node(
        package='racing_localization',
        executable='ground_truth_supervisor.py',
        name='ground_truth_supervisor',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'ipc://1234/ground_truth_robot'}
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        webots,
        webots_bridge,
        robot_state_publisher,
        supervisor_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])