import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    assignment_navigation_dir = get_package_share_directory(
        'assignment_navigation'
    )

    bumperbot_bringup_dir = get_package_share_directory(
        'bumperbot_bringup'
    )

    nav2_bringup_dir = get_package_share_directory(
        'nav2_bringup'
    )

    # ---------------------------------------------------------
    # Map
    # ---------------------------------------------------------

    map_file = os.path.join(
        assignment_navigation_dir,
        'maps',
        'walls_only.yaml'
    )

    # ---------------------------------------------------------
    # Nav2 parameters
    # ---------------------------------------------------------

    params_file = os.path.join(
        assignment_navigation_dir,
        'config',
        'nav2_params.yaml'
    )

    # ---------------------------------------------------------
    # Start Gazebo + BumperBot
    #
    # IMPORTANT:
    # Use an OPEN Gazebo world instead of small_house.
    # The bumperbot_description package provides empty.world.
    # ---------------------------------------------------------

    simulated_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bumperbot_bringup_dir,
                'launch',
                'simulated_robot.launch.py'
            )
        ),
        launch_arguments={
            'world_name': 'empty',
        }.items()
    )

    # ---------------------------------------------------------
    # Nav2
    # ---------------------------------------------------------

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True',
        }.items()
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------

    rviz_config = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Launch order
    # ---------------------------------------------------------

    return LaunchDescription([
        simulated_robot,
        nav2,
        rviz,
    ])
