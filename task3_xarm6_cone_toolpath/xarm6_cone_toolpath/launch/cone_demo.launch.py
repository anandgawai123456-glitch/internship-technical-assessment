#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Start xArm6 Gazebo + MoveIt simulation
    xarm6_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'xarm6_moveit_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'hw_ns': 'xarm',
        }.items()
    )

    # Give Gazebo, controllers and MoveIt time to initialize,
    # then execute the generated cone trajectory.
    execute_toolpath = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution([
                        FindPackageShare('xarm6_cone_toolpath'),
                        'scripts',
                        'execute_cone_trajectory.py'
                    ])
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        xarm6_sim,
        execute_toolpath,
    ])
