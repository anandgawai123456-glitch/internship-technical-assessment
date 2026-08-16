#!/usr/bin/env python3

import csv
import sys

import rclpy
from pathlib import Path
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory


CSV_FILE = str(Path(__file__).resolve().parent.parent / 'cone_toolpath.csv')

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]


class ConeTrajectoryExecutor(Node):

    def __init__(self):
        super().__init__('cone_trajectory_executor')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm6_traj_controller/follow_joint_trajectory'
        )

    def load_trajectory(self):
        trajectory = []

        with open(CSV_FILE, newline='') as f:
            reader = csv.DictReader(f)

            required = [
                'time_from_start',
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'joint5',
                'joint6'
            ]

            for name in required:
                if name not in reader.fieldnames:
                    raise RuntimeError(
                        f'Missing CSV column: {name}'
                    )

            for row in reader:

                t = float(row['time_from_start'])

                sec = int(t)
                nanosec = int(
                    (t - sec) * 1_000_000_000
                )

                from trajectory_msgs.msg import JointTrajectoryPoint

                point = JointTrajectoryPoint()

                point.positions = [
                    float(row[name])
                    for name in JOINT_NAMES
                ]

                point.time_from_start.sec = sec
                point.time_from_start.nanosec = nanosec

                trajectory.append(point)

        return trajectory

    def execute(self):

        self.get_logger().info(
            'Waiting for xArm6 trajectory controller...'
        )

        if not self.client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                'Trajectory action server not available.'
            )
            return False

        points = self.load_trajectory()

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = points

        self.get_logger().info(
            f'Loaded {len(points)} trajectory points.'
        )

        self.get_logger().info(
            f'Executing trajectory from {CSV_FILE}'
        )

        future = self.client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                'Trajectory goal was rejected.'
            )

            return False

        self.get_logger().info(
            'Trajectory goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        if result.error_code == 0:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'CONE TOOLPATH EXECUTION SUCCESSFUL'
            )

            self.get_logger().info(
                '========================================'
            )

            return True

        self.get_logger().error(
            f'Trajectory execution failed. '
            f'error_code={result.error_code}'
        )

        self.get_logger().error(
            f'error_string="{result.error_string}"'
        )

        return False


def main():

    rclpy.init()

    node = ConeTrajectoryExecutor()

    try:
        success = node.execute()

    except Exception as exc:

        node.get_logger().error(
            f'Execution error: {exc}'
        )

        success = False

    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
