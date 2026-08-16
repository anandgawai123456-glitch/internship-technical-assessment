#!/usr/bin/env python3

import csv
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


JOINTS = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]


class Diagnostic(Node):

    def __init__(self):

        super().__init__('xarm6_cone_ik_diagnostic')

        self.joint_state = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

    def joint_callback(self, msg):
        self.joint_state = msg

    def get_state(self):

        joint_map = dict(
            zip(
                self.joint_state.name,
                self.joint_state.position
            )
        )

        state = RobotState()

        state.joint_state.name = JOINTS
        state.joint_state.position = [
            joint_map[j] for j in JOINTS
        ]

        return state

    def ik_test(
        self,
        name,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        collision
    ):

        pose = PoseStamped()

        pose.header.frame_id = 'world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request = GetPositionIK.Request()

        request.ik_request.group_name = 'xarm6'
        request.ik_request.ik_link_name = 'link_eef'
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = self.get_state()
        request.ik_request.avoid_collisions = collision

        future = self.client.call_async(request)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

        response = future.result()

        if response is None:
            print(f'{name}: SERVICE FAILURE')
            return

        print(
            f'{name}: '
            f'code={response.error_code.val}'
        )

        if response.error_code.val == 1:

            solution = dict(
                zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position
                )
            )

            print('  solution:')

            for joint in JOINTS:
                print(
                    f'    {joint}: '
                    f'{solution[joint]:.6f}'
                )

    def run(self):

        if not self.client.wait_for_service(
            timeout_sec=10.0
        ):
            print('/compute_ik unavailable')
            return

        for _ in range(100):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.joint_state is not None:
                break

        if self.joint_state is None:
            print('No joint states')
            return

        # ------------------------------------------------------
        # Read the actual generated first point.
        # ------------------------------------------------------

        with open(
            'cone_cartesian_path.csv',
            newline=''
        ) as f:

            point = next(csv.DictReader(f))

        x = float(point['x'])
        y = float(point['y'])
        z = float(point['z'])

        nx = float(point['nx'])
        ny = float(point['ny'])
        nz = float(point['nz'])

        print()
        print('===== ACTUAL CONE POINT =====')
        print(
            f'x={x:.6f}, '
            f'y={y:.6f}, '
            f'z={z:.6f}'
        )

        print(
            f'normal='
            f'({nx:.6f}, {ny:.6f}, {nz:.6f})'
        )

        print()

        # ------------------------------------------------------
        # Test 1:
        # Known FK orientation, collisions OFF
        # ------------------------------------------------------

        self.ik_test(
            'TEST 1 | known orientation | collision OFF',
            x, y, z,
            1.0, 0.0, 0.0, 0.0,
            False
        )

        # ------------------------------------------------------
        # Test 2:
        # Known FK orientation, collisions ON
        # ------------------------------------------------------

        self.ik_test(
            'TEST 2 | known orientation | collision ON',
            x, y, z,
            1.0, 0.0, 0.0, 0.0,
            True
        )

        # ------------------------------------------------------
        # Build a quaternion whose local Z axis follows
        # the surface normal.
        # ------------------------------------------------------

        z_axis = [nx, ny, nz]

        reference = [0.0, 0.0, 1.0]

        dot = sum(
            z_axis[i] * reference[i]
            for i in range(3)
        )

        if abs(dot) > 0.95:
            reference = [0.0, 1.0, 0.0]

        # x = reference cross z
        x_axis = [
            reference[1] * z_axis[2]
            - reference[2] * z_axis[1],

            reference[2] * z_axis[0]
            - reference[0] * z_axis[2],

            reference[0] * z_axis[1]
            - reference[1] * z_axis[0]
        ]

        norm = math.sqrt(
            sum(v * v for v in x_axis)
        )

        x_axis = [
            v / norm for v in x_axis
        ]

        # y = z cross x
        y_axis = [
            z_axis[1] * x_axis[2]
            - z_axis[2] * x_axis[1],

            z_axis[2] * x_axis[0]
            - z_axis[0] * x_axis[2],

            z_axis[0] * x_axis[1]
            - z_axis[1] * x_axis[0]
        ]

        R = [
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]]
        ]

        trace = (
            R[0][0]
            + R[1][1]
            + R[2][2]
        )

        if trace > 0:

            s = math.sqrt(trace + 1.0) * 2.0

            qw = 0.25 * s
            qx = (R[2][1] - R[1][2]) / s
            qy = (R[0][2] - R[2][0]) / s
            qz = (R[1][0] - R[0][1]) / s

        else:

            qw = 1.0
            qx = 0.0
            qy = 0.0
            qz = 0.0

        # ------------------------------------------------------
        # Test 3:
        # Surface orientation, collisions OFF
        # ------------------------------------------------------

        self.ik_test(
            'TEST 3 | surface orientation | collision OFF',
            x, y, z,
            qx, qy, qz, qw,
            False
        )

        print()
        print('===== DIAGNOSTIC COMPLETE =====')


def main():

    rclpy.init()

    node = Diagnostic()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
