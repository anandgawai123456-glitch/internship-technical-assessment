#!/usr/bin/env python3

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


class ConePoseTest(Node):

    def __init__(self):
        super().__init__('xarm6_cone_pose_test')

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

    def get_robot_state(self):

        joint_map = dict(
            zip(
                self.joint_state.name,
                self.joint_state.position
            )
        )

        state = RobotState()

        state.joint_state.name = JOINTS
        state.joint_state.position = [
            joint_map[j]
            for j in JOINTS
        ]

        return state

    def test_ik(self, name, x, y, z, qx, qy, qz, qw):

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
        request.ik_request.robot_state = self.get_robot_state()

        request.ik_request.avoid_collisions = False

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
            f'{name}: MoveIt error code = '
            f'{response.error_code.val}'
        )

        if response.error_code.val == 1:

            solution = dict(
                zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position
                )
            )

            print('  IK SOLUTION:')

            for joint in JOINTS:
                print(
                    f'    {joint}: '
                    f'{solution[joint]:.6f} rad'
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
            print('No /joint_states received')
            return

        print()
        print('===== CONE POSE DIAGNOSTIC =====')
        print()

        # ------------------------------------------------------
        # Test 1
        #
        # Same orientation that MoveIt returned from FK.
        # Only position is changed to our cone point.
        # ------------------------------------------------------

        self.test_ik(
            'TEST 1 - known FK orientation',
            0.100,
            0.000,
            0.160,
            1.0,
            0.0,
            0.0,
            0.0
        )

        print()

        # ------------------------------------------------------
        # Test 2
        #
        # Orientation generated from the cone surface normal.
        #
        # Normal:
        # nx = 0.936329
        # ny = 0
        # nz = 0.351123
        # ------------------------------------------------------

        nx = 0.9363291775690445
        ny = 0.0
        nz = 0.35112344158839176

        # Construct a frame with local Z = surface normal.

        z_axis = [nx, ny, nz]

        reference = [0.0, 0.0, 1.0]

        dot = sum(
            z_axis[i] * reference[i]
            for i in range(3)
        )

        if abs(dot) > 0.95:
            reference = [0.0, 1.0, 0.0]

        x_axis = [
            reference[1] * z_axis[2]
            - reference[2] * z_axis[1],

            reference[2] * z_axis[0]
            - reference[0] * z_axis[2],

            reference[0] * z_axis[1]
            - reference[1] * z_axis[0]
        ]

        x_norm = math.sqrt(
            sum(v * v for v in x_axis)
        )

        x_axis = [
            v / x_norm
            for v in x_axis
        ]

        y_axis = [
            z_axis[1] * x_axis[2]
            - z_axis[2] * x_axis[1],

            z_axis[2] * x_axis[0]
            - z_axis[0] * x_axis[2],

            z_axis[0] * x_axis[1]
            - z_axis[1] * x_axis[0]
        ]

        matrix = [
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]]
        ]

        trace = (
            matrix[0][0]
            + matrix[1][1]
            + matrix[2][2]
        )

        if trace > 0:

            s = math.sqrt(trace + 1.0) * 2

            qw = 0.25 * s
            qx = (
                matrix[2][1] - matrix[1][2]
            ) / s
            qy = (
                matrix[0][2] - matrix[2][0]
            ) / s
            qz = (
                matrix[1][0] - matrix[0][1]
            ) / s

        else:

            # Fallback conversion.
            qw = 1.0
            qx = 0.0
            qy = 0.0
            qz = 0.0

        self.test_ik(
            'TEST 2 - surface-normal orientation',
            0.100,
            0.000,
            0.160,
            qx,
            qy,
            qz,
            qw
        )

        print()
        print('===== DIAGNOSTIC COMPLETE =====')


def main():

    rclpy.init()

    node = ConePoseTest()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
