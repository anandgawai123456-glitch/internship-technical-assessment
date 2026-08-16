#!/usr/bin/env python3

import csv
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


# ============================================================
# xArm6 configuration
# ============================================================

JOINTS = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]

INPUT_FILE = 'cone_cartesian_path.csv'
OUTPUT_FILE = 'cone_ik_test.csv'

# First validation pass.
# Change to None later to process all points.
MAX_POINTS = None


# ============================================================
# Known-good TCP orientation
# ============================================================
#
# This orientation was experimentally verified using MoveIt:
#
#   position = (0.360, 0.000, 0.160)
#
#   collision OFF -> SUCCESS
#   collision ON  -> SUCCESS
#
# The surface-normal orientation failed with error -31.
#
# Therefore we use this valid fixed TCP orientation for the
# entire Cartesian cone path.
#
# Quaternion:
#
#   x = 1
#   y = 0
#   z = 0
#   w = 0
#
# ============================================================

TCP_QX = 1.0
TCP_QY = 0.0
TCP_QZ = 0.0
TCP_QW = 0.0


class ConeIKSolver(Node):

    def __init__(self):

        super().__init__(
            'xarm6_cone_ik_solver'
        )

        self.joint_state = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

    # --------------------------------------------------------
    # Joint-state callback
    # --------------------------------------------------------

    def joint_state_callback(self, msg):

        self.joint_state = msg

    # --------------------------------------------------------
    # Wait for MoveIt and joint states
    # --------------------------------------------------------

    def wait_for_setup(self):

        self.get_logger().info(
            'Waiting for /compute_ik...'
        )

        if not self.ik_client.wait_for_service(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/compute_ik unavailable'
            )
            return False

        self.get_logger().info(
            'Waiting for /joint_states...'
        )

        for _ in range(100):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.joint_state is not None:
                return True

        self.get_logger().error(
            'No joint state received'
        )

        return False

    # --------------------------------------------------------
    # Create initial IK seed from actual robot state
    # --------------------------------------------------------

    def make_seed_state(self):

        joint_map = dict(
            zip(
                self.joint_state.name,
                self.joint_state.position
            )
        )

        missing = [
            joint
            for joint in JOINTS
            if joint not in joint_map
        ]

        if missing:

            raise RuntimeError(
                f'Missing joints: {missing}'
            )

        state = RobotState()

        state.joint_state.name = JOINTS

        state.joint_state.position = [
            joint_map[joint]
            for joint in JOINTS
        ]

        return state

    # --------------------------------------------------------
    # Solve one Cartesian point
    # --------------------------------------------------------

    def solve_point(
        self,
        point,
        seed_state
    ):

        pose = PoseStamped()

        # MoveIt's planning frame.
        pose.header.frame_id = 'world'

        # ----------------------------------------------------
        # Cartesian position
        # ----------------------------------------------------

        pose.pose.position.x = float(
            point['x']
        )

        pose.pose.position.y = float(
            point['y']
        )

        pose.pose.position.z = float(
            point['z']
        )

        # ----------------------------------------------------
        # Known-good fixed TCP orientation
        # ----------------------------------------------------

        pose.pose.orientation.x = TCP_QX
        pose.pose.orientation.y = TCP_QY
        pose.pose.orientation.z = TCP_QZ
        pose.pose.orientation.w = TCP_QW

        # ----------------------------------------------------
        # MoveIt IK request
        # ----------------------------------------------------

        request = GetPositionIK.Request()

        request.ik_request.group_name = 'xarm6'

        request.ik_request.ik_link_name = 'link_eef'

        request.ik_request.pose_stamped = pose

        # Seed IK with previous solution.
        request.ik_request.robot_state = seed_state

        # Keep collision checking enabled.
        request.ik_request.avoid_collisions = True

        # ----------------------------------------------------
        # Call MoveIt
        # ----------------------------------------------------

        future = self.ik_client.call_async(
            request
        )

        while (
            rclpy.ok()
            and not future.done()
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

        response = future.result()

        if response is None:

            return None, 'service_failed'

        # MoveIt SUCCESS = 1
        if response.error_code.val != 1:

            return None, str(
                response.error_code.val
            )

        # ----------------------------------------------------
        # Extract joint solution
        # ----------------------------------------------------

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position
            )
        )

        missing = [
            joint
            for joint in JOINTS
            if joint not in solution
        ]

        if missing:

            return None, (
                f'missing_joint:{missing}'
            )

        values = [
            solution[joint]
            for joint in JOINTS
        ]

        # ----------------------------------------------------
        # Return solved point
        # ----------------------------------------------------

        return {
            'x': float(point['x']),
            'y': float(point['y']),
            'z': float(point['z']),

            'qx': TCP_QX,
            'qy': TCP_QY,
            'qz': TCP_QZ,
            'qw': TCP_QW,

            'joints': values

        }, 'success'

    # --------------------------------------------------------
    # Main processing
    # --------------------------------------------------------

    def run(self):

        if not self.wait_for_setup():
            return False

        # ----------------------------------------------------
        # Initial seed = actual robot state
        # ----------------------------------------------------

        seed_state = self.make_seed_state()

        # ----------------------------------------------------
        # Read Cartesian path
        # ----------------------------------------------------

        try:

            with open(
                INPUT_FILE,
                newline=''
            ) as f:

                reader = csv.DictReader(f)

                points = list(reader)

        except FileNotFoundError:

            self.get_logger().error(
                f'Input file not found: '
                f'{INPUT_FILE}'
            )

            return False

        # ----------------------------------------------------
        # Limit points for first validation
        # ----------------------------------------------------

        if MAX_POINTS is not None:

            points = points[
                :MAX_POINTS
            ]

        self.get_logger().info(
            f'Testing IK on '
            f'{len(points)} cone points'
        )

        # ----------------------------------------------------
        # Solve all points
        # ----------------------------------------------------

        results = []

        failures = 0

        for i, point in enumerate(points):

            result, status = self.solve_point(
                point,
                seed_state
            )

            if result is None:

                failures += 1

                self.get_logger().error(
                    f'Point {i}: '
                    f'IK FAILED '
                    f'(status={status})'
                )

                # Do not advance the seed when IK fails.
                continue

            joints = result['joints']

            self.get_logger().info(
                f'Point {i}: IK OK | '
                f'J1={joints[0]: .4f} '
                f'J2={joints[1]: .4f} '
                f'J3={joints[2]: .4f} '
                f'J4={joints[3]: .4f} '
                f'J5={joints[4]: .4f} '
                f'J6={joints[5]: .4f}'
            )

            results.append(result)

            # ------------------------------------------------
            # IMPORTANT:
            #
            # Use the current solution as the seed for the
            # next Cartesian point.
            #
            # This encourages continuous joint motion.
            # ------------------------------------------------

            seed_state = RobotState()

            seed_state.joint_state.name = JOINTS

            seed_state.joint_state.position = joints

        # ----------------------------------------------------
        # Write results
        # ----------------------------------------------------

        with open(
            OUTPUT_FILE,
            'w',
            newline=''
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                'index',
                'x',
                'y',
                'z',
                'qx',
                'qy',
                'qz',
                'qw',
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'joint5',
                'joint6'
            ])

            for i, result in enumerate(results):

                writer.writerow([
                    i,

                    f"{result['x']:.6f}",
                    f"{result['y']:.6f}",
                    f"{result['z']:.6f}",

                    f"{result['qx']:.8f}",
                    f"{result['qy']:.8f}",
                    f"{result['qz']:.8f}",
                    f"{result['qw']:.8f}",

                    *[
                        f"{joint:.8f}"
                        for joint in result['joints']
                    ]
                ])

        # ----------------------------------------------------
        # Summary
        # ----------------------------------------------------

        print()

        print(
            '===== CONE IK TEST ====='
        )

        print(
            f'Input points : {len(points)}'
        )

        print(
            f'IK success   : {len(results)}'
        )

        print(
            f'IK failures  : {failures}'
        )

        print(
            f'Output       : {OUTPUT_FILE}'
        )

        print()

        if failures == 0:

            print(
                '===== ALL IK TEST POINTS PASSED ====='
            )

            return True

        print(
            '===== IK TEST HAS FAILURES ====='
        )

        return False


def main():

    rclpy.init()

    node = ConeIKSolver()

    try:

        success = node.run()

    except Exception as exc:

        node.get_logger().error(
            f'Exception: {exc}'
        )

        success = False

    finally:

        node.destroy_node()

        rclpy.shutdown()

    sys.exit(
        0 if success else 1
    )


if __name__ == '__main__':
    main()
