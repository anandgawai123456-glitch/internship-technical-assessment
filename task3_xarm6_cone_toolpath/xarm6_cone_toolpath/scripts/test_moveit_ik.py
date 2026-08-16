#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK


class MoveItIKTest(Node):

    def __init__(self):
        super().__init__('xarm6_ik_test')

        self.joint_state = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def wait_for_services(self):
        self.get_logger().info('Waiting for /compute_fk...')
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/compute_fk not available')
            return False

        self.get_logger().info('Waiting for /compute_ik...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/compute_ik not available')
            return False

        return True

    def wait_for_joint_state(self):
        self.get_logger().info('Waiting for /joint_states...')

        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.joint_state is not None:
                return True

        self.get_logger().error('No /joint_states received')
        return False

    def run_test(self):

        if not self.wait_for_services():
            return False

        if not self.wait_for_joint_state():
            return False

        # ---------------------------------------------------------
        # Extract the six xArm6 joints
        # ---------------------------------------------------------

        required_joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        joint_map = dict(
            zip(
                self.joint_state.name,
                self.joint_state.position
            )
        )

        missing = [
            j for j in required_joints
            if j not in joint_map
        ]

        if missing:
            self.get_logger().error(
                f'Missing joints: {missing}'
            )
            return False

        robot_state = RobotState()

        robot_state.joint_state.name = required_joints
        robot_state.joint_state.position = [
            joint_map[j] for j in required_joints
        ]

        self.get_logger().info(
            'Current xArm6 joints:'
        )

        for name in required_joints:
            self.get_logger().info(
                f'  {name}: {joint_map[name]:.6f} rad'
            )

        # ---------------------------------------------------------
        # STEP 1: Forward Kinematics
        # ---------------------------------------------------------

        fk_request = GetPositionFK.Request()

        fk_request.header.frame_id = 'world'
        fk_request.fk_link_names = ['link_eef']
        fk_request.robot_state = robot_state

        self.get_logger().info(
            'Calling MoveIt FK for link_eef...'
        )

        fk_future = self.fk_client.call_async(fk_request)

        while rclpy.ok() and not fk_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        fk_response = fk_future.result()

        if fk_response is None:
            self.get_logger().error('FK call failed')
            return False

        self.get_logger().info(
            f'FK error code: '
            f'{fk_response.error_code.val}'
        )

        if not fk_response.pose_stamped:
            self.get_logger().error(
                'FK returned no pose'
            )
            return False

        tcp_pose = fk_response.pose_stamped[0]

        self.get_logger().info(
            'Current TCP pose from MoveIt:'
        )

        self.get_logger().info(
            f'  frame: {tcp_pose.header.frame_id}'
        )

        self.get_logger().info(
            f'  position: '
            f'x={tcp_pose.pose.position.x:.6f}, '
            f'y={tcp_pose.pose.position.y:.6f}, '
            f'z={tcp_pose.pose.position.z:.6f}'
        )

        self.get_logger().info(
            f'  orientation: '
            f'x={tcp_pose.pose.orientation.x:.6f}, '
            f'y={tcp_pose.pose.orientation.y:.6f}, '
            f'z={tcp_pose.pose.orientation.z:.6f}, '
            f'w={tcp_pose.pose.orientation.w:.6f}'
        )

        # ---------------------------------------------------------
        # STEP 2: Inverse Kinematics
        #
        # Ask MoveIt to recover joint coordinates that achieve
        # the exact TCP pose returned by FK.
        # ---------------------------------------------------------

        ik_request = GetPositionIK.Request()

        ik_request.ik_request.group_name = 'xarm6'
        ik_request.ik_request.ik_link_name = 'link_eef'

        ik_request.ik_request.pose_stamped = tcp_pose

        ik_request.ik_request.robot_state = robot_state

        ik_request.ik_request.avoid_collisions = True

        self.get_logger().info(
            'Calling MoveIt IK for the FK-generated TCP pose...'
        )

        ik_future = self.ik_client.call_async(ik_request)

        while rclpy.ok() and not ik_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        ik_response = ik_future.result()

        if ik_response is None:
            self.get_logger().error('IK call failed')
            return False

        self.get_logger().info(
            f'IK error code: '
            f'{ik_response.error_code.val}'
        )

        if ik_response.error_code.val != 1:
            self.get_logger().error(
                'MoveIt did not find a valid IK solution.'
            )
            return False

        # ---------------------------------------------------------
        # Print resulting six joint coordinates
        # ---------------------------------------------------------

        solution = dict(
            zip(
                ik_response.solution.joint_state.name,
                ik_response.solution.joint_state.position
            )
        )

        self.get_logger().info(
            '===== IK SOLUTION ====='
        )

        for joint in required_joints:
            if joint in solution:
                self.get_logger().info(
                    f'  {joint}: '
                    f'{solution[joint]:.6f} rad'
                )
            else:
                self.get_logger().warn(
                    f'{joint} not present in IK solution'
                )

        self.get_logger().info(
            '===== IK TEST PASSED ====='
        )

        self.get_logger().info(
            'No trajectory was sent to the robot.'
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = MoveItIKTest()

    try:
        success = node.run_test()
    except Exception as exc:
        node.get_logger().error(
            f'Exception: {exc}'
        )
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
