#!/usr/bin/env python3

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


class WorkspaceProbe(Node):

    def __init__(self):

        super().__init__('xarm6_workspace_probe')

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

    def robot_state(self):

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

    def test(self, x, y, z):

        pose = PoseStamped()

        pose.header.frame_id = 'world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Known-good FK orientation.
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        request = GetPositionIK.Request()

        request.ik_request.group_name = 'xarm6'
        request.ik_request.ik_link_name = 'link_eef'

        request.ik_request.pose_stamped = pose

        request.ik_request.robot_state = self.robot_state()

        request.ik_request.avoid_collisions = False

        future = self.client.call_async(request)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

        response = future.result()

        if response is None:
            return False

        return response.error_code.val == 1

    def run(self):

        if not self.client.wait_for_service(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/compute_ik unavailable'
            )
            return

        for _ in range(100):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.joint_state is not None:
                break

        if self.joint_state is None:
            self.get_logger().error(
                'No joint state'
            )
            return

        print()
        print('===== XARM6 WORKSPACE PROBE =====')
        print()

        # --------------------------------------------------
        # Probe points around the known valid FK pose.
        # --------------------------------------------------

        points = [

            # Known valid reference.
            (0.207, 0.000, 0.112),

            # Same x, varying z.
            (0.207, 0.000, 0.150),
            (0.207, 0.000, 0.200),
            (0.207, 0.000, 0.250),
            (0.207, 0.000, 0.300),

            # Larger x.
            (0.250, 0.000, 0.160),
            (0.300, 0.000, 0.160),
            (0.350, 0.000, 0.160),
            (0.400, 0.000, 0.160),

            # Smaller x.
            (0.180, 0.000, 0.160),
            (0.150, 0.000, 0.160),

            # Y offsets.
            (0.207, 0.050, 0.160),
            (0.207, 0.100, 0.160),
            (0.207, 0.150, 0.160),

            # Higher positions.
            (0.250, 0.000, 0.250),
            (0.300, 0.000, 0.250),
            (0.350, 0.000, 0.250),
        ]

        success = 0

        for x, y, z in points:

            ok = self.test(x, y, z)

            status = 'REACHABLE' if ok else 'NO IK'

            print(
                f'x={x: .3f} '
                f'y={y: .3f} '
                f'z={z: .3f} '
                f'-> {status}'
            )

            if ok:
                success += 1

        print()
        print(
            f'Reachable: {success}/{len(points)}'
        )
        print()
        print('===== PROBE COMPLETE =====')


def main():

    rclpy.init()

    node = WorkspaceProbe()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
