#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose


class NavMonitor(Node):

    def __init__(self):
        super().__init__('nav_monitor')

        self.robot_x = None
        self.robot_y = None

        self.distance_remaining = None

        self.status = "IDLE"

        self.goal_active = False
        self.goal_finished = False

        # ---------------------------------------------------------
        # Robot odometry
        # ---------------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            '/bumperbot_controller/odom',
            self.odom_callback,
            10
        )

        # ---------------------------------------------------------
        # Nav2 action feedback
        # ---------------------------------------------------------

        self.feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )

        # ---------------------------------------------------------
        # Nav2 action status
        # ---------------------------------------------------------

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )

        # ---------------------------------------------------------
        # Display timer
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.5,
            self.monitor_callback
        )

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "       NAVIGATION MONITOR STARTED"
        )
        self.get_logger().info(
            "========================================"
        )

    # =============================================================
    # ODOMETRY
    # =============================================================

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # =============================================================
    # NAV2 FEEDBACK
    # =============================================================

    def feedback_callback(self, msg):

        if self.goal_finished:
            return

        feedback = msg.feedback

        # Nav2 directly provides this value.
        self.distance_remaining = feedback.distance_remaining

        self.goal_active = True
        self.status = "NAVIGATING"

    # =============================================================
    # NAV2 STATUS
    # =============================================================

    def status_callback(self, msg):

        if not msg.status_list:
            return

        latest = msg.status_list[-1]

        status = latest.status

        # ---------------------------------------------------------
        # ACCEPTED
        # ---------------------------------------------------------

        if status == 1:

            self.goal_active = True
            self.goal_finished = False
            self.status = "NAVIGATING"

        # ---------------------------------------------------------
        # EXECUTING
        # ---------------------------------------------------------

        elif status == 2:

            self.goal_active = True
            self.goal_finished = False
            self.status = "NAVIGATING"

        # ---------------------------------------------------------
        # CANCELING
        # ---------------------------------------------------------

        elif status == 3:

            self.status = "NAVIGATING"

        # ---------------------------------------------------------
        # SUCCEEDED
        # ---------------------------------------------------------

        elif status == 4:

            self.goal_active = False
            self.goal_finished = True

            self.status = "SUCCEEDED"
            self.distance_remaining = 0.0

            self.get_logger().info(
                "========================================"
            )
            self.get_logger().info(
                "          NAVIGATION SUCCEEDED"
            )
            self.get_logger().info(
                "========================================"
            )

        # ---------------------------------------------------------
        # CANCELED
        # ---------------------------------------------------------

        elif status == 5:

            self.goal_active = False
            self.goal_finished = True

            self.status = "IDLE"
            self.distance_remaining = None

        # ---------------------------------------------------------
        # ABORTED
        # ---------------------------------------------------------

        elif status == 6:

            self.goal_active = False
            self.goal_finished = True

            self.status = "IDLE"
            self.distance_remaining = None

    # =============================================================
    # DISPLAY
    # =============================================================

    def monitor_callback(self):

        print("\033[2J\033[H", end="")

        print("========================================")
        print("          NAVIGATION MONITOR")
        print("========================================")

        # ---------------------------------------------------------
        # Current robot pose
        # ---------------------------------------------------------

        if self.robot_x is not None:

            print(
                f"Current Pose: "
                f"({self.robot_x:.2f}, {self.robot_y:.2f})"
            )

        else:

            print("Current Pose: --")

        # ---------------------------------------------------------
        # Remaining distance
        # ---------------------------------------------------------

        if self.distance_remaining is not None:

            print(
                f"Remaining Distance: "
                f"{self.distance_remaining:.2f} m"
            )

        else:

            print("Remaining Distance: --")

        # ---------------------------------------------------------
        # Status
        # ---------------------------------------------------------

        print(f"Status: {self.status}")

        print("========================================")


def main(args=None):

    rclpy.init(args=args)

    node = NavMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
