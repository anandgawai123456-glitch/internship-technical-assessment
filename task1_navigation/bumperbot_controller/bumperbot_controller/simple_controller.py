```python
#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        # Robot parameters
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter(
            "wheel_radius"
        ).get_parameter_value().double_value

        self.wheel_separation_ = self.get_parameter(
            "wheel_separation"
        ).get_parameter_value().double_value

        self.get_logger().info(
            f"Using wheel radius: {self.wheel_radius_}"
        )
        self.get_logger().info(
            f"Using wheel separation: {self.wheel_separation_}"
        )

        # Odometry state
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.prev_time_ = None

        # ============================================================
        # DIRECT VELOCITY PATH
        #
        # /cmd_vel
        #      ↓
        # this node
        #      ↓
        # /wheel_velocity_controller/commands
        # ============================================================

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10
        )

        self.vel_sub_ = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.velCallback,
            10
        )

        self.joint_sub_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.jointCallback,
            10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry,
            "/bumperbot_controller/odom",
            10
        )

        # Differential-drive kinematics
        #
        # [v]   [r/2   r/2 ] [wl]
        # [w] = [r/L  -r/L ] [wr]
        #
        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2.0,
             self.wheel_radius_ / 2.0],

            [self.wheel_radius_ / self.wheel_separation_,
             -self.wheel_radius_ / self.wheel_separation_]
        ])

        self.get_logger().info(
            f"Conversion matrix:\n{self.speed_conversion_}"
        )

        # Odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        self.odom_msg_.pose.pose.orientation.w = 1.0

        # TF broadcaster
        self.br_ = TransformBroadcaster(self)

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.get_logger().info(
            "SimpleController ready: /cmd_vel -> "
            "/wheel_velocity_controller/commands"
        )

    # ================================================================
    # CMD_VEL CALLBACK
    # ================================================================

    def velCallback(self, msg):

        # Desired robot velocity
        v = msg.linear.x
        w = msg.angular.z

        robot_speed = np.array([
            [v],
            [w]
        ])

        # Calculate wheel velocities
        #
        # inverse matrix gives:
        # wheel_speed[0] = left wheel
        # wheel_speed[1] = right wheel
        #
        wheel_speed = np.matmul(
            np.linalg.inv(self.speed_conversion_),
            robot_speed
        )

        left_speed = wheel_speed[0, 0]
        right_speed = wheel_speed[1, 0]

        # IMPORTANT:
        # Joint order is:
        # wheel_left_joint
        # wheel_right_joint
        #
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [
            left_speed,
            right_speed
        ]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    # ================================================================
    # JOINT STATES -> ODOMETRY
    # ================================================================

    def jointCallback(self, msg):

        # Need both wheel positions
        if len(msg.position) < 2:
            return

        current_time = Time.from_msg(msg.header.stamp)

        # Initialize using the first joint-state message.
        # This prevents a bad first dt.
        if self.prev_time_ is None:
            self.left_wheel_prev_pos_ = msg.position[0]
            self.right_wheel_prev_pos_ = msg.position[1]
            self.prev_time_ = current_time
            return

        dt = current_time - self.prev_time_

        if dt.nanoseconds <= 0:
            return

        # Wheel position change
        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_

        # Save current state
        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = current_time

        # Wheel angular velocities
        dt_sec = dt.nanoseconds / S_TO_NS

        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec

        # Robot linear/angular velocity
        linear = (
            self.wheel_radius_ * fi_right
            + self.wheel_radius_ * fi_left
        ) / 2.0

        angular = (
            self.wheel_radius_ * fi_right
            - self.wheel_radius_ * fi_left
        ) / self.wheel_separation_

        # Robot displacement
        d_s = (
            self.wheel_radius_ * dp_right
            + self.wheel_radius_ * dp_left
        ) / 2.0

        d_theta = (
            self.wheel_radius_ * dp_right
            - self.wheel_radius_ * dp_left
        ) / self.wheel_separation_

        self.theta_ += d_theta

        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Quaternion
        q = quaternion_from_euler(
            0.0,
            0.0,
            self.theta_
        )

        # ============================================================
        # ODOMETRY
        # ============================================================

        self.odom_msg_.header.stamp = msg.header.stamp

        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        # ============================================================
        # TF: odom -> base_footprint
        # ============================================================

        self.transform_stamped_.header.stamp = msg.header.stamp

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.br_.sendTransform(self.transform_stamped_)


def main():

    rclpy.init()

    node = SimpleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

