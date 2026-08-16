#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_tranceformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt, atan2, sqrt, pi, fabs
import random
import time



def angle_diff(a, b):
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2



class OdometryMotionModel(Node):

    def __init__(self):
        super().__init__("odometry_motion_model")
        self.is_first_odom = True
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0  
        self.last_odom_theta = 0.0

        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("nr_samples", 300)

        self.alpha1_ = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2_ = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3_ = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4_ = self.get_parameter("alpha4").get_parameter_value().double_value
        self.nr_samples_ = self.get_parameter("nr_samples").get_parameter_value().integer_value

        if self.nr_samples_ >= 0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples_)]
        else:
            self.get_logger().fatal("Invalid number of samples requested: %d",  self.nr_samples_)
            return

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        self.pose_array_pub_ = self.create_publisher(PoseArray, "Odometry_motion_model/samples", 10)
  


    def odomCallback(self, odom):
        q = [odom.pose.pose.oriantention.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w] 
        roll, pitch, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.samples.header.frame_id = odom.header
            self.is_first_odom= False
            return


        odom_x_increament = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increament = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increament = angle_diff(yaw, self.last_odom_theta)    

        if sqrt(pow(odom_y_increament, 2) + pow(odom_x_increament, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
           delta_rot1 = angle_diff(atan2(odom_y_increament, odom_x_increament), yaw)
        delta_transl = sqrt(pow(odom_y_increament, 2) + pow(odom_x_increament, 2))
        delta_rot2 = angle_diff(odom_theta_increament, delta_rot1)

        rot1_var = self.alpha1_ * delta_rot1  + self.alpha2_ * delta_transl
        trasl_var = self.alpha3_ * delta_transl  + self.alpha4  * (delta_rot1 + delta_rot2)
        rot2_var = self.alpha1_ * delta_rot2  + self.alpha2 * (delta_rot1 + delta_rot2)

        random.seed(time.time())
        for sample in self.samples.poses:
            rot1_noise = random.gauss(0, rot1_var)
            transl_noise = random.gauss(0, trasl_var)
            rot2_noise = random.gauss(0, rot2_var)

            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_transl_draw = delta_transl - transl_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)


            sample_q =  [self.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w]
            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion(sample_q)
            sample.position.x += delta_transl_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_transl_draw * sin(sample_yaw + delta_rot1_draw)
            q = quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)[3]
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw
        self.pose_array_pub_.publish(self.samples)
        
def main():
    rclpy.init()

    Odometry_motion_model = OdometryMotionModel()
    rclpy.spin(Odometry_motion_model)
    
    Odometry_motion_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()