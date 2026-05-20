#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient


class StretchArucoPickup(Node):
    def __init__(self):
        super().__init__('stretch_aruco_pickup', namespace='/')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Base velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        # Trajectory controller (arm + head)
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

        self.marker_frame = 'column_1'
        self.timer = self.create_timer(0.1, self.loop)

        # State machine:
        self.state = 'ARM_IN'
        self.arm_retracted = False
        self.base_aligned = False

        # Pivot timing
        self.pivot_start_time = None
        self.pivot_duration = 3.0  # seconds for 90° rotation

        # Head reset flag
        self.initial_head_reset_done = False

        # TF freshness guard (ALIGN_BASE only)
        self.last_x = None
        self.stale_count = 0

    # -----------------------------------------------------------
    # HEAD CONTROL
    # -----------------------------------------------------------
    def send_head_pan(self, angle):
        if not self.traj_client.wait_for_server(timeout_sec=0.01):
            return

        traj = JointTrajectory()
        traj.joint_names = ["joint_head_pan"]

        p = JointTrajectoryPoint()
        p.positions = [angle]
        p.time_from_start.sec = 1

        traj.points = [p]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal)

    def send_head_tilt(self, angle):
        if not self.traj_client.wait_for_server(timeout_sec=0.01):
            return

        traj = JointTrajectory()
        traj.joint_names = ["joint_head_tilt"]

        p = JointTrajectoryPoint()
        p.positions = [angle]
        p.time_from_start.sec = 1

        traj.points = [p]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal)

    # -----------------------------------------------------------
    # ARM RETRACT
    # -----------------------------------------------------------
    def retract_arm(self):
        if self.arm_retracted:
            return

        if not self.traj_client.wait_for_server(timeout_sec=0.1):
            return

        self.get_logger().info("Retracting arm...")

        joint_names = [
            "joint_lift",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_gripper_finger_left",
        ]

        p = JointTrajectoryPoint()
        p.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        p.time_from_start.sec = 3

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points = [p]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self.traj_client.send_goal_async(goal)
        future.add_done_callback(self.retract_done)

    def retract_done(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Arm retract rejected")
            return
        handle.get_result_async().add_done_callback(self.retract_result)

    def retract_result(self, future):
        self.get_logger().info("Arm fully retracted.")
        self.arm_retracted = True
        self.state = 'ALIGN_BASE'

    # -----------------------------------------------------------
    # BASE ALIGNMENT
    # -----------------------------------------------------------
    def align_base(self, x, y):
        twist = Twist()

        # Distance safety
        if x < 0.35:
            self.get_logger().info("Reached grasp distance.")
            self.base_aligned = True
            self.cmd_vel_pub.publish(Twist())
            return

        # Angle to marker
        theta = math.atan2(y, x)
        ang_tol = 0.20
        ang_k = 1.0

        # Distance control
        desired = 0.50
        lin_err = x - desired
        lin_tol = 0.10
        lin_k = 0.4

        if abs(theta) > ang_tol:
            twist.angular.z = ang_k * theta
        elif abs(lin_err) > lin_tol:
            twist.linear.x = lin_k * lin_err
        else:
            self.get_logger().info("Base aligned for pivot.")
            self.base_aligned = True
            self.cmd_vel_pub.publish(Twist())
            return

        twist.angular.z = max(min(twist.angular.z, 0.4), -0.4)
        twist.linear.x = max(min(twist.linear.x, 0.20), -0.20)
        self.cmd_vel_pub.publish(twist)

    # -----------------------------------------------------------
    # FIXED 90° BASE ROTATION
    # -----------------------------------------------------------
    def pivot_base_90(self):
        twist = Twist()

        if self.pivot_start_time is None:
            self.pivot_start_time = self.get_clock().now()
            self.get_logger().info("Starting 90° base pivot...")

        elapsed = (self.get_clock().now() - self.pivot_start_time).nanoseconds / 1e9

        if elapsed < self.pivot_duration:
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            return False
        else:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Base pivot complete.")
            return True

    # -----------------------------------------------------------
    # FIXED -90° HEAD ROTATION
    # -----------------------------------------------------------
    def pivot_head_back(self):
        self.send_head_pan(-math.pi/2)
        self.get_logger().info("Head rotated back toward marker.")
        return True

    # -----------------------------------------------------------
    # MAIN LOOP
    # -----------------------------------------------------------
    def loop(self):

        # Reset head at startup
        if not self.initial_head_reset_done:
            if self.traj_client.wait_for_server(timeout_sec=0.1):
                self.send_head_pan(0.0)
                self.send_head_tilt(-0.3)   # constant tilt
                self.initial_head_reset_done = True
                self.get_logger().info("Head reset to forward, tilt -0.3.")
            return

        if self.state == 'DONE':
            return

        if self.state == 'ARM_IN':
            self.retract_arm()
            return

        # TF lookup (only needed for ALIGN_BASE)
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                self.marker_frame,
                RclTime()
            )
        except TransformException:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        # -------------------------------------------------------
        # TF Freshness Guard (ALIGN_BASE ONLY)
        # -------------------------------------------------------
        if self.state == 'ALIGN_BASE':
            if self.last_x is not None:
                if abs(x - self.last_x) < 0.002:
                    self.stale_count += 1
                else:
                    self.stale_count = 0

            self.last_x = x

            if self.stale_count > 3:
                self.get_logger().warn("Marker lost — stopping base.")
                self.cmd_vel_pub.publish(Twist())
                return

        # -------------------------------------------------------
        # State Machine
        # -------------------------------------------------------
        if self.state == 'ALIGN_BASE':
            self.align_base(x, y)
            if self.base_aligned:
                self.state = 'PIVOT_BASE_90'

        elif self.state == 'PIVOT_BASE_90':
            if self.pivot_base_90():
                self.state = 'PIVOT_HEAD_BACK'

        elif self.state == 'PIVOT_HEAD_BACK':
            if self.pivot_head_back():
                self.state = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = StretchArucoPickup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

