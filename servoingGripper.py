#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime

import tf2_ros
from tf2_ros import TransformException

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient


class PreServoArmPosition(Node):
    def __init__(self):
        super().__init__('pre_servo_arm_position')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Trajectory client
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

        self.target_frame = 'column_1'
        self.gripper_frame = 'link_aruco_inner_wrist'

        self.timer = self.create_timer(0.1, self.loop)

        self.phase = 'LIFT'
        self.arm_extended = False

        self.get_logger().info("Pre-servo arm positioning node started.")

    # -----------------------------------------------------------
    # SEND TRAJECTORY
    # -----------------------------------------------------------
    def send_traj(self, joint_names, positions, duration=2.0):
        if not self.traj_client.wait_for_server(timeout_sec=0.1):
            return

        traj = JointTrajectory()
        traj.joint_names = joint_names

        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = int(duration)

        traj.points = [p]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal)

    # -----------------------------------------------------------
    # MAIN LOOP
    # -----------------------------------------------------------
    def loop(self):
        # Try to get target marker TF
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                self.target_frame,
                RclTime()
            )
        except TransformException:
            self.get_logger().warn("Waiting for target marker TF...")
            return

        marker_z = t.transform.translation.z

        # -------------------------------------------------------
        # PHASE 1: Lift arm to marker height
        # -------------------------------------------------------
        if self.phase == 'LIFT':
            lift_target = marker_z - 0.05
            lift_target = max(min(lift_target, 0.9), 0.1)

            self.get_logger().info(f"Lifting arm to {lift_target:.3f} m")

            self.send_traj(
                ["joint_lift"],
                [lift_target],
                duration=2.0
            )

            self.phase = 'EXTEND'
            return

        # -------------------------------------------------------
        # PHASE 2: Extend arm until gripper marker becomes visible
        # -------------------------------------------------------
        if self.phase == 'EXTEND':
            # Try to see if gripper marker is visible
            try:
                self.tf_buffer.lookup_transform(
                    'base_link',
                    self.gripper_frame,
                    RclTime()
                )
                self.get_logger().info("Gripper marker visible — ready for servoing.")
                self.phase = 'DONE'
                return
            except TransformException:
                pass

            # Extend arm gradually
            if not self.arm_extended:
                self.get_logger().info("Extending arm to 0.40 m")

                arm_len = 0.40
                self.send_traj(
                    ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"],
                    [arm_len, arm_len, arm_len, arm_len],
                    duration=3.0
                )

                # Pitch wrist so marker faces camera
                self.send_traj(
                    ["joint_wrist_pitch"],
                    [-0.4],
                    duration=2.0
                )

                self.arm_extended = True

            return

        # -------------------------------------------------------
        # DONE
        # -------------------------------------------------------
        if self.phase == 'DONE':
            self.get_logger().info("Pre-servo positioning complete. Run the servo node now.")
            return


def main(args=None):
    rclpy.init(args=args)
    node = PreServoArmPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

