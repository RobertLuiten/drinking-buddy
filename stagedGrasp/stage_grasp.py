#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from tf_transformations import quaternion_matrix


class StretchReadyPose(Node):
    def __init__(self):
        super().__init__('stretch_ready_pose', namespace='/')

        # Load ready.yaml
        with open('ready.yaml', 'r') as f:
            data = yaml.safe_load(f)
        self.ready_joints = data['ready_pose']['joints']

        # Trajectory client
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

        self.sent = False
        self.ready_to_align = False
        self.ready_timer = self.create_timer(0.1, self.try_send_ready_pose)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_supps = None
        self.marker_frame = 'supps'

        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.base_centered = False
        self.arm_centered = False

        self.latest_supps_wrist = None

        # Timers
        self.supps_timer = self.create_timer(0.05, self.update_tf)  # 1 Hz TF update
        self.align_timer = self.create_timer(0.1, self.loop)          # 2 Hz base alignment

        #  for arm_align
        self.arm_goal_active = False
        self.latest_base_wrist = None

        self.arm_offset = 0.0

        self.final_approach_done = False

        self.gripped = False
        self.grip_complete = False
        self.retracted = False

        


        

    # -----------------------------------------------------------
    # SEND READY POSE
    # -----------------------------------------------------------
    def try_send_ready_pose(self):
        if self.sent:
            return

        if not self.traj_client.wait_for_server(timeout_sec=0.1):
            return

        self.get_logger().info("Sending READY pose...")

        joint_names = list(self.ready_joints.keys())
        positions = [self.ready_joints[j] for j in joint_names]

        traj = JointTrajectory()
        traj.joint_names = joint_names

        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = 3
        traj.points = [p]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.traj_client.send_goal_async(goal)
        future.add_done_callback(self.ready_sent)

    def ready_sent(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("READY pose rejected")
            return

        self.get_logger().info("READY pose accepted.")
        self.sent = True
        handle.get_result_async().add_done_callback(self.ready_done)


    def ready_done(self, future):
        self.get_logger().info("READY pose complete.")

        # Wait 1 second for joints + TF to settle
        self.create_timer(1.0, self._mark_ready_to_align)

    def _mark_ready_to_align(self):
        self.ready_to_align = True



    # -----------------------------------------------------------
    # TF: SUPPS
    # -----------------------------------------------------------
    def update_tf(self):
        t = self.get_supps()
        t_wrist = self.get_supps_wrist()
        t_wrist_base = self.get_base_wrist()
        if t is not None:
            self.latest_supps = t
        #     self.get_logger().info("Supps TF valid")
        # else:
        #     self.get_logger().warn("Supps TF is None")

        if t_wrist is not None:
            self.latest_supps_wrist = t_wrist

        if t_wrist_base is not None:
            self.latest_base_wrist = t_wrist_base
        

        


    def get_supps(self, timeout_sec=0.5):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                self.marker_frame,
                Time(),
                Duration(seconds=timeout_sec)
            )
            return t
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {repr(e)}")
            return None

    def get_base_wrist(self, timeout_sec=0.5):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'link_grasp_center',
                Time(),
                Duration(seconds=timeout_sec)
            )
            return t
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {repr(e)}")
            return None


    def get_supps_wrist(self, timeout_sec=0.5):
        try:
            t = self.tf_buffer.lookup_transform(
                'link_grasp_center',
                self.marker_frame,
                Time(),
                Duration(seconds=timeout_sec)
            )
            
            # tx = t.transform.translation.x
            # ty = t.transform.translation.y
            # tz = t.transform.translation.z
            # self.get_logger().info(
            #     f"grasp_center→supps  x={tx:.3f}  y={ty:.3f}  z={tz:.3f}"
            # )
            return t
        except Exception as e:
            self.get_logger().error(f"TF wrist to supps lookup failed: {repr(e)}")
            return None


    # -----------------------------------------------------------
    # BASE ALIGNMENT (ONLY FORWARD/BACK)
    # -----------------------------------------------------------


    def align_base(self):
        if self.latest_supps is None or self.latest_base_wrist is None:
            self.get_logger().warn("TFs missing — cannot align base.")
            return

        s = self.latest_supps.transform.translation      # base_link→supps
        w = self.latest_base_wrist.transform.translation # base_link→gripper

        # Distance along base_link X between gripper and marker
        
        distance = w.x - s.x
        desired = 0.19   # keep gripper slightly behind marker
        tol = 0.01       # acceptable alignment tolerance
        k = 0.15          # proportional gain

        twist = Twist()

        err = desired - distance

        self.get_logger().info(
            f"[BASE ALIGN] s.x={s.x: .3f}, w.x={w.x:.3f}, err={err:.3f}, "
            f"desired={desired:.3f}, tol={tol:.3f}"
        )

        if abs(err - desired) < tol:
            self.get_logger().info("Base aligned: gripper X lined up with supps Z.")
            self.base_centered = True
            self.cmd_vel_pub.publish(Twist())
            return

        # Move forward/back along base_link X
        twist.linear.x = k * (err - desired)
        twist.linear.x = max(min(twist.linear.x, 0.20), -0.20)
        self.cmd_vel_pub.publish(twist)
    
    def align_arm(self, err):
        self.get_logger().info(f"[ARM ALIGN] err={err:.3f}")

        tol = 0.01  # 1 cm band around min_dist
        if abs(err) < tol:
            self.get_logger().info("Arm at pre-grasp distance.")
            self.arm_centered = True
            return

        if self.arm_goal_active:
            return

        # SAFETY: never go closer than (min_dist - 2 cm)
        if err < -0.02:
            self.get_logger().warn("Too close to marker — not extending further.")
            self.arm_centered = True
            return

        k = 0.4
        delta = k * err
        delta = max(min(delta, 0.10), -0.10)

        d = delta / 4.0
        self.arm_offset += d
        self.arm_offset = max(min(self.arm_offset, 0.45), -0.05)

        self.arm_goal_active = True
        self.send_arm_goal(self.arm_offset)


    def _arm_goal_done(self, future):
        self.arm_goal_active = False

    def send_arm_goal(self, offset):
        traj = JointTrajectory()
        traj.joint_names = [
            "joint_lift",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_gripper_finger_left",
        ]

        positions = [self.ready_joints[j] for j in traj.joint_names]
        for i in range(1, 5):
            positions[i] = self.ready_joints[traj.joint_names[i]] + offset

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal).add_done_callback(self._arm_goal_done)

    def send_arm_retract_goal(self, offset):
        traj = JointTrajectory()
        traj.joint_names = [
            "joint_lift",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
        ]

        positions = [self.ready_joints[j] for j in traj.joint_names]

        # apply offset only to arm segments
        for i in range(1, 5):
            positions[i] = self.ready_joints[traj.joint_names[i]] + offset

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal).add_done_callback(self._arm_goal_done)

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ["joint_gripper_finger_left"]

        point = JointTrajectoryPoint()
        point.positions = [-0.5]
        point.time_from_start.sec = 4

        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.traj_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response)


    def _gripper_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return

        self.get_logger().info("Gripper goal accepted")

        # THIS is where we wait for actual completion
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_callback)


    def _gripper_result_callback(self, future):
        result = future.result()

        # FollowJointTrajectoryResult is wrapped in result.result
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Gripper motion succeeded.")
            self.grip_complete = True
        else:
            self.get_logger().warn(f"Gripper failed with status: {status}")
            self.grip_complete = False

    def retract_arm(self):
        self.arm_offset -= 0.15
        self.arm_offset = max(self.arm_offset, 0.0)
        self.send_arm_retract_goal(self.arm_offset)

    def retract_done(self, future):
        self.get_logger().info("Arm retracted.")
        self.retracted = True

    # -----------------------------------------------------------
    # MAIN ALIGN LOOP
    # -----------------------------------------------------------

    def loop(self):
         
        if not self.sent:
            return
        

        # waiting until ready pose done
        if not self.ready_to_align:
            # still waiting for READY pose to be accepted
            return



        if self.latest_supps_wrist is None:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn("Marker lost — stopping base.")
            return

        # --- BASE ALIGNMENT ---
        if not self.base_centered:
            
            self.align_base()
            return   # <-- THIS is the missing piece



        # # --- ARM ALIGNMENT ---
        if not self.arm_centered:

            if self.latest_supps_wrist is None:
                self.get_logger().warn("Marker lost - stopping arm.")
                self.arm_goal_active = False
                return

            tx = self.latest_supps_wrist.transform.translation.x  # forward distance

            # target: stop 8 cm in front of marker
            min_dist = 0.08
            err = tx - min_dist

            self.get_logger().info(
                f"[DEBUG] tx={tx:.3f}, err={err:.3f}"
)
            
            self.align_arm(err)
            return 

        if self.arm_centered and not self.final_approach_done:
            extra = 0.04
            self.arm_offset += extra
            self.arm_offset = max(min(self.arm_offset, .045), -.05)
            self.final_approach_done = True
            self.send_arm_goal(self.arm_offset)

        # # -- The Coute de Gras: Gripping The Cup and Retracting The Arm!
        if self.final_approach_done and not self.gripped:
            self.gripped = True
            self.close_gripper()
            return

        if self.grip_complete and not self.retracted:
            self.retract_arm()
            return





def main(args=None):
    rclpy.init(args=args)
    node = StretchReadyPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
