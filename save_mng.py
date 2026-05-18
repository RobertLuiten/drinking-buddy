#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import time
import threading

from stretch_body.robot import Robot
from stretch_body.gamepad_teleop import GamePadTeleop

POSE_FILE = "/home/hello-robot/.poses.json"

CONTROLLABLE_JOINTS = [
    "joint_lift",
    "joint_arm_l0",
    "joint_arm_l1",
    "joint_arm_l2",
    "joint_arm_l3",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
    "joint_gripper_finger_left",
    "joint_gripper_finger_right",
]


class SaveMgr(Node):
    def __init__(self):
        super().__init__("save_mgr")

        self.poses = {}
        self.load_poses()

        # Own the robot
        self.robot = Robot()
        self.robot.startup()

        # Gamepad teleop (uses same robot)
        self.gamepad = GamePadTeleop(robot_instance=False)
        self.gamepad.robot = self.robot
        self.gamepad.startup(robot=self.robot)

        # Run teleop loop in a background thread
        self._teleop_running = True
        self.teleop_thread = threading.Thread(
            target=self.teleop_loop, daemon=True
        )
        self.teleop_thread.start()

        # Subscribe to save/list commands
        self.create_subscription(
            String,
            "/pose_mgr",
            self.pose_mgr_callback,
            10,
        )

        # Publisher for pose list
        self.pose_list_pub = self.create_publisher(String, "/pose_list", 10)

        self.get_logger().info(
            "SaveMgr Ready (owns robot, gamepad teleop + pose saving)"
        )

    # -----------------------------
    # TELEOP LOOP
    # -----------------------------
    def teleop_loop(self):
        try:
            while self._teleop_running:
                self.gamepad.step_mainloop(robot=self.robot)
        except Exception as e:
            self.get_logger().error(f"Teleop loop error: {e}")
        finally:
            self.robot.stop()

    # -----------------------------
    # COMMAND HANDLER
    # -----------------------------
    def pose_mgr_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid JSON: {e}")
            return

        cmd = data.get("command", "").lower()
        name = data.get("name", "")

        if cmd == "save":
            self.save_pose(name)

        elif cmd == "list":
            self.publish_pose_list()
            return

        else:
            self.get_logger().warn(f"SaveMgr ignoring unsupported command: {cmd}")

        self.publish_pose_list()

    # -----------------------------
    # SAVE POSE (DIRECT FROM ROBOT)
    # -----------------------------
    def save_pose(self, name):
        # Pull latest status from hardware
        self.robot.pull_status()

        pose = {}

        # Lift
        pose["joint_lift"] = self.robot.lift.status["pos"]

        # Arm segments (all share same extension)
        arm_pos = self.robot.arm.status["pos"]
        for i in range(4):
            pose[f"joint_arm_l{i}"] = arm_pos

        # Wrist + gripper (if present)
        eoast = self.robot.end_of_arm.status

        if "wrist_yaw" in eoast:
            pose["joint_wrist_yaw"] = eoast["wrist_yaw"]["pos"]
        if "wrist_pitch" in eoast:
            pose["joint_wrist_pitch"] = eoast["wrist_pitch"]["pos"]
        if "wrist_roll" in eoast:
            pose["joint_wrist_roll"] = eoast["wrist_roll"]["pos"]

        if "gripper_finger_left" in eoast:
            pose["joint_gripper_finger_left"] = eoast["gripper_finger_left"]["pos"]
        if "gripper_finger_right" in eoast:
            pose["joint_gripper_finger_right"] = eoast["gripper_finger_right"]["pos"]

        # Filter to controllable joints only
        pose = {j: v for j, v in pose.items() if j in CONTROLLABLE_JOINTS}

        self.poses[name] = pose
        self.save_poses()
        self.get_logger().info(f"Saved pose '{name}'")

    # -----------------------------
    # PUBLISH POSE LIST
    # -----------------------------
    def publish_pose_list(self):
        msg = String()
        msg.data = json.dumps(list(self.poses.keys()))
        self.pose_list_pub.publish(msg)

    # -----------------------------
    # SAVE / LOAD POSES
    # -----------------------------
    def save_poses(self):
        try:
            with open(POSE_FILE, "w") as f:
                json.dump(self.poses, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save poses: {e}")

    def load_poses(self):
        if os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE, "r") as f:
                    self.poses = json.load(f)
                self.get_logger().info(f"Loaded {len(self.poses)} poses")
            except Exception as e:
                self.get_logger().error(f"Failed to load poses: {e}")
        else:
            self.get_logger().info("No pose file found")

    # -----------------------------
    # CLEAN SHUTDOWN
    # -----------------------------
    def destroy_node(self):
        self._teleop_running = False
        time.sleep(0.1)
        try:
            self.gamepad.stop()
        except Exception:
            pass
        try:
            self.robot.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SaveMgr()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

