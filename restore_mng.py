#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from stretch_body.robot import Robot
import json
import os
import time

POSE_FILE = "/home/hello-robot/.poses.json"

class RestoreMgr(Node):
    def __init__(self):
        super().__init__("restore_mgr")

        # Robot owner
        self.robot = Robot()
        self.robot.startup()

        # Subscribe to commands
        self.create_subscription(
            String,
            "/pose_mgr",
            self.pose_mgr_callback,
            10
        )

        # Publisher for pose list
        self.pose_list_pub = self.create_publisher(String, "/pose_list", 10)

        self.get_logger().info("RestoreMgr Ready (list + restore + delete + quit)")

    # -----------------------------
    # LOAD POSES FROM DISK
    # -----------------------------
    def load_poses(self):
        if os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE, "r") as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"Failed to load poses: {e}")
        return {}

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

        if cmd == "list":
            self.publish_pose_list()

        elif cmd == "restore":
            self.restore_pose(name)

        elif cmd == "delete":
            self.delete_pose(name)

        elif cmd == "quit":
            self.shutdown()

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # -----------------------------
    # PUBLISH POSE LIST
    # -----------------------------
    def publish_pose_list(self):
        poses = self.load_poses()
        msg = String()
        msg.data = json.dumps(list(poses.keys()))
        self.pose_list_pub.publish(msg)
        self.get_logger().info("Published pose list")

    # -----------------------------
    # RESTORE POSE
    # -----------------------------
    def restore_pose(self, name):
        poses = self.load_poses()

        if name not in poses:
            self.get_logger().warn(f"Pose '{name}' not found")
            return

        pose = poses[name]
        self.get_logger().info(f"Restoring pose '{name}'")

        # Lift
        if "joint_lift" in pose:
            self.robot.lift.move_to(pose["joint_lift"])

        # Arm (all segments share same extension)
        if "joint_arm_l0" in pose:
            self.robot.arm.move_to(pose["joint_arm_l0"])

        # Wrist joints
        eoam = self.robot.end_of_arm
        if "joint_wrist_yaw" in pose:
            eoam.move_to("wrist_yaw", pose["joint_wrist_yaw"])
        if "joint_wrist_pitch" in pose:
            eoam.move_to("wrist_pitch", pose["joint_wrist_pitch"])
        if "joint_wrist_roll" in pose:
            eoam.move_to("wrist_roll", pose["joint_wrist_roll"])

        # Gripper
        if "joint_gripper_finger_left" in pose:
            eoam.move_to("gripper_finger_left", pose["joint_gripper_finger_left"])
        if "joint_gripper_finger_right" in pose:
            eoam.move_to("gripper_finger_right", pose["joint_gripper_finger_right"])

        self.robot.push_command()
        time.sleep(0.1)

        self.get_logger().info(f"Pose '{name}' restored")

    # -----------------------------
    # DELETE POSE
    # -----------------------------
    def delete_pose(self, name):
        poses = self.load_poses()

        if name in poses:
            del poses[name]
            with open(POSE_FILE, "w") as f:
                json.dump(poses, f, indent=2)
            self.get_logger().info(f"Deleted pose '{name}'")
            self.publish_pose_list()
        else:
            self.get_logger().warn(f"Pose '{name}' not found")

    # -----------------------------
    # CLEAN SHUTDOWN
    # -----------------------------
    def shutdown(self):
        self.get_logger().info("Shutting down RestoreMgr")
        self.robot.stop()
        rclpy.shutdown()
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = RestoreMgr()
    rclpy.spin(node)
    node.robot.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

