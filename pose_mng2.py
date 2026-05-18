#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from stretch_body.robot import Robot
import json
import os
import time

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


class PoseManager(Node):
    def __init__(self):
        super().__init__("pose_manager")

        self.poses = {}
        self.latest_joint_state = None

        # Initialize Stretch robot SDK
        self.robot = Robot()
        self.robot.startup()

        # Load saved poses from disk
        self.load_poses()

        # Subscribe to pose manager commands
        self.create_subscription(
            String,
            "/pose_mgr",
            self.pose_mgr_callback,
            10
        )

        # Publisher for pose list updates
        self.pose_list_pub = self.create_publisher(String, "/pose_list", 10)

        # Publisher for joint states (SDK-only mode)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timer to publish joint states at 30 Hz
        self.create_timer(1.0 / 30.0, self.publish_joint_states)

        self.get_logger().info("Pose Manager Ready (SDK MODE, standalone)")

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

        elif cmd == "restore":
            self.restore_pose(name)

        elif cmd == "delete":
            self.delete_pose(name)

        elif cmd == "quit":
            self.get_logger().info("Shutting down Pose Manager...")
            self.shutdown()
            return

        elif cmd == "list":
            self.publish_pose_list()
            self.get_logger().info("Published pose list")
            return

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

        self.publish_pose_list()

    # -----------------------------
    # SAVE POSE
    # -----------------------------
    def save_pose(self, name):
        if self.latest_joint_state is None:
            self.get_logger().warn("No joint states yet")
            return

        pose = {}
        for joint, pos in zip(self.latest_joint_state.name,
                              self.latest_joint_state.position):
            if joint in CONTROLLABLE_JOINTS:
                pose[joint] = pos

        self.poses[name] = pose
        self.save_poses()
        self.get_logger().info(f"Saved pose '{name}'")

    # -----------------------------
    # RESTORE POSE
    # -----------------------------
    def restore_pose(self, name):
        if name not in self.poses:
            self.get_logger().warn(f"Pose '{name}' not found")
            return

        pose = self.poses[name]
        self.get_logger().info(f"Restoring pose '{name}'")

        # Lift
        if "joint_lift" in pose:
            self.robot.lift.move_to(pose["joint_lift"])

        # Arm (all segments share the same extension)
        arm_vals = [pose.get(f"joint_arm_l{i}", None) for i in range(4)]
        if all(v is not None for v in arm_vals):
            self.robot.arm.move_to(arm_vals[0])

        # Wrist joints
        if "joint_wrist_yaw" in pose:
            self.robot.end_of_arm.move_to("wrist_yaw", pose["joint_wrist_yaw"])

        if "joint_wrist_pitch" in pose:
            self.robot.end_of_arm.move_to("wrist_pitch", pose["joint_wrist_pitch"])

        if "joint_wrist_roll" in pose:
            self.robot.end_of_arm.move_to("wrist_roll", pose["joint_wrist_roll"])

        # Gripper fingers
        if "joint_gripper_finger_left" in pose:
            self.robot.end_of_arm.move_to(
                "gripper_finger_left",
                pose["joint_gripper_finger_left"]
            )

        if "joint_gripper_finger_right" in pose:
            self.robot.end_of_arm.move_to(
                "gripper_finger_right",
                pose["joint_gripper_finger_right"]
            )

        self.robot.push_command()
        time.sleep(0.1)

        self.get_logger().info(f"Pose '{name}' restored")

    # -----------------------------
    # DELETE POSE
    # -----------------------------
    def delete_pose(self, name):
        if name in self.poses:
            del self.poses[name]
            self.save_poses()
            self.get_logger().info(f"Deleted pose '{name}'")

    # -----------------------------
    # PUBLISH POSE LIST
    # -----------------------------
    def publish_pose_list(self):
        msg = String()
        msg.data = json.dumps(list(self.poses.keys()))
        self.pose_list_pub.publish(msg)

    # -----------------------------
    # SAVE POSES TO DISK
    # -----------------------------
    def save_poses(self):
        try:
            with open(POSE_FILE, "w") as f:
                json.dump(self.poses, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save poses: {e}")

    # -----------------------------
    # LOAD POSES FROM DISK
    # -----------------------------
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
    def shutdown(self):
        try:
            self.robot.stop()
        except Exception as e:
            self.get_logger().warn(f"Robot stop error: {e}")

        self.get_logger().info("Stopping ROS node...")
        rclpy.shutdown()
        os._exit(0)

    # -----------------------------
    # PUBLISH JOINT STATES FROM SDK
    # -----------------------------
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        self.robot.pull_status()

        names = []
        positions = []

        # Lift
        names.append("joint_lift")
        positions.append(self.robot.lift.status["pos"])

        # Arm segments
        arm_pos = self.robot.arm.status["pos"]
        for i in range(4):
            names.append(f"joint_arm_l{i}")
            positions.append(arm_pos)

        # Wrist joints
        names.append("joint_wrist_yaw")
        positions.append(self.robot.end_of_arm.status["wrist_yaw"]["pos"])

        names.append("joint_wrist_pitch")
        positions.append(self.robot.end_of_arm.status["wrist_pitch"]["pos"])

        names.append("joint_wrist_roll")
        positions.append(self.robot.end_of_arm.status["wrist_roll"]["pos"])

        # Gripper fingers (only if present)
        eoast = self.robot.end_of_arm.status

        if "gripper_finger_left" in eoast:
            names.append("joint_gripper_finger_left")
            positions.append(eoast["gripper_finger_left"]["pos"])

        if "gripper_finger_right" in eoast:
            names.append("joint_gripper_finger_right")
            positions.append(eoast["gripper_finger_right"]["pos"])

        js.name = names
        js.position = positions

        self.joint_state_pub.publish(js)
        self.latest_joint_state = js


def main(args=None):
    rclpy.init(args=args)
    node = PoseManager()
    rclpy.spin(node)
    node.robot.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

