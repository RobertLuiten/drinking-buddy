# Code for Drinking Buddy pose manager

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
    #"joint_gripper_finger_left"
]

class PoseManager(Node):
    def __init__(self):
        super().__init__("pose_manager")

        self.poses = {}
        self.latest_joint_state = None

        self.robot = Robot()
        self.robot.startup()

        self.load_poses()

        self.create_subscription(JointState, "/joint_states",
                                 self.joint_state_callback, 10)

        self.create_subscription(String, "/pose_mgr",
                                 self.pose_mgr_callback, 10)

        self.pose_list_pub = self.create_publisher(String, "/pose_list", 10)

        self.get_logger().info("Pose Manager Ready (SDK MODE)")

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

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
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

        self.publish_pose_list()

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

        # Gripper (correct API)
        

        # Send commands
        self.robot.push_command()
        time.sleep(0.1)

        self.get_logger().info(f"Pose '{name}' restored")

    def delete_pose(self, name):
        if name in self.poses:
            del self.poses[name]
            self.save_poses()
            self.get_logger().info(f"Deleted pose '{name}'")

    def publish_pose_list(self):
        msg = String()
        msg.data = json.dumps(list(self.poses.keys()))
        self.pose_list_pub.publish(msg)

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

def main(args=None):
    rclpy.init(args=args)
    node = PoseManager()
    rclpy.spin(node)
    node.robot.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
