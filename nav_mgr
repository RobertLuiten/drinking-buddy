# The navigation manager for the project
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        # Command subscriber
        self.sub = self.create_subscription(
            String,
            '/nav_mgr_cmd',
            self.cmd_callback,
            10
        )

        # Nav2 interface (BasicNavigator spins its own node internally)
        self.navigator = BasicNavigator()

        # TF2 interface
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Saved poses
        self.saved = {}

        # Wait for Nav2 to activate
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active and ready.")

    # ----------------------------
    # TF lookup: map → base_link
    # ----------------------------
    def get_robot_pose(self):
        try:
            # Allow TF to process messages
            rclpy.spin_once(self, timeout_sec=0.1)

            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            return pose

        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    # ----------------------------
    # Command handler
    # ----------------------------
    def cmd_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")

        parts = msg.data.strip().split()
        if not parts:
            return

        cmd = parts[0]

        # SAVE
        if cmd == "save" and len(parts) == 2:
            name = parts[1]
            pose = self.get_robot_pose()
            if pose is None:
                self.get_logger().warn("Could not get robot pose.")
                return
            self.saved[name] = pose
            self.get_logger().info(f"Saved pose '{name}'")

        # GO
        elif cmd == "go" and len(parts) == 2:
            name = parts[1]
            if name not in self.saved:
                self.get_logger().warn(f"No such pose '{name}'")
                return

            goal = self.saved[name]
            self.get_logger().info(f"Navigating to '{name}'")

            # Send goal
            self.navigator.goToPose(goal)

            # Wait for completion using the correct API
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            self.get_logger().info(f"Navigation result: {result}")

        # LIST
        elif cmd == "list":
            self.get_logger().info(f"Saved poses: {list(self.saved.keys())}")

        else:
            self.get_logger().warn("Commands: save <name>, go <name>, list")


def main():
    rclpy.init()
    node = NavManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

