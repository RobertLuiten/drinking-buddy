
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener

from your_package_name.action import GoToNamedPose   # <-- update package name


class NavManagerActionServer(Node):
    def __init__(self):
        super().__init__('nav_manager_action_server')

        # Command subscriber (save/list still supported)
        self.sub = self.create_subscription(
            String,
            '/nav_mgr_cmd',
            self.cmd_callback,
            10
        )

        # Nav2 interface
        self.navigator = BasicNavigator()

        # TF2 interface
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Saved poses
        self.saved = {}

        # Action server
        self._action_server = ActionServer(
            self,
            GoToNamedPose,
            'go_to_named_pose',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active and ready.")

    # ----------------------------
    # TF lookup: map → base_link
    # ----------------------------
    def get_robot_pose(self):
        try:
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
    # Action cancel handler
    # ----------------------------
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received.")
        self.navigator.cancelTask()
        return CancelResponse.ACCEPT

    # ----------------------------
    # Action execution
    # ----------------------------
    async def execute_callback(self, goal_handle):
        target_name = goal_handle.request.target_name
        self.get_logger().info(f"Action goal received: go to '{target_name}'")

        feedback = GoToNamedPose.Feedback()

        # Check pose exists
        if target_name not in self.saved:
            msg = f"No saved pose named '{target_name}'"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return GoToNamedPose.Result(success=False, message=msg)

        goal_pose = self.saved[target_name]

        # Send navigation goal
        self.navigator.goToPose(goal_pose)
        feedback.state = "moving"
        goal_handle.publish_feedback(feedback)

        # Monitor progress
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                goal_handle.canceled()
                return GoToNamedPose.Result(success=False, message="Goal canceled")

            rclpy.spin_once(self, timeout_sec=0.1)

        # Get result
        result = self.navigator.getResult()

        if result == BasicNavigator.TaskResult.SUCCEEDED:
            msg = f"Reached '{target_name}'"
            self.get_logger().info(msg)
            goal_handle.succeed()
            return GoToNamedPose.Result(success=True, message=msg)

        else:
            msg = f"Failed to reach '{target_name}'"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return GoToNamedPose.Result(success=False, message=msg)

    # ----------------------------
    # Command handler (save/list)
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

        # LIST
        elif cmd == "list":
            self.get_logger().info(f"Saved poses: {list(self.saved.keys())}")

        else:
            self.get_logger().warn("Commands: save <name>, list")


def main():
    rclpy.init()
    node = NavManagerActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
