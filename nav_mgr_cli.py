# Navigation CLI for testing purposes and Lab #15

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse
import sys

class NavCLI(Node):
    def __init__(self):
        super().__init__('nav_cli')
        self.pub = self.create_publisher(String, '/nav_mgr_cmd', 10)

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {text}")


def main():
    parser = argparse.ArgumentParser(description="Nav Manager CLI")
    sub = parser.add_subparsers(dest="cmd")

    save = sub.add_parser("save")
    save.add_argument("name")

    go = sub.add_parser("go")
    go.add_argument("name")

    sub.add_parser("list")

    args = parser.parse_args()

    rclpy.init()
    node = NavCLI()

    if args.cmd == "save":
        node.send(f"save {args.name}")
    elif args.cmd == "go":
        node.send(f"go {args.name}")
    elif args.cmd == "list":
        node.send("list")
    else:
        print("Commands: save <name>, go <name>, list")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
