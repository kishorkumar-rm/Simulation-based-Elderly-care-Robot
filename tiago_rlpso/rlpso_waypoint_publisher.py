#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
import time


class RLPSOGlobalPlanner(Node):

    def __init__(self):
        super().__init__('rlpso_global_planner')
        self.pub = self.create_publisher(Path, '/rlpso_path', 10)

        self.timer = self.create_timer(2.0, self.publish_path)
        self.get_logger().info("RLPSO Global Planner started")

    def publish_path(self):
        path = Path()
        path.header.frame_id = "map"

        # Dummy RLPSO-style global exploration path
        x, y = 0.0, 0.0
        for _ in range(25):
            x += random.uniform(0.2, 0.6)
            y += random.uniform(-0.5, 0.5)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)
        self.get_logger().info("Published RLPSO global path")


def main():
    rclpy.init()
    node = RLPSOGlobalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

