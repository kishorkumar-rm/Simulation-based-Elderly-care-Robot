#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math

# ===============================
# Parameters
# ===============================
NUM_PARTICLES = 40
MAX_ITER = 10

MAX_LINEAR = 0.25
MAX_ANGULAR = 1.2

SAFE_DISTANCE = 0.6      # meters
CRITICAL_DISTANCE = 0.4

C1 = 1.4
C2 = 1.4

GRID_RES = 0.5  # exploration memory resolution


class RLPSOController(Node):

    def __init__(self):
        super().__init__('rlpso_obstacle_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.scan = None
        self.visited = set()

        self.get_logger().info("RLPSO obstacle-aware controller started")

    # -------------------------------
    # LiDAR callback
    # -------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    # -------------------------------
    # Obstacle metrics
    # -------------------------------
    def min_front_distance(self):
        if self.scan is None:
            return float('inf')

        ranges = np.array(self.scan.ranges)
        ranges[ranges == 0.0] = np.inf

        center = len(ranges) // 2
        front = ranges[center - 15:center + 15]
        return np.min(front)

    # -------------------------------
    # Exploration memory
    # -------------------------------
    def exploration_penalty(self, linear, angular):
        x = round(linear / GRID_RES, 1)
        z = round(angular / GRID_RES, 1)
        key = (x, z)
        if key in self.visited:
            return 1.0
        self.visited.add(key)
        return 0.0

    # -------------------------------
    # Fitness function (CORE FIX)
    # -------------------------------
    def fitness(self, linear, angular):
        d = self.min_front_distance()

        if d < CRITICAL_DISTANCE:
            return -1000.0  # hard violation

        clearance_reward = d
        smooth_penalty = abs(angular)
        revisit_penalty = self.exploration_penalty(linear, angular)

        return clearance_reward - smooth_penalty - revisit_penalty

    # -------------------------------
    # RLPSO core
    # -------------------------------
    def compute_cmd(self):
        X = []
        V = []
        Pbest = []

        for _ in range(NUM_PARTICLES):
            lin = random.uniform(0.05, MAX_LINEAR)
            ang = random.uniform(-MAX_ANGULAR, MAX_ANGULAR)
            X.append([lin, ang])
            V.append([0.0, 0.0])
            Pbest.append([lin, ang])

        Gbest = X[0]

        for _ in range(MAX_ITER):
            for i in range(NUM_PARTICLES):
                if self.fitness(*X[i]) > self.fitness(*Pbest[i]):
                    Pbest[i] = X[i]

                if self.fitness(*Pbest[i]) > self.fitness(*Gbest):
                    Gbest = Pbest[i]

            for i in range(NUM_PARTICLES):
                r1, r2 = random.random(), random.random()
                V[i][0] += C1 * r1 * (Pbest[i][0] - X[i][0]) + C2 * r2 * (Gbest[0] - X[i][0])
                V[i][1] += C1 * r1 * (Pbest[i][1] - X[i][1]) + C2 * r2 * (Gbest[1] - X[i][1])
                X[i][0] += V[i][0]
                X[i][1] += V[i][1]

        return Gbest

    # -------------------------------
    # Control loop
    # -------------------------------
    def control_loop(self):
        if self.scan is None:
            return

        d = self.min_front_distance()

        twist = Twist()

        # Emergency avoidance
        if d < SAFE_DISTANCE:
            twist.linear.x = 0.05
            twist.angular.z = random.choice([-1.0, 1.0])
        else:
            lin, ang = self.compute_cmd()
            twist.linear.x = float(lin)
            twist.angular.z = float(ang)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = RLPSOController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

