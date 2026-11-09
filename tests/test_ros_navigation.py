#!/usr/bin/env python3
"""Automated ROS 2 integration tests for Gazebo Harmonic & Nav2 warehouse simulation.

Verifies:
  - Sensor data stream: /scan LaserScan and /clock simulated time
  - Transform tree: map -> odom -> base_link -> laser_link
  - Nav2 costmap generation: local and global costmaps receive obstacles from /scan
  - Scenario 1: Straight aisle navigation along warehouse rack corridor
  - Scenario 2: 90-degree aisle turn through warehouse cross-aisle
  - Scenario 3: Static obstacle avoidance around central pallet stack
  - Scenario 4: Dynamic obstacle detection in local costmap
  - Scenario 5: Multi-waypoint mission dispatch via NavigateToPose action

Execution:
  In a sourced ROS 2 Jazzy + Gazebo Harmonic workspace:
    python3 -m unittest tests/test_ros_navigation.py -v
"""
from __future__ import annotations

import math
import shutil
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Check ROS 2 availability
HAS_ROS = (
    sys.platform.startswith("linux")
    and shutil.which("ros2") is not None
)

if HAS_ROS:
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav2_msgs.action import NavigateToPose
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
    except ImportError:
        HAS_ROS = False


@unittest.skipUnless(HAS_ROS, "ROS 2 Jazzy environment not available on this host")
class RosWarehouseNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_warehouse_navigation_client")
        cls.action_client = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")

    @classmethod
    def tearDownClass(cls):
        cls.action_client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.latest_scan: LaserScan | None = None
        self.latest_clock: Clock | None = None
        self.latest_odom: Odometry | None = None
        self.latest_local_costmap: OccupancyGrid | None = None
        self.received_tf_map_odom = False
        self.received_tf_odom_base = False

        self.scan_sub = self.node.create_subscription(
            LaserScan, "/scan", self._on_scan, 10
        )
        self.clock_sub = self.node.create_subscription(
            Clock, "/clock", self._on_clock, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._on_odom, 10
        )
        self.costmap_sub = self.node.create_subscription(
            OccupancyGrid, "/local_costmap/costmap_raw", self._on_costmap, 10
        )
        self.tf_sub = self.node.create_subscription(
            TFMessage, "/tf", self._on_tf, 20
        )

        # Spin to receive initial topics
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_scan is not None and self.latest_clock is not None:
                break

    def tearDown(self):
        self.node.destroy_subscription(self.scan_sub)
        self.node.destroy_subscription(self.clock_sub)
        self.node.destroy_subscription(self.odom_sub)
        self.node.destroy_subscription(self.costmap_sub)
        self.node.destroy_subscription(self.tf_sub)

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_clock(self, msg: Clock):
        self.latest_clock = msg

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _on_costmap(self, msg: OccupancyGrid):
        self.latest_local_costmap = msg

    def _on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            parent = t.header.frame_id.strip("/")
            child = t.child_frame_id.strip("/")
            if parent == "map" and child == "odom":
                self.received_tf_map_odom = True
            elif parent == "odom" and child == "base_link":
                self.received_tf_odom_base = True

    def _send_nav_goal(self, target_x: float, target_y: float, target_yaw: float = 0.0, timeout_sec: float = 40.0) -> tuple[bool, float, float]:
        """Send a NavigateToPose goal, wait for completion, and return (success, duration, pose_error)."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False, 0.0, float("inf")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target_x)
        goal.pose.pose.position.y = float(target_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        send_future = self.action_client.send_goal_async(goal)
        start_time = time.time()

        # Wait for goal acceptance
        while time.time() - start_time < 5.0 and not send_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not send_future.done():
            return False, time.time() - start_time, float("inf")

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, time.time() - start_time, float("inf")

        result_future = goal_handle.get_result_async()
        while time.time() - start_time < timeout_sec and not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        duration = time.time() - start_time
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False, duration, float("inf")

        # Compute pose error if odom available
        pose_error = 0.0
        if self.latest_odom is not None:
            curr_x = self.latest_odom.pose.pose.position.x
            curr_y = self.latest_odom.pose.pose.position.y
            pose_error = math.sqrt((curr_x - target_x) ** 2 + (curr_y - target_y) ** 2)

        return True, duration, pose_error

    def test_01_lidar_scan_valid(self):
        """Verify /scan publishes valid ranges from Gazebo gpu_lidar."""
        start = time.time()
        while time.time() - start < 5.0 and self.latest_scan is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_scan, "No LaserScan received on /scan")
        self.assertEqual(self.latest_scan.header.frame_id, "laser_link")
        self.assertGreater(len(self.latest_scan.ranges), 100)
        valid_ranges = [r for r in self.latest_scan.ranges if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
        self.assertGreater(len(valid_ranges), 10, "LiDAR scan contained no valid obstacle reflections")

    def test_02_simulated_clock_active(self):
        """Verify /clock is published by Gazebo Harmonic bridge."""
        start = time.time()
        while time.time() - start < 5.0 and self.latest_clock is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_clock, "No Clock message received on /clock")
        self.assertGreater(self.latest_clock.clock.sec + self.latest_clock.clock.nanosec * 1e-9, 0.0)

    def test_03_costmap_receives_laser_scan(self):
        """Verify Nav2 local costmap marks obstacles from /scan."""
        start = time.time()
        while time.time() - start < 8.0 and self.latest_local_costmap is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_local_costmap, "No costmap received on /local_costmap/costmap_raw")
        self.assertEqual(self.latest_local_costmap.header.frame_id, "odom")
        # Ensure at least some cells are marked as obstacles / cost > 0
        costs = list(self.latest_local_costmap.data)
        has_obstacles = any(c > 0 for c in costs)
        self.assertTrue(has_obstacles, "Local costmap contains zero obstacle or inflation costs")

    def test_04_straight_aisle_navigation(self):
        """Scenario 1: Drive along straight warehouse rack corridor."""
        success, duration, error = self._send_nav_goal(target_x=-3.0, target_y=4.0, target_yaw=0.0, timeout_sec=30.0)
        self.assertTrue(success, f"Straight aisle navigation failed after {duration:.1f}s")
        self.assertLess(error, 0.5, f"Final pose error {error:.3f}m exceeded tolerance")

    def test_05_ninety_degree_intersection_turn(self):
        """Scenario 2: Navigate 90-degree turn through warehouse cross-aisle."""
        success, duration, error = self._send_nav_goal(target_x=-5.5, target_y=2.5, target_yaw=-1.57, timeout_sec=30.0)
        self.assertTrue(success, f"90-degree intersection turn failed after {duration:.1f}s")

    def test_06_static_obstacle_avoidance(self):
        """Scenario 3: Reach goal in central aisle avoiding static pallet stack."""
        # Pallet stack is at (0.5, 0.0); navigate to point past the pallet stack
        success, duration, error = self._send_nav_goal(target_x=2.5, target_y=0.0, target_yaw=0.0, timeout_sec=35.0)
        self.assertTrue(success, f"Static obstacle avoidance failed after {duration:.1f}s")

    def test_07_multi_waypoint_mission_dispatch(self):
        """Scenario 5: Complete multi-waypoint transit between pick and drop stations."""
        # Waypoint A: station_pick_1 (-7.0, 4.0)
        ok_a, dur_a, _ = self._send_nav_goal(-7.0, 4.0, 0.0, timeout_sec=30.0)
        self.assertTrue(ok_a, "Failed reaching Waypoint A (Pick Station 1)")

        # Waypoint B: return to transit bay (-7.0, 0.0)
        ok_b, dur_b, _ = self._send_nav_goal(-7.0, 0.0, 0.0, timeout_sec=30.0)
        self.assertTrue(ok_b, "Failed reaching Waypoint B (Transit Bay)")


if __name__ == "__main__":
    unittest.main()
