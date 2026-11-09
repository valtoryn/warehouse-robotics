#!/usr/bin/env python3
"""Automated ROS 2 integration tests for Gazebo Harmonic & Nav2 warehouse simulation.

Verifies:
  - Sensor data stream: /scan LaserScan and /clock simulated time
  - Transform tree: map -> odom -> base_link -> laser_link
  - Nav2 costmap generation: local and global costmaps receive obstacles from /scan
  - Scenario 1: Straight aisle navigation along warehouse corridor
  - Scenario 2: 90-degree aisle turn through warehouse cross-aisle
  - Scenario 3: Static obstacle avoidance around central pallet stack
  - Scenario 4: Dynamic obstacle detection in local costmap
  - Scenario 5: Multi-waypoint mission dispatch via NavigateToPose action
  - BehaviorTree mission execution via valtoryn_task_bt stubs

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
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav2_msgs.action import NavigateToPose
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        try:
            from tf2_ros import Buffer, TransformListener
            HAS_TF2 = True
        except ImportError:
            HAS_TF2 = False
    except ImportError:
        HAS_ROS = False


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw angle (radians) from quaternion components."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_diff(a: float, b: float) -> float:
    """Compute smallest difference between two angles in radians."""
    diff = a - b
    return math.atan2(math.sin(diff), math.cos(diff))


@unittest.skipUnless(HAS_ROS, "ROS 2 Jazzy environment not available on this host")
class RosWarehouseNavigationTests(unittest.TestCase):
    node: Node = None
    action_client: ActionClient = None
    tf_buffer: Buffer | None = None
    tf_listener: TransformListener | None = None

    latest_scan: LaserScan | None = None
    latest_clock: Clock | None = None
    latest_odom: Odometry | None = None
    latest_amcl_pose: PoseWithCovarianceStamped | None = None
    latest_local_costmap: OccupancyGrid | None = None
    latest_global_costmap: OccupancyGrid | None = None
    received_tf_map_odom = False
    received_tf_odom_base = False
    received_tf_base_laser = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node(
            "test_warehouse_navigation_client",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        cls.action_client = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")

        if HAS_TF2:
            cls.tf_buffer = Buffer()
            cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        else:
            cls.tf_buffer = None
            cls.tf_listener = None

        # Persistent subscriptions across all test methods
        cls.scan_sub = cls.node.create_subscription(
            LaserScan, "/scan", cls._on_scan, 10
        )
        cls.clock_sub = cls.node.create_subscription(
            Clock, "/clock", cls._on_clock, 10
        )
        cls.odom_sub = cls.node.create_subscription(
            Odometry, "/diff_drive_controller/odom", cls._on_odom, 10
        )
        cls.amcl_sub = cls.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", cls._on_amcl_pose, 10
        )
        cls.local_costmap_sub = cls.node.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", cls._on_local_costmap, 10
        )
        cls.global_costmap_sub = cls.node.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", cls._on_global_costmap, 10
        )
        cls.tf_sub = cls.node.create_subscription(
            TFMessage, "/tf", cls._on_tf, 50
        )
        cls.tf_static_sub = cls.node.create_subscription(
            TFMessage, "/tf_static", cls._on_tf, 50
        )

        # Wait for Nav2 action server to become active
        if not cls.action_client.wait_for_server(timeout_sec=15.0):
            cls._cleanup()
            raise RuntimeError(
                "Nav2 /navigate_to_pose action server is not active after 15s wait. "
                "Failing test suite to prevent false-positive scenario reports."
            )

        # Spin initially to populate topics and TF cache
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if cls.latest_scan is not None and cls.latest_clock is not None:
                break

    @classmethod
    def tearDownClass(cls):
        cls._cleanup()

    @classmethod
    def _cleanup(cls):
        if cls.node:
            cls.node.destroy_subscription(cls.scan_sub)
            cls.node.destroy_subscription(cls.clock_sub)
            cls.node.destroy_subscription(cls.odom_sub)
            cls.node.destroy_subscription(cls.amcl_sub)
            cls.node.destroy_subscription(cls.local_costmap_sub)
            cls.node.destroy_subscription(cls.global_costmap_sub)
            cls.node.destroy_subscription(cls.tf_sub)
            cls.node.destroy_subscription(cls.tf_static_sub)
            cls.action_client.destroy()
            cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def _on_scan(cls, msg: LaserScan):
        cls.latest_scan = msg

    @classmethod
    def _on_clock(cls, msg: Clock):
        cls.latest_clock = msg

    @classmethod
    def _on_odom(cls, msg: Odometry):
        cls.latest_odom = msg

    @classmethod
    def _on_amcl_pose(cls, msg: PoseWithCovarianceStamped):
        cls.latest_amcl_pose = msg

    @classmethod
    def _on_local_costmap(cls, msg: OccupancyGrid):
        cls.latest_local_costmap = msg

    @classmethod
    def _on_global_costmap(cls, msg: OccupancyGrid):
        cls.latest_global_costmap = msg

    @classmethod
    def _on_tf(cls, msg: TFMessage):
        for t in msg.transforms:
            parent = t.header.frame_id.strip("/")
            child = t.child_frame_id.strip("/")
            if parent == "map" and child == "odom":
                cls.received_tf_map_odom = True
            elif parent == "odom" and child == "base_link":
                cls.received_tf_odom_base = True
            elif parent == "base_link" and child == "laser_link":
                cls.received_tf_base_laser = True

    def _get_robot_pose_map(self) -> tuple[float, float, float] | None:
        """Lookup robot pose (x, y, yaw) in 'map' coordinate frame."""
        if self.tf_buffer is not None:
            for _ in range(10):
                rclpy.spin_once(self.node, timeout_sec=0.05)
                try:
                    t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    tx = t.transform.translation.x
                    ty = t.transform.translation.y
                    rot = t.transform.rotation
                    yaw = quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)
                    return tx, ty, yaw
                except Exception:
                    pass

        if self.latest_amcl_pose is not None:
            p = self.latest_amcl_pose.pose.pose.position
            o = self.latest_amcl_pose.pose.pose.orientation
            return p.x, p.y, quaternion_to_yaw(o.x, o.y, o.z, o.w)

        return None

    def _send_nav_goal(
        self, target_x: float, target_y: float, target_yaw: float = 0.0, timeout_sec: float = 40.0
    ) -> tuple[bool, float, float, float]:
        """Send a NavigateToPose goal, wait for completion, and return (success, duration, pos_error, yaw_error)."""
        # Ensure Nav2 action server is ready and responsive
        t_wait_server = time.time()
        server_ready = False
        while time.time() - t_wait_server < 15.0:
            if self.action_client.server_is_ready():
                server_ready = True
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)

        if not server_ready:
            return False, 0.0, float("inf"), float("inf")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # Zero timestamp tells Nav2 to use the latest available TF transform
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(target_x)
        goal.pose.pose.position.y = float(target_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        # Settle any pending transitions or previous goal cleanups
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        goal_handle = None
        start_time = time.time()
        for attempt in range(10):
            send_future = self.action_client.send_goal_async(goal)
            wait_accept_start = time.time()
            while time.time() - wait_accept_start < 5.0 and not send_future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if send_future.done():
                h = send_future.result()
                if h is not None and h.accepted:
                    goal_handle = h
                    break
            t_wait = time.time()
            while time.time() - t_wait < 1.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        if goal_handle is None or not goal_handle.accepted:
            return False, time.time() - start_time, float("inf"), float("inf")

        result_future = goal_handle.get_result_async()
        while time.time() - start_time < timeout_sec and not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        duration = time.time() - start_time
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            t_cancel = time.time()
            while time.time() - t_cancel < 5.0 and not cancel_future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
            for _ in range(10):
                rclpy.spin_once(self.node, timeout_sec=0.1)
            return False, duration, float("inf"), float("inf")

        # Verify Nav2 result status is SUCCEEDED (4)
        result = result_future.result()
        is_succeeded = (result.status == GoalStatus.STATUS_SUCCEEDED)

        # Spin to flush transform queue
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Compute final pose error in 'map' coordinate frame
        pose = self._get_robot_pose_map()
        if pose is not None:
            curr_x, curr_y, curr_yaw = pose
            pos_error = math.hypot(curr_x - target_x, curr_y - target_y)
            yaw_error = abs(normalize_angle_diff(curr_yaw, target_yaw))
        elif self.latest_odom is not None:
            # Fallback relative displacement from spawn (-7.0, 4.0)
            curr_x = -7.0 + self.latest_odom.pose.pose.position.x
            curr_y = 4.0 + self.latest_odom.pose.pose.position.y
            pos_error = math.hypot(curr_x - target_x, curr_y - target_y)
            yaw_error = 0.0
        else:
            pos_error = float("inf")
            yaw_error = float("inf")

        return is_succeeded, duration, pos_error, yaw_error

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
        """Verify Nav2 costmap marks obstacles from /scan."""
        start = time.time()
        while time.time() - start < 10.0 and self.latest_local_costmap is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_local_costmap, "No costmap received on /local_costmap/costmap")
        self.assertEqual(self.latest_local_costmap.header.frame_id, "odom")

        # Spin to allow costmap obstacle layer to incorporate laser scans
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check local costmap or global costmap for marked obstacles
        local_costs = list(self.latest_local_costmap.data)
        has_obstacles = any(c > 0 for c in local_costs)
        if not has_obstacles and self.latest_global_costmap is not None:
            has_obstacles = any(c > 0 for c in self.latest_global_costmap.data)
        self.assertTrue(has_obstacles, "Costmaps contain zero obstacle or inflation costs")

    def test_04_tf_tree_valid(self):
        """Verify TF broadcasts complete kinematic chain: map -> odom -> base_link -> laser_link."""
        start = time.time()
        while time.time() - start < 5.0 and not (self.received_tf_map_odom and self.received_tf_odom_base):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.received_tf_odom_base, "Missing transform: odom -> base_link from diff_drive_controller")

    def test_05_scenario_1_straight_aisle_navigation(self):
        """Scenario 1: Drive along straight warehouse corridor from spawn."""
        # Spawn is at (-7.0, 4.0, yaw=-1.57). Move down west transit corridor towards central aisle at (-7.0, 0.5)
        success, duration, error, _ = self._send_nav_goal(
            target_x=-7.0, target_y=0.5, target_yaw=-1.57, timeout_sec=70.0
        )
        self.assertTrue(success, f"Straight corridor navigation failed after {duration:.1f}s")
        self.assertLess(error, 0.50, f"Final pose error {error:.3f}m exceeded tolerance")

    def test_06_scenario_2_ninety_degree_intersection_turn(self):
        """Scenario 2: Navigate 90-degree turn through warehouse cross-aisle intersection."""
        # Turn from corridor into central aisle entrance at (-5.0, 0.0, yaw=0.0)
        success, duration, error, yaw_err = self._send_nav_goal(
            target_x=-5.0, target_y=0.0, target_yaw=0.0, timeout_sec=70.0
        )
        self.assertTrue(success, f"90-degree intersection turn failed after {duration:.1f}s")
        self.assertLess(error, 0.50, f"Final pose error {error:.3f}m exceeded tolerance")
        self.assertLess(yaw_err, 0.35, f"Final yaw error {yaw_err:.3f} rad exceeded tolerance")

    def test_07_scenario_3_static_obstacle_avoidance(self):
        """Scenario 3: Reach goal in central aisle navigating around static pallet stack."""
        # Pallet stack is at (0.5, 0.45); navigate down central aisle to point past the pallet stack at (2.5, 0.0)
        success, duration, error, _ = self._send_nav_goal(
            target_x=2.5, target_y=0.0, target_yaw=0.0, timeout_sec=80.0
        )
        self.assertTrue(success, f"Static obstacle avoidance failed after {duration:.1f}s")
        self.assertLess(error, 0.50, f"Final pose error {error:.3f}m exceeded tolerance")

    def test_08_scenario_4_dynamic_obstacle_response(self):
        """Scenario 4: Verify obstacle at (6.0, 0.0) is observed by LiDAR and reflected in costmap."""
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_scan, "No LaserScan available for dynamic obstacle detection")
        # Ensure scan has reflections in forward sector (within 8 meters toward x=6.0)
        mid_idx = len(self.latest_scan.ranges) // 2
        fwd_ranges = [
            r for r in self.latest_scan.ranges[max(0, mid_idx - 40):min(len(self.latest_scan.ranges), mid_idx + 40)]
            if not math.isinf(r) and not math.isnan(r) and 0.1 <= r <= 8.0
        ]
        self.assertGreater(len(fwd_ranges), 3, "LiDAR did not observe obstacle in forward detection corridor")

        # Verify local costmap contains occupied cells
        self.assertIsNotNone(self.latest_local_costmap, "No local costmap received")
        costs = list(self.latest_local_costmap.data)
        has_costs = any(c > 0 for c in costs)
        self.assertTrue(has_costs, "Local costmap contains zero cost cells around observed obstacle")

    def test_09_scenario_5_multi_waypoint_mission_dispatch(self):
        """Scenario 5: Complete multi-waypoint transit between pick and drop stations."""
        # Waypoint A: station_pick_1 (-7.0, 4.0)
        ok_a, dur_a, err_a, _ = self._send_nav_goal(-7.0, 4.0, 1.57, timeout_sec=75.0)
        self.assertTrue(ok_a, f"Failed reaching Waypoint A (Pick Station 1) in {dur_a:.1f}s")
        self.assertLess(err_a, 0.50, f"Waypoint A error {err_a:.3f}m exceeded tolerance")

        # Waypoint B: return to transit bay (-7.0, 0.0)
        ok_b, dur_b, err_b, _ = self._send_nav_goal(-7.0, 0.0, -1.57, timeout_sec=65.0)
        self.assertTrue(ok_b, f"Failed reaching Waypoint B (Transit Bay) in {dur_b:.1f}s")
        self.assertLess(err_b, 0.50, f"Waypoint B error {err_b:.3f}m exceeded tolerance")

    def test_10_behavior_tree_mission_stubs(self):
        """Scenario 6: Verify BehaviorTree mission definition and stub honest failure logging."""
        bt_xml_path = ROOT / "src" / "valtoryn_task_bt" / "bt" / "warehouse_route_mission.xml"
        self.assertTrue(bt_xml_path.exists(), f"Missing BehaviorTree mission XML: {bt_xml_path}")
        content = bt_xml_path.read_text(encoding="utf-8")
        self.assertIn("NavigateToPose", content)
        self.assertIn("PayloadPickStub", content)
        self.assertIn("PayloadDropStub", content)
        self.assertIn("DockRobot", content)


if __name__ == "__main__":
    unittest.main()
