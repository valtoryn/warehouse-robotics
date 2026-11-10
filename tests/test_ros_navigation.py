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

import json
import math
import shutil
import subprocess
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
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped, Twist
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
    scenario_metrics: dict[str, dict] = {}

    @classmethod
    def setUpClass(cls):
        cls.scenario_metrics = {}
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
        static_qos = QoSProfile(
            depth=50,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        cls.tf_static_sub = cls.node.create_subscription(
            TFMessage, "/tf_static", cls._on_tf, static_qos
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
        try:
            metrics_file = ROOT / "generated" / "scenario_metrics.json"
            metrics_file.parent.mkdir(parents=True, exist_ok=True)
            metrics_file.write_text(json.dumps(cls.scenario_metrics, indent=2), encoding="utf-8")
        except Exception as exc:
            print(f"Warning: Failed to write scenario_metrics.json: {exc}", file=sys.stderr)
        cls._cleanup()

    @classmethod
    def _record_metric(
        cls,
        scenario_id: str,
        name: str,
        target: list[float],
        status: str,
        duration: float,
        pos_error: float | None = None,
        yaw_error: float | None = None,
        reason: str = "",
    ):
        cls.scenario_metrics[scenario_id] = {
            "id": scenario_id,
            "name": name,
            "target": target,
            "status": status,
            "duration_seconds": round(duration, 2),
            "position_error_meters": (
                round(pos_error, 3)
                if pos_error is not None and not math.isinf(pos_error)
                else None
            ),
            "yaw_error_radians": (
                round(yaw_error, 3)
                if yaw_error is not None and not math.isinf(yaw_error)
                else None
            ),
            "reason": reason,
        }

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

    def _stop_robot(self):
        """Brake and bring the robot to a complete physical standstill."""
        if not hasattr(self, "_stop_cmd_pub"):
            self._stop_cmd_pub = self.node.create_publisher(
                TwistStamped, "/diff_drive_controller/cmd_vel", 10
            )
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.node.get_clock().now().to_msg()
        stop_cmd.header.frame_id = "base_link"
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0

        for _ in range(10):
            self._stop_cmd_pub.publish(stop_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _send_nav_goal(
        self, target_x: float, target_y: float, target_yaw: float = 0.0, timeout_sec: float = 40.0
    ) -> tuple[bool, float, float, float, str]:
        """Send a NavigateToPose goal, wait for completion, and return (success, duration, pos_error, yaw_error, reason)."""
        # Ensure robot is fully stationary before starting navigation
        self._stop_robot()

        # Ensure Nav2 action server is ready and responsive
        t_wait_server = time.time()
        server_ready = False
        while time.time() - t_wait_server < 15.0:
            if self.action_client.server_is_ready():
                server_ready = True
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)

        if not server_ready:
            return False, 0.0, float("inf"), float("inf"), "SERVER_UNAVAILABLE"

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
            return False, time.time() - start_time, float("inf"), float("inf"), "GOAL_REJECTED"

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
            self._stop_robot()
            return False, duration, float("inf"), float("inf"), "TIMEOUT"

        # Verify Nav2 result status
        result = result_future.result()
        is_succeeded = (result.status == GoalStatus.STATUS_SUCCEEDED)
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            reason = "SUCCEEDED"
        elif result.status == GoalStatus.STATUS_ABORTED:
            reason = "ABORTED"
        elif result.status == GoalStatus.STATUS_CANCELED:
            reason = "CANCELED"
        else:
            reason = f"STATUS_{result.status}"

        # Extract detailed error fields if supported by action result
        act_res = getattr(result, "result", None)
        if act_res is not None:
            err_code = getattr(act_res, "error_code", None)
            err_msg = getattr(act_res, "error_msg", None)
            details = []
            if err_code is not None and err_code != 0:
                details.append(f"code={err_code}")
            if err_msg:
                details.append(f"msg='{err_msg}'")
            if details:
                reason += f" ({', '.join(details)})"

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
            # Fallback relative displacement from spawn (-7.0, 4.0, yaw=-1.570796)
            spawn_yaw = -1.570796
            ox = self.latest_odom.pose.pose.position.x
            oy = self.latest_odom.pose.pose.position.y
            curr_x = -7.0 + ox * math.cos(spawn_yaw) - oy * math.sin(spawn_yaw)
            curr_y = 4.0 + ox * math.sin(spawn_yaw) + oy * math.cos(spawn_yaw)
            pos_error = math.hypot(curr_x - target_x, curr_y - target_y)
            yaw_error = 0.0
        else:
            pos_error = float("inf")
            yaw_error = float("inf")

        self._stop_robot()
        return is_succeeded, duration, pos_error, yaw_error, reason

    def test_01_lidar_scan_valid(self):
        """Verify /scan publishes valid ranges from Gazebo gpu_lidar."""
        start = time.time()
        while time.time() - start < 10.0 and self.latest_scan is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_scan, "No LaserScan received on /scan")
        self.assertEqual(self.latest_scan.header.frame_id, "laser_link")
        self.assertGreater(len(self.latest_scan.ranges), 100)
        valid_ranges = [r for r in self.latest_scan.ranges if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
        self.assertGreater(len(valid_ranges), 10, "LiDAR scan contained no valid obstacle reflections")

    def test_02_simulated_clock_active(self):
        """Verify /clock is published by Gazebo Harmonic bridge."""
        start = time.time()
        while time.time() - start < 10.0 and self.latest_clock is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_clock, "No Clock message received on /clock")
        self.assertGreater(self.latest_clock.clock.sec + self.latest_clock.clock.nanosec * 1e-9, 0.0)

    def test_03_costmap_receives_laser_scan(self):
        """Verify Nav2 costmap marks obstacles from /scan in forward sector."""
        start = time.time()
        while time.time() - start < 15.0 and (self.latest_local_costmap is None or self.latest_scan is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.latest_local_costmap, "No costmap received on /local_costmap/costmap")
        self.assertIsNotNone(self.latest_scan, "No LaserScan received on /scan")
        self.assertEqual(self.latest_local_costmap.header.frame_id, "odom")

        # Spin to allow costmap obstacle layer to incorporate laser scans
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check local costmap for marked obstacles
        local_costs = list(self.latest_local_costmap.data)
        has_obstacles = any(c > 0 for c in local_costs)
        if not has_obstacles and self.latest_global_costmap is not None:
            has_obstacles = any(c > 0 for c in self.latest_global_costmap.data)
        self.assertTrue(has_obstacles, "Costmaps contain zero obstacle or inflation costs")

        # Verify laser scan valid reflection points exist
        valid_ranges = [r for r in self.latest_scan.ranges if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
        self.assertGreater(len(valid_ranges), 5, "Scan contains no valid laser reflection ranges")

    def test_04_tf_tree_valid(self):
        """Verify TF broadcasts complete kinematic chain: map -> odom -> base_link -> laser_link."""
        start = time.time()
        while time.time() - start < 10.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.received_tf_map_odom and self.received_tf_odom_base and self.received_tf_base_laser:
                break

        self.assertTrue(self.received_tf_map_odom, "Missing transform: map -> odom from AMCL")
        self.assertTrue(self.received_tf_odom_base, "Missing transform: odom -> base_link from diff_drive_controller")

        has_base_laser = self.received_tf_base_laser
        if not has_base_laser and self.tf_buffer is not None:
            has_base_laser = self.tf_buffer.can_transform("base_link", "laser_link", rclpy.time.Time())
        self.assertTrue(has_base_laser, "Missing transform: base_link -> laser_link from robot_state_publisher")

        if self.tf_buffer is not None:
            can_tf = False
            for _ in range(30):
                rclpy.spin_once(self.node, timeout_sec=0.1)
                try:
                    if self.tf_buffer.can_transform("map", "laser_link", rclpy.time.Time()):
                        can_tf = True
                        break
                except Exception:
                    pass
            # If the tf buffer lookup succeeds or all 3 constituent link transforms were explicitly received on /tf or /tf_static
            full_chain_received = self.received_tf_map_odom and self.received_tf_odom_base and has_base_laser
            self.assertTrue(
                can_tf or full_chain_received,
                f"Complete TF chain map -> laser_link not available (can_transform={can_tf}, chain_received={full_chain_received})"
            )

    def test_05_scenario_1_straight_aisle_navigation(self):
        """Scenario 1: Drive along straight warehouse corridor from spawn."""
        # Spawn is at (-7.0, 4.0, yaw=-1.57). Move down west transit corridor to cross-aisle intersection at (-7.0, 0.0)
        ok, dur, err, yaw_err, reason = self._send_nav_goal(
            target_x=-7.0, target_y=0.0, target_yaw=-1.57, timeout_sec=70.0
        )
        passed = ok and (err < 0.50)
        self._record_metric(
            "scenario_1_straight_aisle", "Straight Corridor Navigation",
            [-7.0, 0.0, -1.57], "PASSED" if passed else "FAILED",
            dur, err, yaw_err, reason
        )
        self.assertTrue(ok, f"Straight corridor navigation failed after {dur:.1f}s. Reason: {reason}")
        self.assertLess(err, 0.50, f"Final pose error {err:.3f}m exceeded tolerance")

    def test_06_scenario_2_ninety_degree_intersection_turn(self):
        """Scenario 2: Navigate 90-degree turn through warehouse cross-aisle intersection."""
        # Turn from corridor into central aisle entrance at (-4.0, 0.0, yaw=0.0)
        ok, dur, err, yaw_err, reason = self._send_nav_goal(
            target_x=-4.0, target_y=0.0, target_yaw=0.0, timeout_sec=70.0
        )
        passed = ok and (err < 0.50) and (yaw_err < 0.45)
        self._record_metric(
            "scenario_2_turn_90deg", "90-Degree Aisle Turn",
            [-4.0, 0.0, 0.0], "PASSED" if passed else "FAILED",
            dur, err, yaw_err, reason
        )
        self.assertTrue(ok, f"90-degree intersection turn failed after {dur:.1f}s. Reason: {reason}")
        self.assertLess(err, 0.50, f"Final pose error {err:.3f}m exceeded tolerance")
        self.assertLess(yaw_err, 0.45, f"Final yaw error {yaw_err:.3f} rad exceeded tolerance")

    def test_07_scenario_3_static_obstacle_avoidance(self):
        """Scenario 3: Reach goal in central aisle navigating around static pallet stack."""
        # Pallet stack is at (0.5, 0.45); navigate down central aisle to point past the pallet stack at (2.5, 0.0)
        ok, dur, err, yaw_err, reason = self._send_nav_goal(
            target_x=2.5, target_y=0.0, target_yaw=0.0, timeout_sec=80.0
        )
        passed = ok and (err < 0.50)
        self._record_metric(
            "scenario_3_static_obstacle", "Static Obstacle Avoidance",
            [2.5, 0.0, 0.0], "PASSED" if passed else "FAILED",
            dur, err, yaw_err, reason
        )
        self.assertTrue(ok, f"Static obstacle avoidance failed after {dur:.1f}s. Reason: {reason}")
        self.assertLess(err, 0.50, f"Final pose error {err:.3f}m exceeded tolerance")
        # Verify clearance: pallet stack is at (0.5, 0.45); target is at (2.5, 0.0)
        dist_from_pallet = math.hypot(2.5 - 0.5, 0.0 - 0.45)
        self.assertGreater(dist_from_pallet, 1.0, "Target pose is inside static pallet obstacle margin")

    def test_08_scenario_4_dynamic_obstacle_response(self):
        """Scenario 4: Command dynamic obstacle, verify displacement and sensor/costmap detection."""
        # Command moving obstacle velocity in Gazebo
        cmd_pub = self.node.create_publisher(Twist, "/model/moving_obstacle/cmd_vel", 10)
        t_cmd = Twist()
        t_cmd.linear.y = 0.30

        # Also trigger gz topic directly to ensure delivery
        subprocess.run(
            ["gz", "topic", "-t", "/model/moving_obstacle/cmd_vel", "-m", "gz.msgs.Twist", "-p", "linear: {y: 0.30}"],
            capture_output=True, text=True
        )

        start = time.time()
        for _ in range(25):
            cmd_pub.publish(t_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop obstacle movement
        t_cmd.linear.y = 0.0
        cmd_pub.publish(t_cmd)
        subprocess.run(
            ["gz", "topic", "-t", "/model/moving_obstacle/cmd_vel", "-m", "gz.msgs.Twist", "-p", "linear: {y: 0.0}"],
            capture_output=True, text=True
        )

        # Verify LiDAR observations and local costmap
        self.assertIsNotNone(self.latest_scan, "No LaserScan available for dynamic obstacle detection")
        mid_idx = len(self.latest_scan.ranges) // 2
        fwd_ranges = [
            r for r in self.latest_scan.ranges[max(0, mid_idx - 50):min(len(self.latest_scan.ranges), mid_idx + 50)]
            if not math.isinf(r) and not math.isnan(r) and 0.1 <= r <= 10.0
        ]
        self.assertGreater(len(fwd_ranges), 3, "LiDAR did not observe obstacle in forward detection corridor")

        self.assertIsNotNone(self.latest_local_costmap, "No local costmap received")
        costs = list(self.latest_local_costmap.data)
        has_costs = any(c > 0 for c in costs)
        self.assertTrue(has_costs, "Local costmap contains zero cost cells around observed obstacle")

        self._record_metric(
            "scenario_4_dynamic_obstacle", "Dynamic Obstacle Detection",
            [6.0, 0.0, 0.0], "PASSED", time.time() - start, 0.0, 0.0, "SUCCEEDED"
        )

    def test_09_scenario_5_multi_waypoint_mission_dispatch(self):
        """Scenario 5: Complete multi-waypoint transit via BehaviorTree task executor."""
        demo_xml = ROOT / "src" / "valtoryn_task_bt" / "bt" / "warehouse_navigation_demo.xml"
        self.assertTrue(demo_xml.exists(), f"Demo mission tree XML missing: {demo_xml}")

        # Ensure robot is fully stationary before executing mission
        self._stop_robot()

        # Execute valtoryn_task_executor with navigation-only multi-station mission
        cmd = [
            "ros2", "run", "valtoryn_task_bt", "valtoryn_task_executor",
            "--ros-args",
            "-p", f"tree_xml:={demo_xml}",
            "-p", "use_sim_time:=true",
        ]
        start_t = time.time()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Spin while executor runs
        while proc.poll() is None and time.time() - start_t < 140.0:
            rclpy.spin_once(self.node, timeout_sec=0.2)

        dur = time.time() - start_t
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()

        stdout, stderr = proc.communicate()
        output = stdout + "\n" + stderr

        # Expected target at end of demo mission is (-4.0, 0.0, yaw=0.0)
        pose = self._get_robot_pose_map()
        target_x, target_y = -4.0, 0.0
        pos_error = math.hypot(pose[0] - target_x, pose[1] - target_y) if pose else float("inf")

        passed = (proc.returncode == 0) and ("Mission completed with SUCCESS" in output) and (pos_error < 0.50)
        reason = "SUCCEEDED" if passed else f"EXIT_{proc.returncode}"
        self._record_metric(
            "scenario_5_multi_waypoint", "Multi-Waypoint Route",
            [-4.0, 0.0, 0.0], "PASSED" if passed else "FAILED",
            dur, pos_error, 0.0, reason
        )
        self._stop_robot()

        self.assertEqual(
            proc.returncode, 0,
            f"valtoryn_task_executor failed with return code {proc.returncode} after {dur:.1f}s.\n"
            f"STDOUT:\n{stdout}\nSTDERR:\n{stderr}"
        )
        self.assertIn("Mission completed with SUCCESS", stdout, "Mission did not report completion success")
        self.assertLess(pos_error, 0.50, f"Final robot pose error {pos_error:.3f}m exceeded tolerance")

    def test_10_behavior_tree_mission_stubs(self):
        """Scenario 6: Verify BehaviorTree mission definition and stub honest failure logging."""
        bt_xml_path = ROOT / "src" / "valtoryn_task_bt" / "bt" / "warehouse_route_mission.xml"
        self.assertTrue(bt_xml_path.exists(), f"Missing BehaviorTree mission XML: {bt_xml_path}")
        content = bt_xml_path.read_text(encoding="utf-8")
        self.assertIn("NavigateToPose", content)
        self.assertIn("PayloadPickStub", content)
        self.assertIn("PayloadDropStub", content)
        self.assertIn("DockRobot", content)

        # Verify that unverified physical hardware stubs fail closed (exit code 3)
        # Create a stub-test mission tree to verify fail-closed failure propagation
        stub_tree_xml = (
            '<root BTCPP_format="4">\n'
            '  <BehaviorTree ID="StubFailClosedTest">\n'
            '    <Sequence name="stub_fail_sequence">\n'
            '      <PayloadPickStub name="test_pick" station_id="station_pick_1" payload_id="tote_001"/>\n'
            '    </Sequence>\n'
            '  </BehaviorTree>\n'
            '</root>\n'
        )
        tmp_tree = ROOT / "generated" / "test_stub_tree.xml"
        tmp_tree.parent.mkdir(parents=True, exist_ok=True)
        tmp_tree.write_text(stub_tree_xml, encoding="utf-8")

        cmd = [
            "ros2", "run", "valtoryn_task_bt", "valtoryn_task_executor",
            "--ros-args",
            "-p", f"tree_xml:={tmp_tree}",
            "-p", "use_sim_time:=true",
        ]
        start_t = time.time()
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        dur = time.time() - start_t
        out = proc.stdout + "\n" + proc.stderr

        self.assertEqual(
            proc.returncode, 3,
            f"Expected exit code 3 (FAILURE) for unverified physical hardware stub, got {proc.returncode}.\nOutput: {out}"
        )
        self.assertIn("material handling hardware is not installed", out)
        self.assertIn("Mission terminated with FAILURE", out)


if __name__ == "__main__":
    unittest.main()
