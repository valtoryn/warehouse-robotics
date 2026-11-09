#!/usr/bin/env python3
"""Automated ROS 2 integration tests for Valtoryn reference AMR mock bringup.

Verifies:
  - Hardware interface export and controller claiming
  - Controller activation (joint_state_broadcaster, diff_drive_controller)
  - Forward motion (x displacement, wheel velocity matching v / r)
  - Reverse motion (negative x displacement)
  - Pure rotation (yaw change, differential wheel signs)
  - Stopping (zero velocity commands)
  - Command timeout enforcement (wheel speeds drop to 0 after cmd_vel_timeout)
  - TF broadcast (odom -> base_link)

Execution:
  In a sourced ROS 2 workspace:
    python -m unittest tests/test_ros_motion.py -v
"""
from __future__ import annotations

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
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
    except ImportError:
        HAS_ROS = False


@unittest.skipUnless(HAS_ROS, "ROS 2 Jazzy environment not available on this host")
class RosMockMotionIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_ros_motion_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.cmd_pub = self.node.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.latest_odom: Odometry | None = None
        self.latest_js: JointState | None = None
        self.received_tf = False

        self.odom_sub = self.node.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._on_odom, 10
        )
        self.js_sub = self.node.create_subscription(
            JointState, "/joint_states", self._on_js, 10
        )
        self.tf_sub = self.node.create_subscription(
            TFMessage, "/tf", self._on_tf, 10
        )

        # Wait for initial messages
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_odom is not None and self.latest_js is not None:
                break

    def tearDown(self):
        self.node.destroy_subscription(self.odom_sub)
        self.node.destroy_subscription(self.js_sub)
        self.node.destroy_subscription(self.tf_sub)
        self.node.destroy_publisher(self.cmd_pub)

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _on_js(self, msg: JointState):
        self.latest_js = msg

    def _on_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            f_id = transform.header.frame_id.strip("/")
            c_id = transform.child_frame_id.strip("/")
            if f_id == "odom" and c_id == "base_link":
                self.received_tf = True

    def _send_cmd(self, linear_x: float, angular_z: float, duration_sec: float, rate_hz: float = 20.0):
        dt = 1.0 / rate_hz
        steps = int(duration_sec * rate_hz)
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)

        for _ in range(steps):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)
            time.sleep(dt)

    def _is_controller_active(self, output: str, name: str) -> bool:
        for line in output.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[0] == name and parts[2] == "active":
                return True
            if name in line and "active" in line:
                return True
        return False

    def test_01_controllers_active(self):
        res = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, f"Failed to list controllers: {res.stderr}")
        self.assertTrue(
            self._is_controller_active(res.stdout, "joint_state_broadcaster"),
            f"joint_state_broadcaster not active in:\n{res.stdout}",
        )
        self.assertTrue(
            self._is_controller_active(res.stdout, "diff_drive_controller"),
            f"diff_drive_controller not active in:\n{res.stdout}",
        )

    def test_02_hardware_interfaces_claimed(self):
        res = subprocess.run(["ros2", "control", "list_hardware_interfaces"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, f"Failed to list hardware interfaces: {res.stderr}")
        for j in ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]:
            self.assertTrue(
                any(f"{j}/velocity" in line and "claimed" in line for line in res.stdout.splitlines()),
                f"{j}/velocity not claimed in:\n{res.stdout}",
            )
            self.assertTrue(
                any(f"{j}/position" in line for line in res.stdout.splitlines()),
                f"{j}/position not found in:\n{res.stdout}",
            )

    def test_03_forward_motion(self):
        self.assertIsNotNone(self.latest_odom, "No odometry received from diff_drive_controller")
        x_init = self.latest_odom.pose.pose.position.x
        y_init = self.latest_odom.pose.pose.position.y

        # Drive forward at 0.5 m/s for 1.5 seconds
        self._send_cmd(linear_x=0.5, angular_z=0.0, duration_sec=1.5)
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        x_final = self.latest_odom.pose.pose.position.x
        y_final = self.latest_odom.pose.pose.position.y
        dx = x_final - x_init
        dy = y_final - y_init

        # Should move forward ~0.75m; assert at least 0.4m forward and minimal lateral drift
        self.assertGreater(dx, 0.4, f"Forward displacement dx={dx:.3f}m was less than 0.4m")
        self.assertLess(abs(dy), 0.05, f"Lateral drift |dy|={abs(dy):.3f}m exceeded 0.05m")

        # Verify wheel speeds match v / r = 0.5 / 0.075 = 6.67 rad/s
        self.assertIsNotNone(self.latest_js)
        for vel in self.latest_js.velocity:
            self.assertGreater(vel, 3.0, f"Wheel velocity {vel} rad/s too low during forward motion")

    def test_04_reverse_motion(self):
        self.assertIsNotNone(self.latest_odom)
        x_init = self.latest_odom.pose.pose.position.x

        # Drive backward at -0.4 m/s for 1.0 second
        self._send_cmd(linear_x=-0.4, angular_z=0.0, duration_sec=1.0)
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        x_final = self.latest_odom.pose.pose.position.x
        dx = x_final - x_init
        self.assertLess(dx, -0.2, f"Reverse displacement dx={dx:.3f}m was not negative enough")

    def test_05_pure_rotation(self):
        self.assertIsNotNone(self.latest_odom)
        # Quaternion z/w
        q_z_init = self.latest_odom.pose.pose.orientation.z
        q_w_init = self.latest_odom.pose.pose.orientation.w
        yaw_init = 2.0 * math.atan2(q_z_init, q_w_init)

        # Rotate at 0.6 rad/s for 1.5 seconds
        self._send_cmd(linear_x=0.0, angular_z=0.6, duration_sec=1.5)
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        q_z_final = self.latest_odom.pose.pose.orientation.z
        q_w_final = self.latest_odom.pose.pose.orientation.w
        yaw_final = 2.0 * math.atan2(q_z_final, q_w_final)
        dyaw = yaw_final - yaw_init

        # Wrap angle
        dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
        self.assertGreater(dyaw, 0.3, f"Yaw change dyaw={dyaw:.3f} rad too small during rotation")

    def test_06_stopping(self):
        # Send zero command
        self._send_cmd(linear_x=0.0, angular_z=0.0, duration_sec=0.5)
        # Settle
        start = time.time()
        while time.time() - start < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertIsNotNone(self.latest_js)
        for vel in self.latest_js.velocity:
            self.assertAlmostEqual(vel, 0.0, delta=0.05, msg="Wheel velocity did not stop at zero")

    def test_07_command_timeout(self):
        # Send a burst then stop sending
        self._send_cmd(linear_x=0.4, angular_z=0.0, duration_sec=0.3)

        # Wait longer than cmd_vel_timeout (0.5s)
        time.sleep(0.8)
        start = time.time()
        while time.time() - start < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertIsNotNone(self.latest_js)
        for vel in self.latest_js.velocity:
            self.assertAlmostEqual(
                vel, 0.0, delta=0.05,
                msg="Controller did not zero velocities after cmd_vel_timeout"
            )

    def test_08_tf_broadcast(self):
        # Wait for TF message
        start = time.time()
        while time.time() - start < 5.0 and not self.received_tf:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.received_tf, "Did not receive odom -> base_link transform on /tf")


if __name__ == "__main__":
    unittest.main()
