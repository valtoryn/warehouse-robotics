#!/usr/bin/env python3
"""Unit tests for tools/check_config.py and tools/gen_urdf.py.

Run: python -m unittest discover -s tests -v
Requires PyYAML and jsonschema (same as tools/check_config.py).
"""
from __future__ import annotations

import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import check_config  # noqa: E402
import check_interfaces  # noqa: E402
import gen_launch  # noqa: E402
import gen_urdf  # noqa: E402
import simulate_kinematics  # noqa: E402
import yaml  # noqa: E402


def write_yaml(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


BASE_CONFIG = {
    "schema_version": "1.0.0",
    "robot": {"id": "test-robot", "variant": "t1"},
    "modules": [
        {
            "type": "com.valtoryn.chassis.differential",
            "version": "^1.0.0",
            "instance": "base",
            "parameters": {"wheel_radius": 0.1, "wheel_separation": 0.5},
        }
    ],
    "target": {"profile": "simulation", "ros_distro": "lyrical"},
}


def minimal_manifest(module_type: str, version: str = "1.0.0") -> dict:
    return {
        "manifest_schema_version": "1.0.0",
        "module_type": module_type,
        "module_version": version,
        "provides": [],
        "requires": [],
        "frames": {"declared": ["base_link"]},
        "platform": {"ros_distro": "lyrical"},
        "verification": {"capabilities": {"declared": [], "verified": []}},
        "safety": {"evidence_ids": [], "claim": "none"},
    }


class RangeTests(unittest.TestCase):
    def test_caret_range(self):
        self.assertTrue(check_config.range_satisfied("^1.0.0", "1.4.2"))
        self.assertFalse(check_config.range_satisfied("^1.0.0", "2.0.0"))
        self.assertFalse(check_config.range_satisfied("^1.0.0", "0.9.9"))

    def test_caret_zero_minor_range(self):
        # SemVer: ^0.1.0 means >=0.1.0 <0.2.0
        self.assertTrue(check_config.range_satisfied("^0.1.0", "0.1.4"))
        self.assertFalse(check_config.range_satisfied("^0.1.0", "0.2.0"))
        self.assertFalse(check_config.range_satisfied("^0.1.0", "1.0.0"))

    def test_caret_zero_patch_range(self):
        # SemVer: ^0.0.2 means >=0.0.2 <0.0.3
        self.assertTrue(check_config.range_satisfied("^0.0.2", "0.0.2"))
        self.assertFalse(check_config.range_satisfied("^0.0.2", "0.0.3"))

    def test_compound_range(self):
        self.assertTrue(check_config.range_satisfied(">=1.0.0 <2.0.0", "1.5.0"))
        self.assertFalse(check_config.range_satisfied(">=1.0.0 <2.0.0", "2.0.0"))
        self.assertFalse(check_config.range_satisfied(">=1.0.0 <2.0.0", "0.9.0"))

    def test_exact_version(self):
        self.assertTrue(check_config.range_satisfied("1.2.3", "1.2.3"))
        self.assertFalse(check_config.range_satisfied("1.2.3", "1.2.4"))

    def test_prerelease_suffix(self):
        self.assertTrue(check_config.range_satisfied("^1.0.0", "1.0.0-rc1"))


class CheckConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        # Redirect module lookup into the temp dir by monkeypatching MODULES_DIR
        self._orig_modules = check_config.MODULES_DIR
        check_config.MODULES_DIR = self.root / "modules"

    def tearDown(self):
        check_config.MODULES_DIR = self._orig_modules
        self.tmp.cleanup()

    def _write_manifest(self, module_type, version="1.0.0"):
        mpath = check_config.MODULES_DIR / module_type / "manifest.yaml"
        write_yaml(mpath, minimal_manifest(module_type, version))

    def _run(self, config) -> list[str]:
        cfg = write_yaml(self.root / "cfg.yaml", config)
        schema = yaml.safe_load((ROOT / "schemas/robot-config.schema.json").read_text())
        mschema = yaml.safe_load((ROOT / "schemas/module-manifest.schema.json").read_text())
        return check_config.check_config(cfg, schema, mschema)

    def test_valid_config_passes(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        self.assertEqual(self._run(BASE_CONFIG), [])

    def test_missing_manifest_fails(self):
        errors = self._run(BASE_CONFIG)
        self.assertTrue(any("no manifest" in e for e in errors))

    def test_version_mismatch_fails(self):
        self._write_manifest("com.valtoryn.chassis.differential", version="2.0.0")
        errors = self._run(BASE_CONFIG)
        self.assertTrue(any("does not match" in e for e in errors))

    def test_duplicate_instance_fails(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        mod = dict(BASE_CONFIG["modules"][0])
        # Two modules with the same instance name 'base'
        cfg = dict(BASE_CONFIG)
        cfg["modules"] = [dict(mod), dict(mod)]
        errors = self._run(cfg)
        self.assertTrue(any("duplicate module instance name" in e for e in errors))

    def test_undeclared_parent_frame_fails(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        self._write_manifest("com.valtoryn.sensor.lidar")
        cfg = dict(BASE_CONFIG)
        m2 = {
            "type": "com.valtoryn.sensor.lidar",
            "version": "^1.0.0",
            "instance": "lidar_1",
            "parent_frame": "nonexistent_custom_link",
        }
        cfg["modules"] = [dict(BASE_CONFIG["modules"][0]), m2]
        errors = self._run(cfg)
        self.assertTrue(any("parent_frame" in e and "not declared" in e for e in errors))

    def test_duplicate_mount_fails(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        mod = dict(BASE_CONFIG["modules"][0])
        mod["instance"] = "second_base"
        mod["mount"] = {"mount_type": "chassis_base", "occupied_mounts": ["deck_primary"]}
        base = dict(BASE_CONFIG)
        m0 = dict(BASE_CONFIG["modules"][0])
        m0["mount"] = {"mount_type": "chassis_base", "occupied_mounts": ["deck_primary"]}
        base["modules"] = [m0, mod]
        errors = self._run(base)
        self.assertTrue(any("claimed by both" in e for e in errors))

    def test_unvalidated_module_rejected_on_hardware_target(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        cfg = dict(BASE_CONFIG)
        cfg["target"] = {"profile": "hardware", "ros_distro": "jazzy"}
        cfg["modules"] = [dict(BASE_CONFIG["modules"][0], unvalidated=True)]
        errors = self._run(cfg)
        self.assertTrue(any("unvalidated" in e for e in errors))

    def test_schema_violation_fails(self):
        cfg = dict(BASE_CONFIG)
        cfg["schema_version"] = "not-a-version"
        errors = self._run(cfg)
        self.assertTrue(errors)

    def test_incompatible_simulation_profile_and_physical_backend(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        cfg = dict(BASE_CONFIG)
        cfg["target"] = {"profile": "simulation", "backend": "physical", "ros_distro": "jazzy"}
        errors = self._run(cfg)
        self.assertTrue(any("cannot use backend 'physical'" in e for e in errors))

    def test_incompatible_hardware_profile_and_gazebo_backend(self):
        self._write_manifest("com.valtoryn.chassis.differential")
        cfg = dict(BASE_CONFIG)
        cfg["target"] = {"profile": "hardware", "backend": "gazebo", "ros_distro": "jazzy"}
        errors = self._run(cfg)
        self.assertTrue(any("cannot use simulation backend 'gazebo'" in e for e in errors))


class GenUrdfTests(unittest.TestCase):
    def test_generates_parseable_urdf(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config)
        ET.fromstring(ET.tostring(urdf))  # raises if malformed
        names = {el.get("name") for el in urdf.findall("link")}
        self.assertIn("base_link", names)
        for w in ("fl", "fr", "rl", "rr"):
            self.assertIn(f"wheel_{w}_link", names)

    def test_ros2_control_tag_present(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config)
        rc = urdf.find("ros2_control")
        self.assertIsNotNone(rc)
        self.assertEqual(rc.get("type"), "system")
        hw = rc.find("hardware")
        self.assertIsNotNone(hw)
        plugin = hw.find("plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.text, "valtoryn_diff_drive/DiffDriveHardware")

    def test_ros2_control_joints_are_children_of_ros2_control(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config)
        rc = urdf.find("ros2_control")
        self.assertIsNotNone(rc)
        joints = rc.findall("joint")
        self.assertEqual(len(joints), 4)
        for j in joints:
            self.assertIsNotNone(j.find("command_interface[@name='velocity']"))
            self.assertIsNotNone(j.find("state_interface[@name='position']"))
            self.assertIsNotNone(j.find("state_interface[@name='velocity']"))
        # Ensure hardware tag does NOT contain joint tags
        hw = rc.find("hardware")
        self.assertEqual(len(hw.findall("joint")), 0)

    def test_backend_selection_mock(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="mock")
        plugin = urdf.find(".//ros2_control/hardware/plugin")
        self.assertEqual(plugin.text, "valtoryn_diff_drive/DiffDriveHardware")
        mock_param = urdf.find(".//ros2_control/hardware/param[@name='mock_hardware']")
        self.assertIsNotNone(mock_param)
        self.assertEqual(mock_param.text, "true")
        self.assertIsNone(urdf.find("gazebo"))

    def test_backend_selection_gazebo(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="gazebo")
        plugin = urdf.find(".//ros2_control/hardware/plugin")
        self.assertEqual(plugin.text, "gz_ros2_control/GazeboSimSystem")
        gz_plugin = urdf.find(".//gazebo/plugin[@filename='gz_ros2_control-system']")
        self.assertIsNotNone(gz_plugin)

    def test_backend_selection_physical(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="physical")
        plugin = urdf.find(".//ros2_control/hardware/plugin")
        self.assertEqual(plugin.text, "valtoryn_diff_drive/DiffDriveHardware")
        mock_param = urdf.find(".//ros2_control/hardware/param[@name='mock_hardware']")
        self.assertIsNotNone(mock_param)
        self.assertEqual(mock_param.text, "false")
        can_param = urdf.find(".//ros2_control/hardware/param[@name='can_interface']")
        self.assertIsNotNone(can_param)
        self.assertEqual(can_param.text, "can0")
        self.assertIsNone(urdf.find("gazebo"))

    def test_invalid_backend_rejected(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        with self.assertRaises(ValueError):
            gen_urdf.generate(config, backend_override="nonexistent_backend")

    def test_wheel_geometry_and_orientation(self):
        config = dict(BASE_CONFIG)
        urdf = gen_urdf.generate(config)
        visual = urdf.find(".//link[@name='wheel_fl_link']/visual")
        self.assertIsNotNone(visual)
        origin = visual.find("origin")
        self.assertIsNotNone(origin)
        # Cylinder must be rolled pi/2 so its cylinder axis aligns with rotation axis Y
        self.assertEqual(origin.get("rpy"), "1.570796 0 0")
        cyl = visual.find("geometry/cylinder")
        self.assertIsNotNone(cyl)
        self.assertAlmostEqual(float(cyl.get("radius")), 0.1)

    def test_gazebo_plugin_present_for_simulation(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="gazebo")
        gz_plugin = urdf.find(".//gazebo/plugin[@filename='gz_ros2_control-system']")
        self.assertIsNotNone(gz_plugin)

    def test_lidar_sensor_generated(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="mock")
        laser_link = urdf.find(".//link[@name='laser_link']")
        self.assertIsNotNone(laser_link)
        laser_joint = urdf.find(".//joint[@name='laser_link_joint']")
        self.assertIsNotNone(laser_joint)
        self.assertEqual(laser_joint.get("type"), "fixed")
        self.assertEqual(laser_joint.find("parent").get("link"), "base_link")
        self.assertEqual(laser_joint.find("child").get("link"), "laser_link")

    def test_gazebo_sensor_and_friction_generated(self):
        config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )
        urdf = gen_urdf.generate(config, backend_override="gazebo")
        gz_sensor = urdf.find(".//gazebo[@reference='laser_link']/sensor[@name='gpu_lidar']")
        self.assertIsNotNone(gz_sensor)
        self.assertEqual(gz_sensor.find("topic").text, "scan")
        self.assertEqual(gz_sensor.find("gz_frame_id").text, "laser_link")

        # Check wheel friction elements
        gz_wheel = urdf.find(".//gazebo[@reference='wheel_fl_link']")
        self.assertIsNotNone(gz_wheel)
        self.assertEqual(gz_wheel.find("mu1").text, "0.8")
        self.assertEqual(gz_wheel.find("mu2").text, "0.05")
        self.assertEqual(gz_wheel.find("fdir1").text, "1 0 0")
        self.assertEqual(gz_wheel.find("kp").text, "1000000.0")
        self.assertEqual(gz_wheel.find("kd").text, "10.0")

    def test_missing_chassis_fails(self):
        config = {"robot": {"id": "x", "variant": "v"}, "modules": [], "target": {}}
        with self.assertRaises(SystemExit):
            gen_urdf.generate(config)


class GenLaunchTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load(
            (ROOT / "config/robots/ref-amr-diff-v1.yaml").read_text(encoding="utf-8")
        )

    def test_controller_params_parse_and_use_config(self):
        text = gen_launch.controller_params(self.config)
        params = yaml.safe_load(text)
        ddc = params["diff_drive_controller"]["ros__parameters"]
        self.assertAlmostEqual(ddc["wheel_separation"], 0.6)
        self.assertEqual(ddc["left_wheel_names"], ["wheel_fl_joint", "wheel_rl_joint"])
        self.assertAlmostEqual(ddc["wheel_separation_multiplier"], 1.20)

    def test_nav2_params_parse_and_costmaps(self):
        params = yaml.safe_load(gen_launch.nav2_params(self.config))
        self.assertIn("FollowPath", params["controller_server"]["ros__parameters"]["controller_plugins"])
        follow = params["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertAlmostEqual(follow["rotate_to_heading_min_angle"], 0.08)
        self.assertTrue(follow["use_cost_regulated_linear_velocity_scaling"])
        self.assertAlmostEqual(
            params["controller_server"]["ros__parameters"]["min_y_velocity_threshold"],
            0.0
        )
        self.assertIn("local_costmap", params)
        self.assertIn("global_costmap", params)

    def test_launch_py_is_valid_python(self):
        import ast
        code = gen_launch.launch_py(self.config, "ref-amr-diff-v1")
        ast.parse(code)
        self.assertIn("robot_description", code)


class InterfaceValidationTests(unittest.TestCase):
    def test_existing_interfaces_valid(self):
        self.assertEqual(check_interfaces.main(), 0)

    def test_extra_separator_in_srv_detected(self):
        tmp = tempfile.NamedTemporaryFile("w+", suffix=".srv", delete=False)
        try:
            tmp.write("---\nstring field\n---\nstring result\n")
            tmp.flush()
            tmp.close()
            errs = check_interfaces.check_srv_file(Path(tmp.name), set())
            self.assertTrue(any("exactly one '---' separator" in e for e in errs))
        finally:
            Path(tmp.name).unlink(missing_ok=True)


class KinematicsSimulationTests(unittest.TestCase):
    def setUp(self):
        self.params = simulate_kinematics.DiffDriveParams(
            wheel_radius=0.1,
            wheel_separation=0.5,
            max_linear_velocity=1.5,
            max_angular_velocity=2.0,
        )
        self.sim = simulate_kinematics.DiffDriveSimulator(self.params)

    def test_forward_inverse_symmetry(self):
        v_in, w_in = 0.8, 0.4
        ul, ur = self.sim.inverse_kinematics(v_in, w_in)
        v_out, w_out = self.sim.forward_kinematics(ul, ur)
        self.assertAlmostEqual(v_in, v_out)
        self.assertAlmostEqual(w_in, w_out)

    def test_velocity_clamping(self):
        ul, ur = self.sim.inverse_kinematics(5.0, 10.0)
        v_out, w_out = self.sim.forward_kinematics(ul, ur)
        self.assertLessEqual(v_out, self.params.max_linear_velocity + 1e-6)
        self.assertLessEqual(w_out, self.params.max_angular_velocity + 1e-6)

    def test_trajectory_closure(self):
        pose, err = simulate_kinematics.run_mission_profile_sim(self.params)
        self.assertLess(err, 0.005)  # Error less than 5mm on closed 4m course

    def test_zero_command(self):
        ul, ur = self.sim.inverse_kinematics(0.0, 0.0)
        self.assertEqual(ul, 0.0)
        self.assertEqual(ur, 0.0)

    def test_pure_rotation(self):
        w = 1.0
        ul, ur = self.sim.inverse_kinematics(0.0, w)
        # In pure rotation, wheels turn in opposite directions
        self.assertAlmostEqual(ul, -ur)
        expected_speed = (self.params.wheel_separation / 2.0) * w / self.params.wheel_radius
        self.assertAlmostEqual(ur, expected_speed)

    def test_pure_translation(self):
        v = 1.0
        ul, ur = self.sim.inverse_kinematics(v, 0.0)
        # In pure translation, both wheels turn at identical speed
        self.assertAlmostEqual(ul, ur)
        expected_speed = v / self.params.wheel_radius
        self.assertAlmostEqual(ul, expected_speed)

    def test_reverse_motion(self):
        v = -0.5
        ul, ur = self.sim.inverse_kinematics(v, 0.0)
        self.assertAlmostEqual(ul, ur)
        self.assertAlmostEqual(ul, v / self.params.wheel_radius)

    def test_curvature_preservation_under_limit(self):
        # Commanded velocities exceeding wheel limits
        v_cmd = 2.0
        w_cmd = 2.0
        r_curvature_expected = v_cmd / w_cmd  # R = 1.0m

        ul, ur = self.sim.inverse_kinematics(v_cmd, w_cmd)
        v_out, w_out = self.sim.forward_kinematics(ul, ur)
        self.assertGreater(w_out, 0.0)
        r_curvature_actual = v_out / w_out
        self.assertAlmostEqual(r_curvature_expected, r_curvature_actual, places=4)

    def test_invalid_parameters_raise(self):
        with self.assertRaises(ValueError):
            simulate_kinematics.DiffDriveParams(
                wheel_radius=0.0,
                wheel_separation=0.5,
                max_linear_velocity=1.0,
                max_angular_velocity=1.0,
            )
        with self.assertRaises(ValueError):
            simulate_kinematics.DiffDriveParams(
                wheel_radius=0.1,
                wheel_separation=-0.5,
                max_linear_velocity=1.0,
                max_angular_velocity=1.0,
            )

    def test_invalid_step_inputs_raise(self):
        with self.assertRaises(ValueError):
            self.sim.step(1.0, 0.0, dt=0.0)
        with self.assertRaises(ValueError):
            self.sim.step(1.0, 0.0, dt=0.02, left_slip=-0.1)

    def test_asymmetric_slip_causes_drift(self):
        sim1 = simulate_kinematics.DiffDriveSimulator(self.params)
        sim2 = simulate_kinematics.DiffDriveSimulator(self.params)

        # Drive forward for 1 second
        dt = 0.02
        for _ in range(50):
            sim1.step(1.0, 0.0, dt, left_slip=1.0, right_slip=1.0)
            # Left wheel slips by 20%
            sim2.step(1.0, 0.0, dt, left_slip=0.8, right_slip=1.0)

        # sim1 drives perfectly straight (y=0, theta=0)
        self.assertAlmostEqual(sim1.pose.y, 0.0)
        self.assertAlmostEqual(sim1.pose.theta, 0.0)

        # sim2 turns towards the left (positive yaw in standard coordinates: ur > ul -> w > 0)
        self.assertGreater(sim2.pose.theta, 0.0)


if __name__ == "__main__":
    unittest.main()
