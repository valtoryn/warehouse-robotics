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
        urdf = gen_urdf.generate(config)
        gz_plugin = urdf.find(".//gazebo/plugin[@filename='gz_ros2_control-system']")
        self.assertIsNotNone(gz_plugin)

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

    def test_nav2_params_parse_and_costmaps(self):
        params = yaml.safe_load(gen_launch.nav2_params(self.config))
        self.assertIn("FollowPath", params["controller_server"]["ros__parameters"]["controller_plugins"])
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


if __name__ == "__main__":
    unittest.main()
