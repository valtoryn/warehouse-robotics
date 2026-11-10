#!/usr/bin/env python3
"""Generate a URDF robot description skeleton from a Valtoryn robot config.

Phase 2 scaffolding: emits a plain URDF (no xacro) containing the chassis base
link, wheel links/joints derived from module parameters, and a <ros2_control>
hardware declaration. The hardware plugin name is a placeholder for the future
Valtoryn ros2_control component — generated files are build artifacts and must
not be edited by hand.

This generator is intentionally minimal: it proves the config -> description
pipeline works end to end. Sensor, payload and compute modules are represented
as fixed frames only; full generation lands with the Phase 2 config generator.

Usage:
  python tools/gen_urdf.py config/robots/ref-amr-diff-v1.yaml [--out PATH]
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent

WHEEL_POSITIONS = {
    # name: (x_offset_sign, y_offset_sign) — positions computed from wheel_separation/2
    "fl": (+1, +1),
    "fr": (+1, -1),
    "rl": (-1, +1),
    "rr": (-1, -1),
}


def fmt(v: float) -> str:
    return f"{v:.6f}".rstrip("0").rstrip(".") or "0"


def inertial_mass_xml(parent: ET.Element, mass: float, ix: float, iy: float, iz: float):
    inertial = ET.SubElement(parent, "inertial")
    ET.SubElement(inertial, "mass", value=fmt(mass))
    ET.SubElement(
        inertial, "inertia",
        ixx=fmt(ix), ixy="0", ixz="0",
        iyy=fmt(iy), iyz="0", izz=fmt(iz),
    )


def cylinder_geom(parent: ET.Element, radius: float, length: float):
    # In URDF, cylinders align along the Z axis by default. Rotating by roll=pi/2
    # aligns the cylinder's symmetry axis with the Y axis (the wheel rotational axis).
    geom = ET.SubElement(parent, "visual")
    ET.SubElement(geom, "origin", xyz="0 0 0", rpy="1.570796 0 0")
    c = ET.SubElement(geom, "geometry")
    ET.SubElement(c, "cylinder", radius=fmt(radius), length=fmt(length))
    ET.SubElement(geom, "material", name="valtoryn_dark")
    # collision identical
    col = ET.SubElement(parent, "collision")
    ET.SubElement(col, "origin", xyz="0 0 0", rpy="1.570796 0 0")
    cc = ET.SubElement(col, "geometry")
    ET.SubElement(cc, "cylinder", radius=fmt(radius), length=fmt(length))


def generate(
    config: dict,
    backend_override: str | None = None,
    controllers_file: str | None = None,
) -> ET.Element:
    robot = ET.Element("robot", name=config["robot"]["id"])

    # Materials
    mat = ET.SubElement(robot, "material", name="valtoryn_dark")
    ET.SubElement(mat, "color", rgba="0.15 0.15 0.18 1")

    chassis_mods = [m for m in config["modules"] if m["type"].endswith("chassis.differential")]
    if not chassis_mods:
        raise SystemExit("config has no com.valtoryn.chassis.differential module")
    ch = chassis_mods[0]
    p = ch.get("parameters", {})
    wheel_r = float(p.get("wheel_radius", 0.075))
    sep = float(p.get("wheel_separation", 0.6))
    wheel_w = float(p.get("wheel_width", 0.05))
    wheelbase = float(p.get("wheelbase", sep))
    half_sep = sep / 2.0
    half_base = wheelbase / 2.0
    # [EST] chassis footprint until mechanical design exists
    chassis_l, chassis_w, chassis_h = 0.8, sep + 0.2, 0.25

    # base_link (origin of robot; chassis module sits at its own pose)
    base = ET.SubElement(robot, "link", name="base_link")
    cp = base.find("visual") if base.find("visual") is not None else ET.SubElement(base, "visual")
    ET.SubElement(cp, "origin", xyz=f"0 0 {fmt(wheel_r + chassis_h / 2)}", rpy="0 0 0")
    cg = ET.SubElement(cp, "geometry")
    ET.SubElement(cg, "box", size=f"{fmt(chassis_l)} {fmt(chassis_w)} {fmt(chassis_h)}")
    ET.SubElement(cp, "material", name="valtoryn_dark")
    cc = ET.SubElement(base, "collision")
    ET.SubElement(cc, "origin", xyz=f"0 0 {fmt(wheel_r + chassis_h / 2)}", rpy="0 0 0")
    cg2 = ET.SubElement(cc, "geometry")
    ET.SubElement(cg2, "box", size=f"{fmt(chassis_l)} {fmt(chassis_w)} {fmt(chassis_h)}")
    inertial_mass_xml(base, mass=50.0, ix=3.0, iy=3.0, iz=4.5)  # [EST] box inertia for 50kg AMR

    # wheel links + continuous joints
    for suffix, (sx, sy) in WHEEL_POSITIONS.items():
        link = ET.SubElement(robot, "link", name=f"wheel_{suffix}_link")
        cylinder_geom(link, wheel_r, wheel_w)
        inertial_mass_xml(link, mass=2.5, ix=0.005, iy=0.009, iz=0.005)  # [EST] wheel inertia
        joint = ET.SubElement(robot, "joint", name=f"wheel_{suffix}_joint", type="continuous")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=f"wheel_{suffix}_link")
        ET.SubElement(
            joint, "origin",
            xyz=f"{fmt(sx * half_base)} {fmt(sy * half_sep)} {fmt(wheel_r)}",
            rpy="0 0 0",
        )
        ET.SubElement(joint, "axis", xyz="0 1 0")

    # Backend selection (mock vs gazebo vs physical)
    target = config.get("target", {})
    profile = target.get("profile", "simulation")
    backend = backend_override or target.get("backend")
    if not backend:
        backend = "physical" if profile == "hardware" else "mock"

    if backend not in ("mock", "gazebo", "physical"):
        raise ValueError(f"Unknown hardware backend '{backend}' (must be mock, gazebo, or physical)")

    # ros2_control hardware declaration
    rc = ET.SubElement(robot, "ros2_control", name="valtoryn_hw", type="system")
    hw = ET.SubElement(rc, "hardware")
    plugin_el = ET.SubElement(hw, "plugin")

    if backend == "gazebo":
        plugin_el.text = "gz_ros2_control/GazeboSimSystem"
    else:
        plugin_el.text = "valtoryn_diff_drive/DiffDriveHardware"
        param_mock = ET.SubElement(hw, "param", name="mock_hardware")
        param_mock.text = "true" if backend == "mock" else "false"
        if backend == "physical":
            param_can = ET.SubElement(hw, "param", name="can_interface")
            param_can.text = "can0"

    for suffix in WHEEL_POSITIONS:
        j = ET.SubElement(rc, "joint", name=f"wheel_{suffix}_joint")
        ET.SubElement(j, "command_interface", name="velocity")
        ET.SubElement(j, "state_interface", name="position")
        ET.SubElement(j, "state_interface", name="velocity")

    # Sensor modules (e.g. 2D navigation LiDAR)
    sensor_mods = [
        m for m in config.get("modules", [])
        if "sensor" in m.get("type", "") or "lidar" in m.get("type", "")
    ]
    for sm in sensor_mods:
        sp = sm.get("parameters", {})
        frame_id = sp.get("frame_id", "laser_link")
        parent_frame = sm.get("parent_frame", "base_link")
        pose_xyz = sm.get("pose", {}).get("xyz", [0.38, 0.0, 0.42])
        pose_rpy = sm.get("pose", {}).get("rpy", [0.0, 0.0, 0.0])
        topic = sp.get("topic", "/scan").lstrip("/")
        update_rate = sp.get("update_rate", 10.0)
        min_range = sp.get("min_range", 0.12)
        max_range = sp.get("max_range", 12.0)
        samples = sp.get("samples", 360)

        # Sensor link
        s_link = ET.SubElement(robot, "link", name=frame_id)
        s_vis = ET.SubElement(s_link, "visual")
        ET.SubElement(s_vis, "origin", xyz="0 0 0", rpy="0 0 0")
        s_geom = ET.SubElement(s_vis, "geometry")
        ET.SubElement(s_geom, "cylinder", radius="0.04", length="0.04")
        ET.SubElement(s_vis, "material", name="valtoryn_dark")
        s_col = ET.SubElement(s_link, "collision")
        ET.SubElement(s_col, "origin", xyz="0 0 0", rpy="0 0 0")
        s_col_geom = ET.SubElement(s_col, "geometry")
        ET.SubElement(s_col_geom, "cylinder", radius="0.04", length="0.04")
        inertial_mass_xml(s_link, mass=0.2, ix=0.0001, iy=0.0001, iz=0.0001)

        # Fixed joint from parent_frame to sensor link
        s_joint = ET.SubElement(robot, "joint", name=f"{frame_id}_joint", type="fixed")
        ET.SubElement(s_joint, "parent", link=parent_frame)
        ET.SubElement(s_joint, "child", link=frame_id)
        ET.SubElement(
            s_joint, "origin",
            xyz=f"{fmt(pose_xyz[0])} {fmt(pose_xyz[1])} {fmt(pose_xyz[2])}",
            rpy=f"{fmt(pose_rpy[0])} {fmt(pose_rpy[1])} {fmt(pose_rpy[2])}",
        )

        # Gazebo GPU LiDAR sensor configuration
        if backend == "gazebo":
            gz_ref = ET.SubElement(robot, "gazebo", reference=frame_id)
            gz_sensor = ET.SubElement(gz_ref, "sensor", name="gpu_lidar", type="gpu_lidar")
            ET.SubElement(gz_sensor, "pose").text = "0 0 0 0 0 0"
            ET.SubElement(gz_sensor, "update_rate").text = str(fmt(update_rate))
            ET.SubElement(gz_sensor, "visualize").text = "false"
            ET.SubElement(gz_sensor, "topic").text = topic
            ET.SubElement(gz_sensor, "gz_frame_id").text = frame_id
            lidar_el = ET.SubElement(gz_sensor, "lidar")
            scan_el = ET.SubElement(lidar_el, "scan")
            h_scan = ET.SubElement(scan_el, "horizontal")
            ET.SubElement(h_scan, "samples").text = str(samples)
            ET.SubElement(h_scan, "resolution").text = "1"
            ET.SubElement(h_scan, "min_angle").text = "-3.14159265"
            ET.SubElement(h_scan, "max_angle").text = "3.14159265"
            v_scan = ET.SubElement(scan_el, "vertical")
            ET.SubElement(v_scan, "samples").text = "1"
            ET.SubElement(v_scan, "resolution").text = "1"
            ET.SubElement(v_scan, "min_angle").text = "0"
            ET.SubElement(v_scan, "max_angle").text = "0"
            range_el = ET.SubElement(lidar_el, "range")
            ET.SubElement(range_el, "min").text = str(fmt(min_range))
            ET.SubElement(range_el, "max").text = str(fmt(max_range))
            ET.SubElement(range_el, "resolution").text = "0.01"

    # Wheel contact physics in Gazebo (surface friction)
    # Low lateral friction (mu2 = 0.05) allows 4-wheel skid-steer AMR to turn smoothly in place
    if backend == "gazebo":
        for suffix in WHEEL_POSITIONS:
            gz_wheel = ET.SubElement(robot, "gazebo", reference=f"wheel_{suffix}_link")
            ET.SubElement(gz_wheel, "mu1").text = "0.8"
            ET.SubElement(gz_wheel, "mu2").text = "0.05"
            ET.SubElement(gz_wheel, "kp").text = "1000000.0"
            ET.SubElement(gz_wheel, "kd").text = "10.0"

    # Gazebo system plugin is only attached for the gazebo simulation backend
    if backend == "gazebo":
        gz_plugin = ET.SubElement(robot, "gazebo")
        p_el = ET.SubElement(
            gz_plugin,
            "plugin",
            filename="gz_ros2_control-system",
            name="gz_ros2_control::GazeboSimROS2ControlPlugin",
        )
        param_el = ET.SubElement(p_el, "parameters")
        param_el.text = controllers_file or str(ROOT / "src" / "valtoryn_bringup" / "config" / "controllers.yaml")

    return robot


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cfg_path = Path(sys.argv[1])
    out = Path(sys.argv[sys.argv.index("--out") + 1]) if "--out" in sys.argv else None
    backend = sys.argv[sys.argv.index("--backend") + 1] if "--backend" in sys.argv else None
    params = sys.argv[sys.argv.index("--params") + 1] if "--params" in sys.argv else None
    config = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    urdf = generate(config, backend_override=backend, controllers_file=params)
    ET.indent(urdf, space="  ")
    text = ET.tostring(urdf, encoding="unicode")
    text = '<?xml version="1.0"?>\n<!-- Generated by tools/gen_urdf.py from ' + cfg_path.name + \
        '. Do not edit; regenerate from the robot config. -->\n' + text + "\n"
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
        print(f"wrote {out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
