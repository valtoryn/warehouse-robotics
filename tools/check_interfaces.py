#!/usr/bin/env python3
"""Static validation of Valtoryn ROS 2 interfaces, package manifests, and BT XML.

Performs static ROS-independent validation:
1. Validates .msg and .srv syntax, types, and field identifiers.
2. Asserts that .srv files have exactly one '---' separator.
3. Checks CMakeLists.txt interface registrations match files on disk.
4. Validates package.xml conformance and dependencies.
5. Validates BehaviorTree.CPP XML well-formedness and schema attributes.

Usage:
  python tools/check_interfaces.py
"""
from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INTERFACES_DIR = ROOT / "src" / "valtoryn_interfaces"
TASK_BT_DIR = ROOT / "src" / "valtoryn_task_bt"
DIFF_DRIVE_DIR = ROOT / "src" / "valtoryn_diff_drive"

PRIMITIVE_TYPES = {
    "bool", "byte", "char",
    "float32", "float64",
    "int8", "uint8",
    "int16", "uint16",
    "int32", "uint32",
    "int64", "uint64",
    "string", "wstring",
}

KNOWN_EXTERNAL_TYPES = {
    "builtin_interfaces/Time",
    "builtin_interfaces/Duration",
    "std_msgs/Header",
    "geometry_msgs/Point",
    "geometry_msgs/Point32",
    "geometry_msgs/Pose",
    "geometry_msgs/Pose2D",
    "geometry_msgs/PoseStamped",
    "geometry_msgs/Twist",
    "geometry_msgs/TwistStamped",
    "geometry_msgs/Vector3",
}

FIELD_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def validate_field(line: str, line_no: int, local_types: set[str], path: Path) -> str | None:
    # Strip comments
    code_part = line.split("#")[0].strip()
    if not code_part:
        return None  # comment or empty line

    # Check constant definition: TYPE NAME = VALUE
    if "=" in code_part:
        parts = code_part.split("=")
        lhs = parts[0].strip().split()
        if len(lhs) != 2:
            return f"{path.name}:{line_no}: invalid constant declaration '{line.strip()}'"
        type_str, name_str = lhs
    else:
        parts = code_part.split()
        if len(parts) < 2:
            return f"{path.name}:{line_no}: invalid field declaration '{line.strip()}'"
        type_str, name_str = parts[0], parts[1]

    # Clean array suffix, e.g. Point32[] or Point32[<=10]
    base_type = re.sub(r"\[.*?\]$", "", type_str)

    # Check type validity
    is_valid_type = (
        base_type in PRIMITIVE_TYPES
        or base_type in KNOWN_EXTERNAL_TYPES
        or base_type in local_types
        or (base_type.startswith("valtoryn_interfaces/") and base_type.split("/")[-1] in local_types)
    )
    if not is_valid_type:
        return f"{path.name}:{line_no}: unknown type '{type_str}' in field declaration"

    if not FIELD_NAME_RE.match(name_str):
        return f"{path.name}:{line_no}: invalid field identifier '{name_str}'"

    return None


def check_msg_file(path: Path, local_types: set[str]) -> list[str]:
    errors: list[str] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    for idx, line in enumerate(lines, 1):
        err = validate_field(line, idx, local_types, path)
        if err:
            errors.append(err)
    return errors


def check_srv_file(path: Path, local_types: set[str]) -> list[str]:
    errors: list[str] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    separator_count = 0
    separator_lines: list[int] = []

    for idx, line in enumerate(lines, 1):
        stripped = line.strip()
        if stripped == "---":
            separator_count += 1
            separator_lines.append(idx)
            continue
        err = validate_field(line, idx, local_types, path)
        if err:
            errors.append(err)

    if separator_count != 1:
        errors.append(
            f"{path.name}: service must have exactly one '---' separator "
            f"(found {separator_count} at lines {separator_lines})"
        )
    return errors


def check_cmakelists(cmake_path: Path, msg_files: list[str], srv_files: list[str]) -> list[str]:
    errors: list[str] = []
    content = cmake_path.read_text(encoding="utf-8")

    for m in msg_files:
        rel = f"msg/{m}"
        if rel not in content:
            errors.append(f"{cmake_path.name}: missing interface registration for '{rel}'")

    for s in srv_files:
        rel = f"srv/{s}"
        if rel not in content:
            errors.append(f"{cmake_path.name}: missing interface registration for '{rel}'")

    return errors


def check_package_xml(xml_path: Path, required_deps: list[str]) -> list[str]:
    errors: list[str] = []
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.tag != "package":
            errors.append(f"{xml_path.name}: root tag is not <package>")
            return errors
        if root.attrib.get("format") != "3":
            errors.append(f"{xml_path.name}: package format must be '3'")

        deps = {el.text.strip() for el in root.findall(".//depend") if el.text}
        deps |= {el.text.strip() for el in root.findall(".//buildtool_depend") if el.text}
        deps |= {el.text.strip() for el in root.findall(".//exec_depend") if el.text}

        for req in required_deps:
            if req not in deps:
                errors.append(f"{xml_path.name}: missing required dependency <depend>{req}</depend>")

    except ET.ParseError as exc:
        errors.append(f"{xml_path.name}: XML parse error: {exc}")
    return errors


def check_bt_xml(xml_path: Path) -> list[str]:
    errors: list[str] = []
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.tag != "root":
            errors.append(f"{xml_path.name}: root tag is not <root>")
        if root.attrib.get("BTCPP_format") != "4":
            errors.append(f"{xml_path.name}: BTCPP_format must be '4'")
        trees = root.findall("BehaviorTree")
        if not trees:
            errors.append(f"{xml_path.name}: no <BehaviorTree> defined")
        for t in trees:
            if not t.attrib.get("ID"):
                errors.append(f"{xml_path.name}: <BehaviorTree> missing ID attribute")
    except ET.ParseError as exc:
        errors.append(f"{xml_path.name}: XML parse error: {exc}")
    return errors


def main() -> int:
    errors: list[str] = []

    if not INTERFACES_DIR.exists():
        print(f"ERROR: {INTERFACES_DIR} does not exist")
        return 1

    msg_dir = INTERFACES_DIR / "msg"
    srv_dir = INTERFACES_DIR / "srv"

    msg_files = sorted([p.name for p in msg_dir.glob("*.msg")])
    srv_files = sorted([p.name for p in srv_dir.glob("*.srv")])
    local_types = {Path(f).stem for f in msg_files}

    for mf in msg_files:
        errors.extend(check_msg_file(msg_dir / mf, local_types))

    for sf in srv_files:
        errors.extend(check_srv_file(srv_dir / sf, local_types))

    cmake_file = INTERFACES_DIR / "CMakeLists.txt"
    if cmake_file.exists():
        errors.extend(check_cmakelists(cmake_file, msg_files, srv_files))

    pkg_file = INTERFACES_DIR / "package.xml"
    if pkg_file.exists():
        errors.extend(check_package_xml(pkg_file, ["builtin_interfaces", "geometry_msgs", "std_msgs", "rosidl_default_generators"]))

    # Task BT package check
    task_pkg = TASK_BT_DIR / "package.xml"
    if task_pkg.exists():
        errors.extend(check_package_xml(task_pkg, ["valtoryn_interfaces", "nav2_msgs"]))

    bt_xml = TASK_BT_DIR / "bt" / "ref_amr_mission.xml"
    if bt_xml.exists():
        errors.extend(check_bt_xml(bt_xml))

    # Diff drive hardware package check
    diff_pkg = DIFF_DRIVE_DIR / "package.xml"
    if diff_pkg.exists():
        errors.extend(check_package_xml(diff_pkg, ["hardware_interface", "pluginlib", "rclcpp"]))

    plugin_xml = DIFF_DRIVE_DIR / "valtoryn_diff_drive.xml"
    if plugin_xml.exists():
        try:
            tree = ET.parse(plugin_xml)
            root = tree.getroot()
            if root.tag != "library":
                errors.append(f"{plugin_xml.name}: root tag is not <library>")
            cls = root.find("class")
            if cls is None:
                errors.append(f"{plugin_xml.name}: missing <class> definition")
            elif cls.attrib.get("name") != "valtoryn_diff_drive/DiffDriveHardware":
                errors.append(f"{plugin_xml.name}: class name does not match expected plugin name")
        except ET.ParseError as exc:
            errors.append(f"{plugin_xml.name}: XML parse error: {exc}")

    if errors:
        print("INTERFACE VALIDATION FAILED:")
        for e in errors:
            print(f"  ERROR {e}")
        return 1

    print(
        f"interface check OK: {len(msg_files)} msg(s), {len(srv_files)} srv(s), "
        f"package manifests and BT XML valid"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
