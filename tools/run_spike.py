#!/usr/bin/env python3
"""Integration spike test runner for Gate G-C (ROS 2 Jazzy + Gazebo + ros2_control + Nav2).

Executes the multi-stage integration spike or reports prerequisite blockers honestly:
  Stage 1: Preflight prerequisite inspection (Linux host, ROS 2 distro, colcon, Gazebo/gz).
  Stage 2: Package compilation via colcon build.
  Stage 3: Artifact generation (URDF + launch + controller parameters).
  Stage 4: Headless node bringup and topic advertisement verification.

Exit Codes:
  0: Full spike passed, OR preflight succeeded when --preflight flag passed.
  2: Environment unavailable (missing Linux, ROS 2, or Gazebo runtime).
  3: Package build failed.
  4: Artifact generation failed.
  5: Launch bringup or topic verification failed.

Usage:
  python tools/run_spike.py --preflight    # Report prerequisites only (non-fatal inspection)
  python tools/run_spike.py                # Execute full spike (requires Linux + ROS 2)
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

EXIT_SPIKE_PASSED = 0
EXIT_PREFLIGHT_OK = 0
EXIT_ENV_UNAVAILABLE = 2
EXIT_BUILD_FAILED = 3
EXIT_GEN_FAILED = 4
EXIT_LAUNCH_FAILED = 5


def check_prerequisites() -> dict[str, bool]:
    return {
        "linux_os": sys.platform.startswith("linux"),
        "ros2_cli": shutil.which("ros2") is not None,
        "colcon_build": shutil.which("colcon") is not None,
        "gz_sim": (shutil.which("gz") is not None or shutil.which("ign") is not None),
    }


def run_stage_build() -> bool:
    print("\n[STAGE 2/4] Building ROS 2 packages via colcon...")
    cmd = [
        "colcon", "build",
        "--packages-select", "valtoryn_interfaces", "valtoryn_diff_drive",
        "--event-handlers", "console_direct+",
    ]
    res = subprocess.run(cmd, cwd=str(ROOT))
    if res.returncode != 0:
        print("ERROR: colcon build failed.")
        return False
    print("Build succeeded.")
    return True


def run_stage_generation() -> bool:
    print("\n[STAGE 3/4] Generating robot description and launch artifacts...")
    gen_dir = ROOT / "generated"
    gen_dir.mkdir(parents=True, exist_ok=True)
    cfg_file = ROOT / "config" / "robots" / "ref-amr-diff-v1.yaml"

    cmd_urdf = [
        sys.executable, str(ROOT / "tools" / "gen_urdf.py"),
        str(cfg_file), "--backend", "mock",
        "--out", str(gen_dir / "spike_robot.urdf"),
    ]
    if subprocess.run(cmd_urdf).returncode != 0:
        print("ERROR: URDF generation failed.")
        return False

    cmd_launch = [
        sys.executable, str(ROOT / "tools" / "gen_launch.py"),
        str(cfg_file), "--out-dir", str(gen_dir),
    ]
    if subprocess.run(cmd_launch).returncode != 0:
        print("ERROR: Launch generation failed.")
        return False

    print("Generation succeeded.")
    return True


def run_stage_launch(timeout: int = 10) -> bool:
    print(f"\n[STAGE 4/4] Verifying node launch and topic advertisement (timeout: {timeout}s)...")
    setup_file = ROOT / "install" / "setup.bash"
    if not setup_file.exists():
        print(f"ERROR: Sourced workspace not found at {setup_file}.")
        return False

    # Launch in a subshell sourcing the workspace
    launch_script = ROOT / "generated" / "ref-amr-diff-v1_launch.py"
    urdf_arg = f"urdf_path:={ROOT / 'generated' / 'spike_robot.urdf'}"
    params_arg = f"params_path:={ROOT / 'generated' / 'ref-amr-diff-v1_controllers.yaml'}"

    launch_cmd = (
        f"source {setup_file} && "
        f"ros2 launch {launch_script} {urdf_arg} {params_arg}"
    )

    proc = subprocess.Popen(
        launch_cmd, shell=True, executable="/bin/bash",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )

    expected_topics = {"/robot_description", "/joint_states"}
    verified_topics = set()
    start_time = time.time()

    try:
        while time.time() - start_time < timeout:
            topic_check = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True, text=True, timeout=3,
            )
            if topic_check.returncode == 0:
                current_topics = set(topic_check.stdout.splitlines())
                for t in expected_topics:
                    if t in current_topics:
                        verified_topics.add(t)
                if expected_topics.issubset(verified_topics):
                    print(f"Successfully verified topics: {sorted(verified_topics)}")
                    return True
            time.sleep(1.0)

        print(f"ERROR: Timed out waiting for topics. Verified: {sorted(verified_topics)}, Missing: {sorted(expected_topics - verified_topics)}")
        return False
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preflight", action="store_true",
        help="Check prerequisites only without attempting build or launch",
    )
    parser.add_argument(
        "--timeout", type=int, default=10,
        help="Timeout in seconds for node launch topic verification (default: 10)",
    )
    args = parser.parse_args()

    prereqs = check_prerequisites()
    all_found = all(prereqs.values())

    print("==================================================")
    print("  Valtoryn Gate G-C Integration Spike Preflight  ")
    print("==================================================")
    for k, v in prereqs.items():
        print(f"  {k:15s}: {'[OK] FOUND' if v else '[MISSING]'}")

    if not all_found:
        print("\n[BLOCKER] Missing host prerequisites for Gate G-C integration spike.")
        print("To execute this spike in an isolated, fully configured container:")
        print("  docker run --rm -v $(pwd):/ws -w /ws osrf/ros:jazzy-desktop bash -c \\")
        print("    'apt-get update && apt-get install -y python3-yaml python3-jsonschema \\")
        print("     ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control && \\")
        print("     python3 tools/run_spike.py'")
        print("Status: Gate G-C cannot be executed on this machine.")
        return EXIT_ENV_UNAVAILABLE

    if args.preflight:
        print("\nPreflight check passed. Host environment supports full integration spike.")
        return EXIT_PREFLIGHT_OK

    # Execute full spike stages
    if not run_stage_build():
        return EXIT_BUILD_FAILED

    if not run_stage_generation():
        return EXIT_GEN_FAILED

    if not run_stage_launch(timeout=args.timeout):
        return EXIT_LAUNCH_FAILED

    print("\n==================================================")
    print("  GATE G-C INTEGRATION SPIKE PASSED               ")
    print("==================================================")
    return EXIT_SPIKE_PASSED


if __name__ == "__main__":
    sys.exit(main())
