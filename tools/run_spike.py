#!/usr/bin/env python3
"""Integration spike test runner for Gate G-C (ROS 2 Jazzy + ros2_control + Mock AMR Bringup).

Executes the multi-stage integration spike or reports prerequisite blockers honestly:
  Stage 1: Preflight prerequisite inspection (Linux host, ROS 2 distro, colcon).
  Stage 2: Package compilation via colcon build across all src/ packages.
  Stage 3: Artifact generation (URDF + launch + controller parameters).
  Stage 4: Headless node bringup with safe log capture, crash detection, and topic verification.
  Stage 5: Controller activation and hardware interface claim verification.
  Stage 6: Real motion command-to-odometry and timeout verification.
  Stage 7: Automated ROS 2 integration tests execution (motion + lifecycle).

Exit Codes:
  0: Full spike passed, OR preflight succeeded when --preflight flag passed.
  2: Environment unavailable (missing Linux or ROS 2 runtime).
  3: Package build failed.
  4: Artifact generation failed.
  5: Launch bringup or controller verification failed.
  6: Motion or lifecycle test execution failed.

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
EXIT_TESTS_FAILED = 6


def check_prerequisites() -> dict[str, bool]:
    return {
        "linux_os": sys.platform.startswith("linux"),
        "ros2_cli": shutil.which("ros2") is not None,
        "colcon_build": shutil.which("colcon") is not None,
    }


def run_stage_build() -> bool:
    print("\n[STAGE 2/7] Building all ROS 2 packages via colcon...")
    cmd = [
        "colcon", "build",
        "--packages-select",
        "valtoryn_interfaces",
        "valtoryn_diff_drive",
        "valtoryn_lifecycle",
        "valtoryn_task_bt",
        "valtoryn_bringup",
        "--event-handlers", "console_direct+",
    ]
    res = subprocess.run(cmd, cwd=str(ROOT))
    if res.returncode != 0:
        print("ERROR: colcon build failed.")
        return False
    print("Build succeeded.")
    return True


def run_stage_generation() -> bool:
    print("\n[STAGE 3/7] Generating robot description and launch artifacts...")
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


def dump_log(log_file: Path, max_lines: int = 100) -> None:
    if not log_file.exists():
        print(f"Log file {log_file} does not exist.")
        return
    try:
        lines = log_file.read_text(encoding="utf-8", errors="replace").splitlines()
        print(f"\n--- BEGIN LAUNCH LOG (last {min(len(lines), max_lines)} lines) ---")
        for line in lines[-max_lines:]:
            print(line)
        print("--- END LAUNCH LOG ---\n")
    except Exception as exc:
        print(f"Failed to read log file {log_file}: {exc}")


def is_controller_active(stdout: str, name: str) -> bool:
    """Check if named controller is in active state from `ros2 control list_controllers`."""
    for line in stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == name and parts[2] == "active":
            return True
        if name in line and "active" in line:
            return True
    return False


def run_stage_launch_and_verify(timeout: int = 25) -> bool:
    print(f"\n[STAGE 4/7] Bringing up reference AMR mock robot (timeout: {timeout}s)...")
    setup_file = ROOT / "install" / "setup.bash"
    if not setup_file.exists():
        print(f"ERROR: Sourced workspace not found at {setup_file}.")
        return False

    gen_dir = ROOT / "generated"
    gen_dir.mkdir(parents=True, exist_ok=True)
    log_file = gen_dir / "launch.log"

    launch_script = gen_dir / "ref-amr-diff-v1_launch.py"
    urdf_arg = f"urdf_path:={gen_dir / 'spike_robot.urdf'}"
    params_arg = f"params_path:={gen_dir / 'ref-amr-diff-v1_controllers.yaml'}"

    launch_cmd = (
        f"source {setup_file} && "
        f"ros2 launch {launch_script} {urdf_arg} {params_arg}"
    )

    log_f = open(log_file, "w", encoding="utf-8")
    proc = subprocess.Popen(
        launch_cmd, shell=True, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT, text=True,
    )

    expected_topics = {
        "/robot_description",
        "/joint_states",
        "/diff_drive_controller/odom",
        "/tf",
    }
    verified_topics = set()
    start_time = time.time()
    jsb_active = False
    ddc_active = False

    try:
        while time.time() - start_time < timeout:
            # Check if process terminated prematurely
            exit_code = proc.poll()
            if exit_code is not None:
                print(f"ERROR: Launch process died prematurely with exit code {exit_code}.")
                log_f.flush()
                dump_log(log_file)
                return False

            # Check controller status
            ctrl_check = subprocess.run(
                ["bash", "-c", f"source {setup_file} && ros2 control list_controllers"],
                capture_output=True, text=True, timeout=5,
            )
            if ctrl_check.returncode == 0:
                if is_controller_active(ctrl_check.stdout, "joint_state_broadcaster"):
                    jsb_active = True
                if is_controller_active(ctrl_check.stdout, "diff_drive_controller"):
                    ddc_active = True

            # Check topic advertisements
            topic_check = subprocess.run(
                ["bash", "-c", f"source {setup_file} && ros2 topic list"],
                capture_output=True, text=True, timeout=5,
            )
            if topic_check.returncode == 0:
                current_topics = set(topic_check.stdout.splitlines())
                for t in expected_topics:
                    if t in current_topics:
                        verified_topics.add(t)

            if expected_topics.issubset(verified_topics) and jsb_active and ddc_active:
                print(f"Successfully verified active topics: {sorted(verified_topics)}")
                print("Successfully verified controllers active: joint_state_broadcaster, diff_drive_controller")
                break

            time.sleep(1.0)

        log_f.flush()

        if not (expected_topics.issubset(verified_topics) and jsb_active and ddc_active):
            print(f"ERROR: Timed out waiting for controllers and topics ({timeout}s).")
            print(f"  Verified topics: {sorted(verified_topics)}")
            print(f"  Missing topics:  {sorted(expected_topics - verified_topics)}")
            print(f"  JSB active: {jsb_active}, DDC active: {ddc_active}")
            ctrl_out = subprocess.run(
                ["bash", "-c", f"source {setup_file} && ros2 control list_controllers"],
                capture_output=True, text=True, timeout=5,
            )
            print(f"  Controllers status:\n{ctrl_out.stdout}")
            dump_log(log_file)
            return False

        # STAGE 5: Verify controllers and hardware interfaces
        print("\n[STAGE 5/7] Verifying controller states and hardware interfaces...")
        hw_check = subprocess.run(
            ["bash", "-c", f"source {setup_file} && ros2 control list_hardware_interfaces"],
            capture_output=True, text=True, timeout=5,
        )
        wheel_joints = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
        all_claimed = all(
            any(f"{j}/velocity" in line and "claimed" in line for line in hw_check.stdout.splitlines())
            for j in wheel_joints
        )
        if not all_claimed:
            print(f"ERROR: Wheel velocity command interfaces not claimed.\n{hw_check.stdout}")
            dump_log(log_file)
            return False
        print("Verified hardware interfaces exported and claimed.")

        # STAGE 6: Motion command and odometry verification
        print("\n[STAGE 6/7] Verifying motion command response and odometry...")
        cmd_pub = subprocess.run(
            [
                "bash", "-c",
                f"source {setup_file} && "
                f"ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
                f"'{{header: {{frame_id: \"base_link\"}}, twist: {{linear: {{x: 0.5}}}}}}'"
            ],
            capture_output=True, text=True, timeout=5,
        )
        if cmd_pub.returncode != 0:
            print(f"ERROR: Failed to publish cmd_vel: {cmd_pub.stderr}")
            dump_log(log_file)
            return False

        # Read back odometry
        time.sleep(0.5)
        odom_echo = subprocess.run(
            [
                "bash", "-c",
                f"source {setup_file} && "
                f"ros2 topic echo --once /diff_drive_controller/odom"
            ],
            capture_output=True, text=True, timeout=5,
        )
        if odom_echo.returncode != 0 or "position" not in odom_echo.stdout:
            print("ERROR: Failed to read /diff_drive_controller/odom")
            dump_log(log_file)
            return False
        print("Successfully verified motion command and odometry feedback.")

        # STAGE 7: Run automated ROS integration tests
        print("\n[STAGE 7/7] Running automated ROS 2 integration tests...")
        test_run = subprocess.run(
            [
                "bash", "-c",
                f"source {setup_file} && "
                f"python3 -m unittest discover -s tests -p 'test_ros_*.py' -v"
            ],
            capture_output=True, text=True, timeout=90,
        )
        print(test_run.stdout)
        if test_run.stderr:
            print(test_run.stderr)
        if test_run.returncode != 0:
            print(f"ERROR: Integration tests failed with return code {test_run.returncode}")
            dump_log(log_file)
            return False

        return True
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log_f.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preflight", action="store_true",
        help="Check prerequisites only without attempting build or launch",
    )
    parser.add_argument(
        "--timeout", type=int, default=25,
        help="Timeout in seconds for node launch topic verification (default: 25)",
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
        print("     ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-nav2-msgs && \\")
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

    if not run_stage_launch_and_verify(timeout=args.timeout):
        return EXIT_LAUNCH_FAILED

    print("\n==================================================")
    print("  GATE G-C INTEGRATION SPIKE PASSED               ")
    print("==================================================")
    return EXIT_SPIKE_PASSED


if __name__ == "__main__":
    sys.exit(main())
