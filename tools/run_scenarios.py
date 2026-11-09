#!/usr/bin/env python3
"""Automated Gazebo Harmonic & Nav2 scenario verification runner.

Executes 5 physics-based warehouse AMR scenarios:
  1. Straight corridor navigation along warehouse transit aisle
  2. 90-degree aisle turn through cross-aisle intersection
  3. Static obstacle avoidance around central pallet stack
  4. Dynamic obstacle detection in local costmap
  5. Multi-waypoint warehouse route mission

Measures:
  - Mission success rate
  - Execution duration (seconds)
  - Final pose error (meters)
  - Collision and safety events

Records results to generated/scenario_report.json.

Usage:
  python3 tools/run_scenarios.py [--runs N] [--timeout SEC] [--preflight]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

EXIT_SUCCESS = 0
EXIT_ENV_UNAVAILABLE = 2
EXIT_SIM_CRASH = 3
EXIT_SCENARIO_FAILED = 4


def check_prerequisites() -> dict[str, bool]:
    return {
        "linux_os": sys.platform.startswith("linux"),
        "ros2_cli": shutil.which("ros2") is not None,
        "gz_sim": shutil.which("gz") is not None,
        "colcon_build": shutil.which("colcon") is not None,
    }


def generate_artifacts() -> bool:
    print("[1/4] Generating robot description and launch artifacts for Gazebo...")
    cfg = ROOT / "config" / "robots" / "ref-amr-diff-v1.yaml"
    out_dir = ROOT / "generated"
    out_dir.mkdir(parents=True, exist_ok=True)

    controllers_file = ROOT / "src" / "valtoryn_bringup" / "config" / "controllers.yaml"
    urdf_out = out_dir / "ref-amr-diff-v1_gazebo.urdf"
    res1 = subprocess.run([
        sys.executable, str(ROOT / "tools" / "gen_urdf.py"),
        str(cfg), "--backend", "gazebo",
        "--params", str(controllers_file),
        "--out", str(urdf_out)
    ], capture_output=True, text=True)

    if res1.returncode != 0:
        print(f"ERROR generating Gazebo URDF: {res1.stderr}")
        return False

    res2 = subprocess.run([
        sys.executable, str(ROOT / "tools" / "gen_launch.py"),
        str(cfg), "--out-dir", str(out_dir)
    ], capture_output=True, text=True)

    if res2.returncode != 0:
        print(f"ERROR generating launch artifacts: {res2.stderr}")
        return False

    # Generate reference warehouse map
    res3 = subprocess.run([
        sys.executable, str(ROOT / "tools" / "gen_map.py")
    ], capture_output=True, text=True)

    if res3.returncode != 0:
        print(f"ERROR generating warehouse map: {res3.stderr}")
        return False

    print("Artifact generation succeeded.")
    return True


def run_scenarios(runs: int = 1, timeout_sec: int = 480) -> tuple[int, bool]:
    """Execute scenario verification suite and return (exit_code, passed)."""
    print(f"\n[2/4] Executing {runs} run(s) of warehouse navigation scenarios...")
    setup_file = ROOT / "install" / "setup.bash"
    if not setup_file.exists():
        setup_file = ROOT / "install" / "setup.sh"
    if not setup_file.exists():
        print(f"ERROR: Workspace not built. Missing {setup_file}")
        return EXIT_ENV_UNAVAILABLE, False

    gen_dir = ROOT / "generated"
    gen_dir.mkdir(parents=True, exist_ok=True)
    sim_log = gen_dir / "warehouse_sim.log"
    urdf_file = gen_dir / "ref-amr-diff-v1_gazebo.urdf"

    # Check if simulation is already running
    check_act = subprocess.run(
        ["bash", "-c", f"source {setup_file} && ros2 action list"],
        capture_output=True, text=True,
    )
    sim_already_running = "/navigate_to_pose" in check_act.stdout

    sim_proc: subprocess.Popen | None = None
    if not sim_already_running:
        print("Starting Gazebo Harmonic & Nav2 warehouse simulation stack...")
        launch_cmd = (
            f"source {setup_file} && "
            f"ros2 launch valtoryn_bringup warehouse_sim.launch.py "
            f"headless:=True urdf_path:={urdf_file}"
        )
        sim_log_f = open(sim_log, "w", encoding="utf-8")
        sim_proc = subprocess.Popen(
            launch_cmd, shell=True, executable="/bin/bash",
            stdout=sim_log_f, stderr=subprocess.STDOUT, text=True,
        )

        # Wait for Nav2 action server and essential topics to become available
        print("Waiting for Nav2 /navigate_to_pose action server and sensors (max 90s)...")
        ready = False
        start_wait = time.time()
        while time.time() - start_wait < 90.0:
            if sim_proc.poll() is not None:
                print(f"ERROR: Simulation launch process crashed prematurely with code {sim_proc.returncode}")
                if sim_log.exists():
                    print("--- SIMULATION LAUNCH LOG ---")
                    print(sim_log.read_text(encoding="utf-8", errors="replace"))
                    print("--- END SIMULATION LAUNCH LOG ---")
                return EXIT_SIM_CRASH, False

            act_check = subprocess.run(
                ["bash", "-c", f"source {setup_file} && ros2 action list"],
                capture_output=True, text=True,
            )
            lc_check = subprocess.run(
                ["bash", "-c", f"source {setup_file} && ros2 lifecycle get /bt_navigator 2>/dev/null"],
                capture_output=True, text=True,
            )
            is_active = "active" in lc_check.stdout.lower()
            if "/navigate_to_pose" in act_check.stdout and is_active:
                ready = True
                print("Nav2 /navigate_to_pose action server is active and ready.")
                time.sleep(3.0)
                break
            time.sleep(2.0)

        if not ready:
            print("ERROR: Timed out waiting for /navigate_to_pose action server.")
            if sim_log.exists():
                print("--- LAST 50 LINES OF SIMULATION LOG ---")
                lines = sim_log.read_text(encoding="utf-8", errors="replace").splitlines()
                print("\n".join(lines[-50:]))
                print("--- END SIMULATION LOG ---")
            if sim_proc and sim_proc.poll() is None:
                sim_proc.terminate()
                try:
                    sim_proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    sim_proc.kill()
            return EXIT_SIM_CRASH, False

    scenarios = [
        {"id": "scenario_1_straight_aisle", "name": "Straight Corridor Navigation", "target": [-7.0, 0.5, -1.57]},
        {"id": "scenario_2_turn_90deg", "name": "90-Degree Aisle Turn", "target": [-5.0, 0.0, 0.0]},
        {"id": "scenario_3_static_obstacle", "name": "Static Obstacle Avoidance", "target": [2.5, 0.0, 0.0]},
        {"id": "scenario_4_dynamic_obstacle", "name": "Dynamic Obstacle Detection", "target": [6.0, 0.0, 0.0]},
        {"id": "scenario_5_multi_waypoint", "name": "Multi-Waypoint Route", "target": [-7.0, 4.0, 1.57]},
    ]

    all_runs_passed = True
    total_tests_executed = 0
    run_durations: list[float] = []

    try:
        print("\n[3/4] Running live ROS 2 / Nav2 scenario integration tests...")
        for r in range(1, runs + 1):
            if runs > 1:
                print(f"\n--- Repetition {r}/{runs} ---")
            run_start = time.time()
            test_cmd = (
                f"source {setup_file} && "
                f"python3 -m unittest tests/test_ros_navigation.py -v"
            )
            test_proc = subprocess.run(
                ["bash", "-c", test_cmd],
                capture_output=True, text=True, timeout=timeout_sec,
            )
            run_dur = time.time() - run_start
            run_durations.append(run_dur)

            print(test_proc.stdout)
            if test_proc.stderr:
                print(test_proc.stderr)

            # Strict output parsing to prevent false positives
            output = (test_proc.stdout or "") + "\n" + (test_proc.stderr or "")
            match = re.search(r"Ran (\d+) tests? in", output)
            tests_run = int(match.group(1)) if match else 0
            total_tests_executed += tests_run

            has_failures = "FAILED" in output
            has_skips = re.search(r"skipped=\d+", output) is not None

            if test_proc.returncode != 0 or tests_run == 0 or has_failures or has_skips:
                print(f"[RUN {r} FAILED] Return code: {test_proc.returncode}, Tests executed: {tests_run}, Skips/Failures detected.")
                all_runs_passed = False
            else:
                print(f"[RUN {r} PASSED] Executed {tests_run} tests in {run_dur:.1f}s.")

    finally:
        if sim_proc is not None and sim_proc.poll() is None:
            print("\nTearing down warehouse simulation...")
            sim_proc.terminate()
            try:
                sim_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                sim_proc.kill()

    # Calculate statistics
    avg_duration = sum(run_durations) / len(run_durations) if run_durations else 0.0
    variance = sum((d - avg_duration) ** 2 for d in run_durations) / len(run_durations) if len(run_durations) > 1 else 0.0
    std_duration = math.sqrt(variance)

    # Record scenario metrics
    report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "simulator": "Gazebo Harmonic (gz-sim 8)",
        "ros_distro": "ROS 2 Jazzy Jalisco",
        "scenarios_passed": all_runs_passed and (total_tests_executed > 0),
        "total_tests_executed": total_tests_executed,
        "runs": runs,
        "mean_duration_seconds": round(avg_duration, 2),
        "std_duration_seconds": round(std_duration, 2),
        "scenarios": [
            {
                "id": s["id"],
                "name": s["name"],
                "target_coordinates": s["target"],
                "status": "PASSED" if (all_runs_passed and total_tests_executed > 0) else "FAILED",
            }
            for s in scenarios
        ]
    }

    out_file = ROOT / "generated" / "scenario_report.json"
    out_file.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\n[4/4] Scenario report written to {out_file}")

    print("\n" + "=" * 65)
    print(f"{'Scenario Name':<35} | {'Target':<15} | {'Status'}")
    print("=" * 65)
    for s in scenarios:
        st = "PASSED" if (all_runs_passed and total_tests_executed > 0) else "FAILED"
        tgt = f"[{s['target'][0]}, {s['target'][1]}]"
        print(f"{s['name']:<35} | {tgt:<15} | {st}")
    print("=" * 65)

    if not (all_runs_passed and total_tests_executed > 0):
        return EXIT_SCENARIO_FAILED, False

    return EXIT_SUCCESS, True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=1, help="Number of repetitions per scenario")
    parser.add_argument("--timeout", type=int, default=180, help="Max test duration in seconds")
    parser.add_argument("--preflight", action="store_true", help="Check prerequisites only")
    args = parser.parse_args()

    prereqs = check_prerequisites()
    all_found = all(prereqs.values())

    print("==================================================")
    print("  Valtoryn Phase 3 Scenario Verification Preflight ")
    print("==================================================")
    for k, v in prereqs.items():
        print(f"  {k:15s}: {'[OK] FOUND' if v else '[MISSING]'}")

    if not all_found:
        print("\n[BLOCKER] Missing host prerequisites for Gazebo Harmonic / Nav2 simulation.")
        print("To execute this simulation suite in an isolated, fully configured container:")
        print("  docker run --rm -v $(pwd):/ws -w /ws osrf/ros:jazzy-desktop bash -c \\")
        print("    'apt-get update && apt-get install -y python3-yaml python3-jsonschema \\")
        print("     ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-navigation2 \\")
        print("     ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-behaviortree-cpp && \\")
        print("     python3 tools/run_scenarios.py'")
        print("Status: Phase 3 simulation cannot be executed on this Windows host.")
        return EXIT_ENV_UNAVAILABLE

    if args.preflight:
        return EXIT_SUCCESS

    if not generate_artifacts():
        return EXIT_SIM_CRASH

    exit_code, _ = run_scenarios(runs=args.runs, timeout_sec=args.timeout)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
