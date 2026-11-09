#!/usr/bin/env python3
"""Differential drive AMR kinematics and simulation verification tool.

Implements and validates the Chassis kinematics contract defined in
docs/architecture/HARDWARE_ABSTRACTION.md section 3 and formulas from
docs/engineering/REFERENCE_ROBOT_SPECIFICATION.md section 2:

Kinematics:
  v  = r * (uR + uL) / 2
  w  = r * (uR - uL) / (2d)
  uL = (v - d * w) / r
  uR = (v + d * w) / r

where:
  r  = wheel_radius (m)
  2d = wheel_separation (m)
  uL, uR = left and right wheel angular velocities (rad/s)
  v  = linear velocity in robot frame (m/s)
  w  = angular velocity about yaw axis (rad/s)

Usage:
  python tools/simulate_kinematics.py config/robots/ref-amr-diff-v1.yaml
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent


@dataclass
class DiffDriveParams:
    wheel_radius: float
    wheel_separation: float
    max_linear_velocity: float
    max_angular_velocity: float
    max_wheel_velocity: float = 0.0

    def __post_init__(self):
        if self.max_wheel_velocity <= 0.0:
            # Maximum angular velocity of wheel at top linear speed
            self.max_wheel_velocity = (
                self.max_linear_velocity + (self.wheel_separation / 2.0) * self.max_angular_velocity
            ) / self.wheel_radius


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians


class DiffDriveSimulator:
    def __init__(self, params: DiffDriveParams):
        self.params = params
        self.d = params.wheel_separation / 2.0
        self.r = params.wheel_radius
        self.pose = RobotPose()

    def inverse_kinematics(self, v: float, w: float) -> tuple[float, float]:
        """Convert linear velocity v and angular velocity w to left and right wheel speeds (rad/s)."""
        # Clamp commanded velocities to parameters
        v_clamped = max(-self.params.max_linear_velocity, min(self.params.max_linear_velocity, v))
        w_clamped = max(-self.params.max_angular_velocity, min(self.params.max_angular_velocity, w))

        ul = (v_clamped - self.d * w_clamped) / self.r
        ur = (v_clamped + self.d * w_clamped) / self.r

        # Actuator limit scaling: preserve curvature if wheel speeds exceed maximum
        max_req = max(abs(ul), abs(ur))
        if max_req > self.params.max_wheel_velocity:
            scale = self.params.max_wheel_velocity / max_req
            ul *= scale
            ur *= scale

        return (ul, ur)

    def forward_kinematics(self, ul: float, ur: float) -> tuple[float, float]:
        """Convert wheel speeds (rad/s) to body velocities (v, w)."""
        v = self.r * (ur + ul) / 2.0
        w = self.r * (ur - ul) / (2.0 * self.d)
        return (v, w)

    def step(self, v_cmd: float, w_cmd: float, dt: float, slip_factor: float = 1.0) -> RobotPose:
        """Simulate one discrete step with arc integration."""
        ul, ur = self.inverse_kinematics(v_cmd, w_cmd)

        # Apply wheel slip if simulating imperfect traction
        ul_actual = ul * slip_factor
        ur_actual = ur * slip_factor

        v, w = self.forward_kinematics(ul_actual, ur_actual)

        # Exact differential drive arc integration
        if abs(w) < 1e-6:
            self.pose.x += v * math.cos(self.pose.theta) * dt
            self.pose.y += v * math.sin(self.pose.theta) * dt
        else:
            theta_next = self.pose.theta + w * dt
            self.pose.x += (v / w) * (math.sin(theta_next) - math.sin(self.pose.theta))
            self.pose.y -= (v / w) * (math.cos(theta_next) - math.cos(self.pose.theta))
            self.pose.theta = math.atan2(math.sin(theta_next), math.cos(theta_next))

        return self.pose


def load_params_from_config(config_path: Path) -> DiffDriveParams:
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    chassis = next(m for m in cfg["modules"] if m["type"].endswith("chassis.differential"))
    p = chassis.get("parameters", {})
    return DiffDriveParams(
        wheel_radius=float(p.get("wheel_radius", 0.075)),
        wheel_separation=float(p.get("wheel_separation", 0.6)),
        max_linear_velocity=float(p.get("max_linear_velocity", 1.2)),
        max_angular_velocity=float(p.get("max_angular_velocity", 1.5)),
    )


def run_mission_profile_sim(params: DiffDriveParams) -> tuple[RobotPose, float]:
    """Runs a standard square waypoint test and checks loop-closure accuracy."""
    sim = DiffDriveSimulator(params)
    dt = 0.02  # 50 Hz controller update rate
    total_time = 0.0

    # Segments: 4 sides of 1.0m each, followed by 90-degree pivot turn
    side_length = 1.0
    v_cruise = min(0.5, params.max_linear_velocity)
    w_turn = min(0.5, params.max_angular_velocity)

    travel_time = side_length / v_cruise
    turn_time = (math.pi / 2.0) / w_turn

    for _ in range(4):
        # Straight line
        steps_travel = int(travel_time / dt)
        for _ in range(steps_travel):
            sim.step(v_cruise, 0.0, dt)
            total_time += dt

        # Turn 90 degrees
        steps_turn = int(turn_time / dt)
        for _ in range(steps_turn):
            sim.step(0.0, w_turn, dt)
            total_time += dt

    # Closure error distance
    closure_error = math.sqrt(sim.pose.x ** 2 + sim.pose.y ** 2)
    return (sim.pose, closure_error)


def main() -> int:
    cfg_file = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "config" / "robots" / "ref-amr-diff-v1.yaml"
    params = load_params_from_config(cfg_file)

    print(f"Testing kinematics for: {cfg_file.name}")
    print(f"  wheel_radius: {params.wheel_radius} m")
    print(f"  wheel_separation: {params.wheel_separation} m")
    print(f"  max_linear_vel: {params.max_linear_velocity} m/s")
    print(f"  max_angular_vel: {params.max_angular_velocity} rad/s")
    print(f"  max_wheel_vel: {params.max_wheel_velocity:.3f} rad/s")

    # Run closed-loop mission
    final_pose, closure_err = run_mission_profile_sim(params)
    print(f"\nCompleted 4-corner trajectory simulation:")
    print(f"  Final pose: x={final_pose.x:.4f} m, y={final_pose.y:.4f} m, theta={final_pose.theta:.4f} rad")
    print(f"  Closed-loop kinematic error: {closure_err:.6f} m")

    if closure_err > 0.01:
        print("ERROR: Kinematic integration closure error exceeded 10mm threshold!")
        return 1

    print("Kinematic simulation verification OK.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
