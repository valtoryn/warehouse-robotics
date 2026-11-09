#!/usr/bin/env python3
"""Differential drive AMR kinematics and simulation verification tool.

EVIDENCE TAXONOMY:
  Mathematical kinematic model. This script verifies forward and inverse
  differential drive equations, limit enforcement, and geometric integration.
  It is NOT a physics-based simulator, Gazebo digital twin, or physical hardware test.

SI Units Convention:
  Linear distances: metres (m)
  Angles: radians (rad)
  Linear velocities: metres per second (m/s)
  Angular velocities: radians per second (rad/s)
  Time / steps: seconds (s)

Kinematics Contract (docs/architecture/HARDWARE_ABSTRACTION.md section 3):
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
    wheel_radius: float          # m
    wheel_separation: float      # m
    max_linear_velocity: float   # m/s
    max_angular_velocity: float  # rad/s
    max_wheel_velocity: float = 0.0  # rad/s

    def __post_init__(self):
        if self.wheel_radius <= 0.0:
            raise ValueError(f"wheel_radius must be positive, got {self.wheel_radius}")
        if self.wheel_separation <= 0.0:
            raise ValueError(f"wheel_separation must be positive, got {self.wheel_separation}")
        if self.max_linear_velocity <= 0.0:
            raise ValueError(f"max_linear_velocity must be positive, got {self.max_linear_velocity}")
        if self.max_angular_velocity <= 0.0:
            raise ValueError(f"max_angular_velocity must be positive, got {self.max_angular_velocity}")

        if self.max_wheel_velocity <= 0.0:
            # Maximum angular velocity of wheel at combined maximum linear and rotational speed
            self.max_wheel_velocity = (
                self.max_linear_velocity + (self.wheel_separation / 2.0) * self.max_angular_velocity
            ) / self.wheel_radius


@dataclass
class RobotPose:
    x: float = 0.0       # m
    y: float = 0.0       # m
    theta: float = 0.0   # rad


class DiffDriveSimulator:
    def __init__(self, params: DiffDriveParams):
        self.params = params
        self.d = params.wheel_separation / 2.0  # half track-width (m)
        self.r = params.wheel_radius            # wheel radius (m)
        self.pose = RobotPose()

    def inverse_kinematics(self, v: float, w: float) -> tuple[float, float]:
        """Convert linear velocity v (m/s) and angular velocity w (rad/s) to wheel speeds (rad/s).

        Scales velocities proportionally to preserve the instantaneous radius of
        curvature R = v / w when limits are exceeded.
        """
        if not math.isfinite(v) or not math.isfinite(w):
            return (0.0, 0.0)

        # Scale body velocities proportionally if either exceeds limits to preserve curvature R = v / w
        scale_v = self.params.max_linear_velocity / abs(v) if abs(v) > self.params.max_linear_velocity else 1.0
        scale_w = self.params.max_angular_velocity / abs(w) if abs(w) > self.params.max_angular_velocity else 1.0
        scale_body = min(scale_v, scale_w)

        v_scaled = v * scale_body
        w_scaled = w * scale_body

        ul = (v_scaled - self.d * w_scaled) / self.r
        ur = (v_scaled + self.d * w_scaled) / self.r

        # Scale wheel speeds proportionally if either exceeds actuator limits,
        # further preserving curvature
        max_req = max(abs(ul), abs(ur))
        if max_req > self.params.max_wheel_velocity and max_req > 1e-9:
            scale_wheel = self.params.max_wheel_velocity / max_req
            ul *= scale_wheel
            ur *= scale_wheel

        return (ul, ur)

    def forward_kinematics(self, ul: float, ur: float) -> tuple[float, float]:
        """Convert wheel angular speeds (rad/s) to body velocities v (m/s) and w (rad/s)."""
        if not math.isfinite(ul) or not math.isfinite(ur):
            return (0.0, 0.0)
        v = self.r * (ur + ul) / 2.0
        w = self.r * (ur - ul) / (2.0 * self.d)
        return (v, w)

    def step(
        self,
        v_cmd: float,
        w_cmd: float,
        dt: float,
        left_slip: float = 1.0,
        right_slip: float = 1.0,
    ) -> RobotPose:
        """Simulate one discrete step with exact arc integration.

        Parameters:
          v_cmd: Commanded linear velocity (m/s)
          w_cmd: Commanded angular velocity (rad/s)
          dt: Integration time step (s), must be > 0
          left_slip: Left traction coefficient [0.0 = total slip, 1.0 = perfect traction]
          right_slip: Right traction coefficient [0.0 = total slip, 1.0 = perfect traction]
        """
        if dt <= 0.0:
            raise ValueError(f"Integration time step dt must be positive, got {dt}")
        if not (0.0 <= left_slip <= 1.0 and 0.0 <= right_slip <= 1.0):
            raise ValueError(f"Slip factors must be in range [0.0, 1.0], got ({left_slip}, {right_slip})")

        ul, ur = self.inverse_kinematics(v_cmd, w_cmd)

        ul_actual = ul * left_slip
        ur_actual = ur * right_slip

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
