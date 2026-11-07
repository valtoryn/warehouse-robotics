# Dependency compatibility matrix

> **Status:** Living artifact (R01 §8 recommendation). Records which component versions are known to
> be compatible, proposed, or unresolved. The ROS distro pin is **not frozen** — see
> [DECISION_GATES.md](DECISION_GATES.md) G-A and [TECHNOLOGY_DECISIONS.md](../architecture/TECHNOLOGY_DECISIONS.md) TD-2.
> Evidence tags per CLAUDE.md. Checked 2026-11.

## Matrix

| Component | Proposed version | Status | Notes / evidence |
|---|---|---|---|
| ROS 2 distro | Lyrical Luth (primary), Jazzy (fallback) | **Proposed — pin undecided** | Lyrical = Ubuntu 26.04; Jazzy = Ubuntu 24.04 which matches JetPack 7.2 on Jetson Orin (G-A evidence) |
| Gazebo | Jetty (LTS) | Proposed | Pairs with Lyrical via packages.ros.org; verify pairing for Jazzy fallback |
| `ros_gz` / `gz_ros2_control` | Jetty-matched | Proposed | Bridge versions must match Gazebo major; check per distro |
| `ros2_control` / `ros2_controllers` | 6.x (Lyrical) / per-distro | **API caveat** | Kilted→Lyrical breaking changes documented (DECISION_GATES G-D): export APIs removed, `HardwareInfo`→`HardwareComponentParams` |
| Nav2 | 1.5.x (Lyrical) | Proposed | Per-distro plugin branches; verify Smac/MPPI/CollisionMonitor/Docking availability on chosen distro (G-C spike) |
| `slam_toolbox` | 2.10.x | Proposed | LGPL-2.1 — BOM/license tracking required |
| `robot_localization` | 3.11.x | Proposed | Apache-2.0 |
| BehaviorTree.CPP | 4.x | Proposed | MIT; Nav2 BT nodes dependency |
| RMW | `rmw_fastrtps_cpp` default; `rmw_zenoh_cpp` spike | Proposed | Never mix RMWs in one system; Zenoh spike is a G-gated evaluation |
| VDA 5050 library | spec 3.0.x; library TBD | **Gap confirmed** | `vda5050_interfaces` implements spec **2.0.0** only; master API experimental (G-E). Thin-build contingency |
| Open-RMF | optional backend | Deferred | Never a core dependency (TD-10) |
| `rplidar_ros` | 2.1.0-5 (Lyrical) | Verified released | Upstream stale (2.1.0 vs 2.1.4 elsewhere; rosdistro #49653) — acceptable for reference robot |
| SICK/Hokuyo safety scanners | none maintained | **Open item** | `sicktoolbox` unmaintained since 2019; protective function is in safety domain; ROS telemetry needs custom/vendor driver |
| Motor drive interfaces | custom `ros2_control` components | Required build | CiA 402 / CAN-FD; no off-the-shelf dependency |

## Rules

1. **Pin distro + Gazebo + `ros_gz` as a unit** — never mix majors (TECHNOLOGY_DECISIONS §4).
2. Every matrix row gets a status: `verified` / `proposed` / `unresolved` / `rejected`. This table
   must be updated whenever a gate passes or a dependency is added.
3. New dependencies require an ADR (CLAUDE.md) and a row here.
4. License column is tracked informally here; `slam_toolbox` (LGPL-2.1) is the only copyleft entry —
   confirm dynamic-linking compliance before release.

## Open questions feeding this matrix

- A-1: ROS distribution pin (blocked on owner decision, informed by G-A).
- G-C spike results will confirm or break the Nav2/Gazebo rows for the pinned distro.
- Camera shortlist not fixed → camera driver rows pending.
