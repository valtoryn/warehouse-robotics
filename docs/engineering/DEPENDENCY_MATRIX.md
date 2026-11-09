# Dependency compatibility matrix

> **Status:** Living artifact (R01 §8 recommendation). Records which component versions are known to
> be compatible, proposed, or unresolved. The ROS distro pin is **not frozen** — see
> [DECISION_GATES.md](DECISION_GATES.md) G-A and [TECHNOLOGY_DECISIONS.md](../architecture/TECHNOLOGY_DECISIONS.md) TD-2.
> Evidence tags per CLAUDE.md. Checked 2026-11.

## Matrix

| Component | Proposed version | Status | Notes / evidence |
|---|---|---|---|
| ROS 2 distro | Jazzy Jalisco (LTS to May 2029) | **Active baseline** | Ubuntu 24.04 LTS; matches JetPack 7.2 on Jetson Orin Nano natively (DECISION_GATES G-A) |
| Gazebo | Harmonic (gz-sim 8, LTS to May 2029) | **Verified released** | Official Gazebo LTS pairing for ROS 2 Jazzy on Ubuntu 24.04; debian `ros-jazzy-ros-gz` |
| `ros_gz` / `gz_ros2_control` | Harmonic-matched (`ros-jazzy-gz-ros2-control`) | **Verified released** | Official hardware plugin and transport bridge for Gazebo Harmonic on Jazzy |
| `ros2_control` / `ros2_controllers` | 4.x (Jazzy) | **Verified released** | `DiffDriveHardware` uses `on_init(const HardwareComponentInterfaceParams &)` (Jazzy API) |
| Nav2 | Jazzy release (`ros-jazzy-navigation2`) | **Verified released** | SmacPlanner2D, Navfn, Regulated Pure Pursuit, AMCL, Collision Monitor, Waypoint Follower |
| `slam_toolbox` | 2.10.x (Jazzy release) | **Verified released** | Online async 2D mapping via Ceres solver; LGPL-2.1 |
| `robot_localization` | 3.11.x | Proposed | Apache-2.0 |
| BehaviorTree.CPP | 4.x (`ros-jazzy-behaviortree-cpp`) | **Verified released** | C++17, Groot2 compatible, non-blocking actions; `valtoryn_task_bt` |
| RMW | `rmw_fastrtps_cpp` default; `rmw_zenoh_cpp` spike | Proposed | Never mix RMWs in one system; Zenoh spike is a G-gated evaluation |
| VDA 5050 library | spec 3.0.x; library TBD | **Gap confirmed** | `vda5050_interfaces` implements spec **2.0.0** only; master API experimental (G-E). Thin-build contingency |
| Open-RMF | optional backend | Deferred | Never a core dependency (TD-10) |
| `rplidar_ros` / `sllidar_ros2` | 2.1.0-5 (Lyrical) / Jazzy release | Verified released | Upstream actively maintained; available on Jazzy and Lyrical |
| SICK / Hokuyo safety scanners | `sick_safetyscanners2` (Jazzy) / `urg_node2` | Verified telemetry | Protective function is in safety domain (hardwired OSSD); ROS telemetry is diagnostic Ethernet |
| RGB-D / Depth Cameras | `realsense2_camera`, `orbbec_camera`, `depthai-ros` | Verified released on Jazzy | Shortlist supported on Ubuntu 24.04 / Jazzy LTS |
| Motor drive interfaces | custom `ros2_control` components (`valtoryn_diff_drive`) | Required build | CiA 402 / CAN-FD via Linux SocketCAN; no third-party package dependency |

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
