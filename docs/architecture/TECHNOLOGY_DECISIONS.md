# Technology decisions — Valtoryn platform

> **Status:** Proposed decisions with evidence. Nothing is implemented or selected finally; owner approval
> and the decision gates below are required. Detailed rationale lives in [ARCHITECTURE_DECISION_RECORDS.md](ARCHITECTURE_DECISION_RECORDS.md).
> **Basis:** R01–R08 audited research.

## 1. Decision summary

| # | Decision | Status | Basis |
|---|---|---|---|
| TD-1 | ROS 2 as middleware | **Proposed — adopt** | Only maintained ecosystem meeting all requirements; R01 §8, R04 §4.1 |
| TD-2 | ROS 2 distribution: Jazzy primary baseline, Lyrical tracked | **Proposed — active baseline Jazzy** | R04 §4.1/§4.3; Gate G-A JetPack 7.2 alignment requires Ubuntu 24.04 LTS; pending owner decision |
| TD-3 | ros2_control as hardware abstraction | **Proposed — adopt** | Closest existing match; R01 §2.3, R02 §3.2 |
| TD-4 | Nav2 as navigation stack behind Valtoryn interface | **Proposed — adopt** | Multi-kinematics, funded maintainer, plugin model; R01 §3, R06 §8.1 |
| TD-5 | SLAM Toolbox + AMCL/robot_localization for localisation | **Proposed — adopt, keep swappable** | R04 §4.1, R06 §8.5 |
| TD-6 | BehaviorTree.CPP for task orchestration | **Proposed — adopt** | Nav2 precedent; R01 §8 |
| TD-7 | Gazebo Jetty + ros_gz + gz_ros2_control as default simulator | **Proposed — adopt** | Only ROS-paired LTS; R04 §2.5, R05 §4 |
| TD-8 | Fast DDS (`rmw_fastrtps_cpp`) default RMW; Zenoh spike | **Proposed — adopt w/ spike** | Default best-supported; Zenoh Tier 1 for fleet scale; R04 §2.7, §4.6 |
| TD-9 | VDA 5050 v3 as external fleet protocol only | **Proposed — adopt spec, build gateway** | R01 §5.3, R07 §4.3 |
| TD-10 | Open-RMF as optional backend, never core | **Proposed — adopt position** | R01 §5.1, R07 §4.3 |
| TD-11 | Minimal reservation-based baseline fleet coordinator | **Proposed — build** | R01 §8, R07 §4.3 |
| TD-12 | Declarative robot config + generated artifacts + capability manifest | **Proposed — build (thin layer on ROS)** | R02 §6–7 |
| TD-13 | C++20 for control/RT/interfaces; Python for tools/launch/tests; no Rust on robot path | **Proposed — adopt** | R04 §4.1 |
| TD-14 | WSL2/dev container for Windows dev; Linux primary | **Proposed — adopt** | R04 §4.5, R05 §3.11 |
| TD-15 | Certified safety hardware separate from ROS; ROS read-only consumer | **Proposed — adopt** | R08 §4, R03 §15.5 |
| TD-16 | Buy certified battery pack/BMS; do not design cell packs in gen-1 | **Proposed — adopt** | R08 §3, R03 §11 |
| TD-17 | Differential-drive reference chassis first | **Proposed — adopt** | R03 §15.2, R04 controllers, VDA 5050 extensible kinematics |
| TD-18 | rosbag2/MCAP recording; launch_testing + Gazebo-in-loop tests | **Proposed — adopt** | R04 §4.4 |
| TD-19 | No new middleware/simulator/navigation/safety stack | **Confirmed constraint** | R01 §8 |

## 2. Explicitly rejected / deferred

| Option | Verdict | Reason |
|---|---|---|
| ROS 1 / custom middleware | Rejected | Ecosystem, lifecycle, security, fleet precedent all ROS 2 (R01) |
| Per-robot forks (alternative A in R02 §6) | Rejected | Violates core requirement |
| Model-driven composition (RobMoSys) | Deferred | Adoption unproven; vocabulary only (R02 §6) |
| Runtime self-describing hot-plug (alternative E) | Deferred/restricted | ros2_control static hardware set; safety risk (R02 §6) |
| VDA 5050 as internal architecture | Rejected | MQTT/JSON edge protocol, QoS 0, no security (R01 §5.3) |
| MassRobotics AMR Interop as primary | Deferred | Dormant since 2021, unlicensed repo (R01 §5.2) |
| Cartographer, Gazebo Classic | Rejected | Unmaintained / absent from Kilted+ (R04 §2.4–2.5) |
| Isaac Sim as default simulator | Deferred | GPU requirement, licence terms unread, Lyrical experimental (R04 §2.5) |
| rclrs (Rust) on robot path | Rejected for now | No stability guarantee, not a released ROS package (R04 §2.8) |
| Hub motors / mecanum for reference robot | Deferred | Insufficient datasheet evidence (R03) |
| First-party safety certification | Rejected | Aim "safety-ready"; buy certified components (R08 §3) |

## 3. Decision gates (must pass before freeze)

| Gate | Content | Owner of evidence |
|---|---|---|
| G-A | Onboard compute BSP supports Ubuntu 26.04 (or container+RT host kernel) | Compute selection workstream |
| G-B | Released Lyrical drivers for chosen LiDAR/IMU/motor drives/cameras | Driver survey |
| G-C | One-week Lyrical/Jetty spike: diff_drive + Nav2 (MPPI/Smac/CollisionMonitor/Docking) + slam_toolbox | Simulation workstream |
| G-D | `ros2_control` Lyrical re-verification of static-hardware claims | Architecture |
| G-E | `vda5050_core` (or alternative) v3.0 capability assessment | Fleet workstream |
| G-F | Concrete-floor `Crr`, wheel derating, caster resistance | Supplier data / measurement |

## 4. Version baseline (proposed, conditional on gates)

| Component | Version | Constraint |
|---|---|---|
| ROS 2 | Lyrical Luth (fallback Jazzy) | One LTS line per release; pin distro+Gazebo+ros_gz as a unit |
| Gazebo | Jetty (LTS to May 2031) | Pairs with Lyrical via packages.ros.org |
| ros2_control / ros2_controllers | 6.x on Lyrical | TwistStamped cmd_vel |
| Nav2 | 1.5.x on Lyrical | Per-distro plugin branches; `nav2_ros_common` types |
| slam_toolbox | 2.10.x | LGPL-2.1 — BOM tracking required |
| robot_localization | 3.11.x | Apache-2.0 |
| RMW | rmw_fastrtps_cpp default; rmw_zenoh_cpp spike | Never mix RMWs in one system |
| VDA 5050 | spec 3.0.x; library TBD | Open library lags at v2.0.0 |
| BehaviorTree.CPP | current 4.x | MIT |
| Recording | rosbag2 + MCAP | Default storage |
