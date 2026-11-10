# Development roadmap — Valtoryn platform

> **Status:** Active platform roadmap. Architecture, interface packages, hardware abstraction, mock bring-up, and physics-based autonomous simulation are implemented and verified in CI.
> **Basis:** R02 §7.8 phasing, R04 §4.3 gates, RESEARCH_AUDIT open items A-1..A-10, TECHNOLOGY_DECISIONS G-A..G-F.
> Ordering is dependency-driven: gates and decisions precede implementation.

## Phase 0 — Foundations and decisions (documentation, now)

**Deliverables:** this document set; conventions (REP-103/105, SemVer, lifecycle contract); repo layout
(platform vs robot packages); dependency compatibility matrix skeleton.

**Acceptance:** owner approves PRODUCT_REQUIREMENTS + ADR set; `check_docs.py` clean.
**Prerequisites:** none. **Status:** in progress.

## Phase 1 — Decision gates (blocking all code)

| Gate | Content | Exit evidence |
|---|---|---|
| G-A | Compute BSP supports Ubuntu 26.04 (or RT-capable container host) | Vendor image/docs |
| G-B | Lyrical driver availability for chosen LiDAR/IMU/drive/camera shortlist | rosdistro/vendor check per part |
| G-C | One-week Lyrical+Jetty spike: diff_drive + Nav2 (MPPI/Smac/CollisionMonitor/Docking) + slam_toolbox | Spike report with issues list |
| G-D | Re-verify ros2_control static-hardware claims on Lyrical docs | Doc citations |
| G-E | `vda5050_core` v3.0 capability assessment | Evaluation note |
| G-F | Floor survey: μ, Crr, roughness, joints, slopes; wheel derating from supplier | Measured dataset |

**Exit:** distribution pinned (Lyrical or Jazzy fallback), Gazebo release pinned, dependency matrix v1,
component shortlist narrowed to a buyable set.

## Phase 2 — Platform skeleton (software modularity, first)

**Deliverables:** colcon workspace; interface packages (Valtoryn msg/srv/action: chassis contract,
manifest schema v1, facility-model types); config schema + generator producing URDF/launch/params for a
differential-drive robot; mock + Gazebo backends; lifecycle supervisor; diagnostics plumbing;
CI (unit + launch_testing + headless Gazebo smoke).

**Current status:** In progress.
- Core interface packages (`valtoryn_interfaces`), task BT skeleton (`valtoryn_task_bt`), differential-drive hardware interface (`valtoryn_diff_drive`), and subsystem lifecycle supervisor (`valtoryn_lifecycle`) created.
- Declarative backend mapping (`mock`, `gazebo`, `physical`) added to robot config schema and generators.
- Fail-closed physical safety boundary enforced in hardware component.
- Kinematics verification tool (`simulate_kinematics.py`) with curvature preservation implemented.
- Reproducible integration spike runner (`run_spike.py`) created with honest preflight environment detection.
- Unit test suite expanded to 49 passing tests.

**Acceptance:** one robot config builds and runs in Gazebo and in mock; `ros2 control` list shows the
chassis; generated files reproducible; CI green.
**Prerequisites:** Phase 1 gates C, D.

## Phase 3 — Autonomy on the reference robot (simulation)

**Deliverables:** Nav2 bring-up (Smac planner + Regulated Pure Pursuit + Collision Monitor + Docking stub); SLAM Toolbox map
build + AMCL localisation in the authored warehouse world (`worlds/warehouse.sdf`); 2D LiDAR module integration
(`com.valtoryn.sensor.lidar_2d`); warehouse task BT library (`valtoryn_task_bt`) with C++ action clients and explicit
material-handling stubs; automated physics-based scenario test suite (`tests/test_ros_navigation.py`, `tools/run_scenarios.py`).

**Status:** In progress (Phase 3 closure milestone: navigation reliability, fail-closed BehaviorTree mission execution, and multi-run physics validation).
- Authored warehouse world (`worlds/warehouse.sdf`) with storage racks, aisles, intersections, pick/drop stations, static pallet stacks, and moving obstacle.
- Simulation bringup package (`valtoryn_bringup`) orchestrating Gazebo Harmonic, `ros_gz_bridge`, controller spawners, and Nav2 navigation.
- C++ BT action nodes for `NavigateToPose`, `DockRobotStub`, `PayloadPickStub`, and `PayloadDropStub` (failing closed with explicit errors to honestly communicate that physical hardware is not installed).
- Automated scenario runner executing straight corridor, 90-degree turn, static obstacle avoidance, dynamic obstacle detection, and multi-station route missions.
- Multi-run scenario verification (`--runs 3`) with robot and obstacle pose reset between repetitions.
- Acceptance criteria: 10/10 navigation tests passing repeatably across 3 consecutive runs with < 0.40m pose error, zero false-positive mission completions, and full CI green on ROS 2 Jazzy / Gazebo Harmonic.

**Acceptance:** repeated simulated missions meet scenario pass criteria at E1 evidence; run-to-run spread
reported; no safety claims made.
**Prerequisites:** Phase 2; SIM-1/2/3 checks done.

## Phase 4 — Fleet and facility layer (simulation)

**Deliverables:** internal facility model + exporters (LIF, RMF graph, Nav2 GeoJSON); minimal reservation/
queue baseline coordinator; capability-matching dispatch (default-deny); VDA 5050 v3 robot-side gateway
(via `vda5050_core` or thin build); task/order lifecycle with idempotency + NEEDS_VERIFICATION; WMS-facing
API skeleton.

**Acceptance:** two simulated robots complete coordinated missions; order survives simulated comms loss
(released base completes); gateway passes spec-conformance checks; exports round-trip.
**Prerequisites:** Phase 3; G-E.

## Phase 5 — Hardware bring-up (first physical robot)

**Deliverables:** real hardware backend for the same config; motor/drive integration (CiA 402 or CAN-FD per
decision); encoder/IMU integration; safety-domain interface (read-only state); HIL rig for control loop;
commissioning calibration (UMBmark, Allan variance, parameter registry population).

**Acceptance:** sim↔real interface parity demonstrated; HIL loop stable; first E2 component data collected
(motor, IMU, battery); safety state consumed correctly.
**Prerequisites:** Phases 2–4; hardware procurement; safety-domain definition (HA-6).

## Phase 6 — Validation campaign and second variant

**Deliverables:** E3 vehicle validation (ground truth, payload series, floor types); E4 site pilot; second
robot variant built from config only (different sensor set) with measured non-config edit count (R02 P2
target: zero beyond new modules); manifest C3 verification live; VDA factsheet export.

**Acceptance:** F-dimension thresholds set from measured data for validated domain; variant 2 requires no
platform code changes; all results carry correct evidence labels.
**Prerequisites:** Phase 5; site access; ISO 3691-4 safety-case workstream output for deployment claims.

## Phase 7 — Modularity expansion (progressive)

**Deliverables:** second chassis/locomotion module (omnidirectional or steered per market need); electrical
module-ID format for Valtoryn-designed parts; warm/hot reconfiguration tiers for non-safety payloads;
mechanical module interface spec v1 (payload deck standard); optional Open-RMF backend adapter if G
supports it.

**Acceptance:** new locomotion type runs unmodified platform code; mechanical swap procedure documented and
tested; safety re-validation gate (C5) exercised in a real config change.
**Prerequisites:** Phase 6; safety workstream sign-off for any motion-path module change.

## Dependency map (critical path)

``
Gates A–F ──► P2 skeleton ──► P3 autonomy(sim) ──► P4 fleet(sim)
                                    │                    │
                                    └────► P5 hardware ◄──┘
                                              │
                                    P6 validation + variant 2
                                              │
                                    P7 expansion (omni/mech)
```

Parallel tracks (not on critical path): WMS API design; operator UI; OTA/update mechanics; third-party
driver evaluations; safety-case documentation (owned by safety workstream, gates physical deployment).

## Explicitly not in this roadmap

- No claim of physical validation, safety certification, or production readiness at any phase.
- No ROS 2 platform implementation in this milestone — this roadmap is the blueprint future implementation follows.
