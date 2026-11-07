# Validation strategy — Valtoryn platform

> **Status:** Proposed strategy. No hardware exists; every hardware-dependent threshold is pending measurement.
> **Basis:** R05 §6–8 (test taxonomy, E0–E4, F1–F20), R08 §5 (L0–L4 evidence ladder), R06 §8.8, R02 §5.4.
> One ladder is used throughout: **E0–E4** (simulation evidence) and **L0–L4** (safety-case evidence,
> R08) are different axes — software/sim evidence never substitutes for safety validation.

## 1. Test pyramid

| Tier | Scope | Environment | Pass criterion |
|---|---|---|---|
| Unit | Functions/classes, no sim | Host CI (colcon test, gtest/pytest) | Deterministic; bitwise where applicable |
| Component | One module, simulated/recorded I/O | Mock hardware (`GenericSystem`) + recorded bags | Contract tests vs interface spec |
| Integration | Full node graph | launch_testing + Gazebo headless | Scenario assertions; N-run statistics |
| Scenario | Defined situations w/ pass/fail | Gazebo worlds + fault injection | Distributional gates (R05 §5.7) |
| Regression | Fixed scenario set per change | CI nightly + per-PR subset | Metrics within tolerance bands; spread monitored |
| HIL | Real compute/firmware/drives, simulated plant | Bench rig | Timing/interface/fault behaviour on real paths |
| Physical | Real robot, real environment | Site trials | Referent data for E2–E4 validation |

## 2. Evidence discipline (binding rules)

1. Every test result is labelled with evidence level E0–E4 (R05 §7). E0/E1 = simulation-only claims;
   E2+ require hardware referent data.
2. Simulation results are never cited as physical validation or safety evidence (FR-S2, R08 §5).
3. Statistical gates: N repeats, fixed seeds, distributional metrics; no bitwise determinism assumptions
   (R05 §3.12; gz-sim issue #3349).
4. Parameter provenance registry: every sim parameter tagged datasheet/measured/assumed + uncertainty;
   assumed parameters cap dependent results at E0/E1.
5. Safety validation follows R08's L0–L4 ladder independently; ROS software contributes only
   safety-*adjacent* regression (e.g. safe-state request behaviour), never protective-function evidence.
6. Payload-state switching, fault injection and comms-loss scenarios are mandatory scenario classes
   (FR-A3, FR-S4).

## 3. Coverage requirements by subsystem

| Subsystem | Required coverage |
|---|---|
| Chassis kinematics | Forward/inverse correctness per kinematics type; odometry covariance; UMBmark calibration procedure (diff drive) |
| Controllers | cmd_vel timeout, limits enforcement, fallback controllers, odometry output |
| Localisation | Consistency metrics (NEES) vs ground truth; recovery after displacement; aisle/repetitive-environment campaign (highest-risk item, R06 §3) |
| Navigation | Planner/controller plugin conformance per drivetrain; costmap filter behaviour; docking accuracy per dock type |
| Fleet/facility model | Export round-trips (LIF/RMF/GeoJSON/VDA); capability-match default-deny; order base/horizon semantics; comms-loss completion of released base |
| Manifest/config | C1–C5 gate tests incl. fail-closed C3 mismatch and C5 safety-hash block |
| Power | Low-battery behaviour paths; charge handshake; SoC estimation vs logged discharge |
| Diagnostics | Every module publishes diagnostics; bond failure → controlled degradation |

## 4. Hardware validation campaign (to be executed when hardware exists)

Sequenced per R05 §5.9 and F-dimensions:

1. **Bench/component:** motor dyno (torque-speed-current), encoder resolution, IMU Allan variance,
   sensor noise logs, battery discharge curves → E2 subsystem models.
2. **Vehicle:** ground-truth rig (motion capture or surveyed marks) for odometry/dynamics/braking/
   stopping-distance across payload levels and floor types → E3.
3. **Site:** matched mission KPIs vs logged fleet operation → E4; comms profiles from site Wi-Fi
   packet logs feed netem replay (R05 F16, F17, F19).
4. **Safety:** separate track per R08 programme — this document does not define protective-function
   verification.

## 5. Regression and release gates

- Per-PR: unit + component + smoke scenario subset.
- Nightly: full scenario suite with N repeats; run-to-run spread report.
- Release: compatibility-matrix check (ROS distro, Gazebo, Nav2, RMF, VDA lib versions) + licence BOM
  audit (Nav2 mixed licences; SLAM Toolbox LGPL) + manifest schema validation.
- Field: rosbag2/MCAP snapshot capture on any fault event; post-incident replay in sim where possible.

## 6. Open items

| ID | Item |
|---|---|
| VS-1 | N (repeat count) and tolerance bands per scenario — set from measured hardware spread |
| VS-2 | Ground-truth method selection (motion capture vs surveyed) per test site |
| VS-3 | ISO 3691-4:2023 verification requirements — licensed standard needed (A-5) |
| VS-4 | Trailer/towing hazard analysis for tugger variants (A-6) |
| VS-5 | HIL rig definition (which real components: compute? drives? MCU?) |
| VS-6 | Aisle-localisation campaign design (R06 unresolved #1) |
