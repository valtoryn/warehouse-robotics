# Simulation requirements — Valtoryn platform

> **Status:** Proposed requirements. No simulator is installed or run; no Valtoryn model exists.
> **Basis:** R05 (primary), R04 §2.5, HARDWARE_INTERFACE_SPECIFICATION §4, RESEARCH_AUDIT.
> Simulation is a validation environment, not the product. Every result carries an evidence level (E0–E4).

## 1. Purpose and non-goals

Simulation exists to (a) develop and regression-test the robot software stack before hardware, (b) produce
referent-comparable data for the hardware validation campaign, (c) exercise fleet and fault scenarios at scale.

Non-goals: safety certification evidence; performance guarantees; a substitute for hardware-in-the-loop or
physical testing.

## 2. Platform requirements

| ID | Requirement | Basis |
|---|---|---|
| SR-1 | Default simulator: Gazebo Jetty (LTS) via `ros_gz` + `gz_ros2_control`, paired with the pinned ROS 2 distro | R04 §4.1, R05 §4 [REC] |
| SR-2 | The same robot configuration (URDF + `<ros2_control>`) drives sim and hardware; only the hardware-plugin backend differs | FR-S1, R05 §5.1 |
| SR-3 | Headless CI operation: `gz sim -s`, `--iterations`, `--seed`, `--headless-rendering`; record/playback via `--record` | R05 §3.11 [FACT R05-S30] |
| SR-4 | Physics engine: DART default; a second engine run as model-form sensitivity check once scenarios exist | R05 §3.1 [FACT R05-S01] |
| SR-5 | Author own reference warehouse world (aisle widths, rack geometry, floor markings, thresholds/ramps) under project licence; third-party worlds optional with per-asset licence records | R05 §3.10 [REC] |
| SR-6 | Simulation parameters carry provenance tags: `datasheet` / `measured` / `assumed` + uncertainty + date | R05 §5.3 [REC] |
| SR-7 | Every simulation report states its evidence level and validated domain; dashboards show labels, not bare numbers | R05 §5.4, §7 |
| SR-8 | Windows dev host: simulation runs in WSL2/dev container or Linux CI; native Windows Gazebo is not a target | R04 §4.5, R05 §3.11 [FACT R05-S33] |

## 3. Physics and dynamics requirements

| ID | Requirement | Basis |
|---|---|---|
| SR-9 | Fixed-step physics (`max_step_size` 0.001 s default; evaluate convergence per F2); explicit seeds per run | R05 §3.11–3.12 |
| SR-10 | Wheel-ground contact: explicit friction values and deliberate `fdir1`; WheelSlip only after hardware slip tests justify it; `wheel_normal_force` recomputed per payload | R05 §3.2 [REC] |
| SR-11 | Actuator model: Valtoryn motor/drive model (electrical + mechanical + current limit + gearbox efficiency/backlash) behind the HAL seam — not native ros2_control joints alone | R05 §3.3 [REC] |
| SR-12 | Payload as separate rigid body parameterized by manifest (mass, COM, inertia); per-wheel normal force recomputed | R05 §3.4 [REC] |
| SR-13 | Battery: Valtoryn power model driven by simulated I·V plus base loads — Gazebo battery alone is insufficient (no torque coupling; known issues incl. 100× drain bug guard `fix_issue_225`) | R05 §3.7 [FACT R05-S04] |
| SR-14 | Latency: injectable delay/jitter/dropout stage for sensor and command paths, configured from measured distributions | R05 §3.8 [REC] |
| SR-15 | Actors permitted for perception/planning tests only; physical-contact scenarios use dynamic bodies | R05 §3.9 [FACT R05-S29] |

## 4. Sensor modelling requirements

| ID | Requirement | Basis |
|---|---|---|
| SR-16 | Baseline noise: SDF documented models (Gaussian + bias + quantisation for generic/IMU; Gaussian per-beam lidar; per-pixel camera) | R05 §3.5 [FACT R05-S06] |
| SR-17 | Structured degradation (range/reflectance-dependent lidar error, correlated noise, dropout) added as measured hardware data arrives — i.i.d. Gaussian alone proves little | R05 §3.5 [EST] |
| SR-18 | Sensor latency is modelled by the SR-14 stage, not assumed inside the simulator | R05 §3.5 [REC] |
| SR-19 | Safety scanners are simulated for perception only; protective functions are never simulated | HARDWARE_INTERFACE_SPEC §4, R08 §4 |

## 5. Scenario and test requirements

| ID | Requirement | Basis |
|---|---|---|
| SR-20 | Test taxonomy levels: unit, component, system, scenario, regression, HIL — separate CI jobs | R05 §6 |
| SR-21 | Regression gates are statistical: N repeats, fixed seed/step, distributional metrics; run-to-run spread tracked as a quality metric (F1) | R05 §3.12, §5.7 |
| SR-22 | Fault-injection scenarios required: sensor loss, comms outage (QoS pairs + netem profiles), low battery, payload change, drive fault, FINALIZED recovery | R05 §3.8, R06 §8.8, R02 R17 |
| SR-23 | Multi-robot scenarios for fleet coordination testing; measured RTF vs robot/sensor count recorded (scaling is unresolved — measure, don't assume) | R05 §3.11 [UNRESOLVED] |
| SR-24 | Determinism is a measured property: fixed seed/step + N-repeat spread monitor; no bitwise assumption | R05 §3.12, issue #3349 |
| SR-25 | Channel-impairment profiles (delay/loss/burst) replayed via `tc-netem` where the CI/dev environment supports it; profile provenance recorded | R05 §3.8 [REC]; U9 open |

## 6. Fidelity metrics (adopted from R05 §8, thresholds pending hardware)

Dimensions F1–F20 stand as the fidelity framework; current status of every threshold is **"no hardware
data"**. Engineering use before hardware: verification-level claims only (E0/E1). The acceptance form is
the R05 §8 overlap-of-uncertainty-intervals rule with δ set from measured hardware data.

## 7. Simulation-to-hardware validation path

1. E0/E1 now: software executes scenarios; logic/interface checks; relative algorithm comparisons
   labelled simulation-only.
2. Component validation (E2): motor dyno, sensor noise logs, battery discharge → subsystem models.
3. Vehicle validation (E3): full-stack sim vs real-unit measurements in representative environment —
   motion-capture or surveyed ground truth (cf. Collins et al. method, R05-S52).
4. Operating-environment validation (E4): site-matched fleet KPIs vs logged operation.
5. Digital-twin staging: simulation model → calibrated model → digital twin (synchronised state),
   per ISO 23247 vocabulary — "digital twin" is not used for unsynchronised models (R05 §3.14).

## 8. Open items

| ID | Item | Source |
|---|---|---|
| SIM-1 | Jetty ↔ Open-RMF demos / `gz_ros2_control` compatibility check | R05 U5 |
| SIM-2 | DART handling of SDF friction elements (`mu`/`mu2`/`slip`, `fdir1` anisotropy) — source read + controlled test | R05 U1 |
| SIM-3 | WheelSlip normal-force behaviour (constant vs dynamic) | R05 U2 |
| SIM-4 | `--seed` coverage across gz-sim components; N-repeat experiment | R05 U3 |
| SIM-5 | RTF vs robot/sensor count benchmark on CI hardware | R05 U4 |
| SIM-6 | tc/netem availability in CI/dev environments | R05 U9 |
| SIM-7 | Warehouse world authoring (geometry from site survey) | SR-5 |
| SIM-8 | Which simulation evidence (if any) ISO 3691-4:2023 verification accepts | R05 U8 → safety workstream |
| SIM-9 | Gazebo battery `fix_issue_225` verification on pinned version | R05 §3.7 |
