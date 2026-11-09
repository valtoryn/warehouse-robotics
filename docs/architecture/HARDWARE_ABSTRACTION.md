# Hardware abstraction — Valtoryn platform

> **Status:** Proposed specification. Nothing is implemented.
> **Basis:** R02 §3.2/§5, R03 §15, R04 §2.2, R05 §5.1, R08 §4.

## 1. Boundary definition

The hardware-abstraction layer (HAL, architecture layer L3) is the single seam between robot software and
physical or simulated hardware. Everything above it (autonomy, core, fleet, applications) is identical across
backends; everything below it is backend-specific.

```
robot software ──► ros2_control (Controller Manager + Resource Manager)
                        │ pluginlib hardware components
                        ▼
        ┌───────────────┼───────────────────┐
        ▼               ▼                   ▼
   real hardware   Gazebo (gz_ros2_control)  mock hardware
   (drivers, MCU,  (simulated joints,        (GenericSystem /
    safety read-    sensors, physics)         loopback for tests)
    only link)
```

- Real-time control runs inside ros2_control's controller manager loop (default 100 Hz, configurable via
  `update_rate`; RT scheduling via `thread_priority`, `cpu_affinity`, `lock_memory`) (R04 §2.2 [FACT]).
- Safety functions are **not** in this layer's command path: the safety domain is separate hardware; HAL only
  *reads* safety state (R08 §4).

## 2. ros2_control structure (adopted as-is)

| Element | Role | Basis |
|---|---|---|
| Resource Manager | Loads hardware components via pluginlib; manages lifecycle and command/state interfaces | R04 §2.2 [FACT] |
| Controller Manager | Loads/activates controllers; matches required↔provided interfaces; runs read→update→write loop | R04 §2.2 |
| Hardware component types | System (multi-DOF, one channel), Sensor (read-only), Actuator (1-DOF) | R04 §2.2 [FACT] |
| URDF `<ros2_control>` tag | Declares components, joints, sensors, `<gpio>`, command/state interfaces with types, min/max/initial values, groups | R02 §3.2 [FACT] |
| Interface data types | double default; float32, bool, int/uint 8–32 via `data_type` | R02 §3.2 [FACT] |
| Limit enforcement | `enforce_command_limits` default true; limits come from robot description — the description is an enforced input | R02 §3.2 [FACT] |
| `fallback_controllers` | Safety-relevant: configured per robot | R02 §3.2 |

**Known limitations (must re-verify on pinned Lyrical — RESEARCH_AUDIT A-4):** URDF reload unsupported;
hardware component reaching FINALIZED is unrecoverable without plugin reload and no reload service exists
(R02 §3.2, §5.6 [FACT on Rolling docs]). Design consequence: cold reconfiguration (Tier 0) is the default;
recovery from FINALIZED is a supervisor process restart, and this path must be tested (R02 R17).

## 3. Chassis kinematics interface (Valtoryn addition)

ros2_control abstracts joints, not locomotion class. Valtoryn adds a **Chassis** contract above it:

| Member | Content | Basis |
|---|---|---|
| `kinematics_type` | Extensible enum (DIFFERENTIAL, OMNIDIRECTIONAL, ACKERMANN, TRICYCLE, SWERVE, …) | R03 §2.1, VDA 5050 extensible kinematics [FACT R02-S27] |
| `forward/inverse` | Twist↔wheel mapping per chassis (diff: `v=r(uL+uR)/2`, `ω=r(uR−uL)/2d`; mecanum/omni/swerve/Ackermann formulas in R03 §2.1) | R03 §2.1 [CALC from R03-S25] |
| `odom` | Wheel odometry with per-chassis covariance; systematic-error calibration (UMBmark-style for diff drive) | R03 §2.2, R03-S29 [FACT] |
| `limits` | Wheel speed, acceleration, steering limits; declared in manifest | R02 R9 |
| `calibration` | Commissioning routine per chassis | R03 §15.1 [REC] |

Controllers used per kinematics (existing, R04 §2.2 [FACT]): `diff_drive_controller`,
`mecanum_drive_controller`, `omni_wheel_drive_controller`, `steering_controllers_library` (bicycle/tricycle/
Ackermann). No swerve controller located — swerve support is [UNRESOLVED] pending a custom controller or
upstream contribution (R03 unresolved #3).

## 4. Actuator and drive boundary

Two swappable backends behind one interface (R03 §15.6 [REC]):

| Backend | Interface | Status |
|---|---|---|
| CiA 402 over CANopen | Industrial drives (e.g. Oriental BLV-R class) | `ros2_canopen` self-declared "not yet ready for production" [FACT R03-S49]; needs conformance testing |
| CAN/CAN-FD | ODrive-class drives | `odrive_node` ROS 2 package exists [FACT R03-S46] |

Requirements on the interface: velocity/effort command + position/velocity feedback, current/torque limits,
STO status read-back (read-only to ROS), fault reporting. Functional-safety signals (STO, SS1, safe speed) are
consumed as *state*; actuation of them stays in the safety domain (R08 §4, R03 §15.5).

## 5. Sensor boundary

| Class | Examples (shortlist, not selected) | Notes |
|---|---|---|
| Safety LiDAR | SICK nanoScan3, Hokuyo UAM-05LPA | Safety outputs stay in safety domain; ROS driver publishes raw data only — raw ROS data carries no safety function [FACT R03-S03] |
| Navigation LiDAR | RPLIDAR S2E, Ouster OS1, Livox Mid-360 | Driver availability on pinned distro is Gate B (R04 §4.3) |
| IMU | Bosch BMI088 class | Mounted rigidly near rotation centre; Allan-variance calibration required [R03 §8] |
| Depth camera | RealSense D400, Orbbec Gemini, Luxonis OAK | Front obstacle/pallet-pocket sensing; minimum-Z drives mount height [R03 §9] |

Collision Monitor constraint (R08): Nav2 Collision Monitor may use only sensors **not shared** with the safety
layer; never configure it on safety-scanner data as if it were protective.

## 6. Power boundary

- Battery/BMS protection path is safety-relevant hardware, outside ROS command path (R03 §11, R08 §4).
- ROS-visible power interface: state of charge, pack voltage, charge state, fault flags — via a ros2_control
  Sensor/GPIO component or diagnostics.
- Charging handshake (digital, e.g. CAN) before charge current enable; contact-dock design is an open item
  (R03 unresolved #13).
- Simulation battery model is insufficient alone (no torque coupling, known Gazebo issues) → Valtoryn power
  model behind this seam (R05 §3.7).

## 7. Compute boundary

- Primary: Jetson Orin class (AI workloads) or industrial x86 — decision gated on Ubuntu 26.04 BSP
  availability (R04 Gate A, R03 §12). Pi 5 acceptable as gateway/diagnostic node.
- MCU layer (STM32 class): hard real-time wheel/steer loops, brake and watchdog handling; micro-ROS optional
  convenience bridge, never a safety layer (R03 §12 [REC]).
- Real-time path: motor control inside ros2_control CM loop and/or MCU; ROS graph is not the RT path for
  anything safety-adjacent (R04 §2.8, R08 §4).

## 8. Backend conformance contract

Every backend (real, Gazebo, mock, HIL) must implement the same interface set for a given configuration:

| Requirement | Detail |
|---|---|
| Declarative backend selection | Selected in robot configuration via `target.backend` enum (`mock`, `gazebo`, `physical`) |
| Plugin mapping in URDF generator | `mock`: `<plugin>valtoryn_diff_drive/DiffDriveHardware</plugin>` with `mock_hardware: true`.<br>`gazebo`: `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` with `<gazebo>` system plugin.<br>`physical`: `<plugin>valtoryn_diff_drive/DiffDriveHardware</plugin>` with `mock_hardware: false`. |
| Same command/state interface names, types, units, limits | Limits enforced identically (mock may relax for testability, declared in config) |
| Same lifecycle semantics | configure→activate→read/write→deactivate→cleanup; error→FINALIZED |
| Fail-closed physical safety | Unsupported or unconfigured physical backends return `CallbackReturn::ERROR` during configuration/activation and `return_type::ERROR` during read/write to prevent unauthorized or fabricated physical execution |
| Same diagnostics topics | Simulator reports simulated faults through the same path |
| Determinism label | Real: physical; sim: statistical (N-run gates per R05 §5.7) |

Mock hardware (`valtoryn_diff_drive/DiffDriveHardware` with loopback) enables controller and launch testing without
hardware; it is the default backend for CI unit and integration tiers. Gazebo physics simulation is supported via
the standard `gz_ros2_control` system plugin. Physical hardware mode fails closed until genuine CAN drivers are wired.

## 9. Error handling contract

| Failure | Detected by | Response |
|---|---|---|
| Hardware component error | ros2_control `on_error` → FINALIZED | Supervisor restarts control process; robot requests safe state via task layer (never direct safety command) |
| Driver comms loss | Heartbeat/watchdog in driver | Lifecycle error; Nav2 bond-style degradation |
| Sensor dropout | Diagnostics + Nav2 costmap staleness | Costmap filters, degraded-mode policies per config |
| Safety state change | Read-only safety channel | Robot follows defined safe state for current mode; autonomy stack cannot override |
| Backend mismatch (sim vs declared profile) | Bring-up C3 identity check | Fail closed |

## 10. Open items (blocking freeze of this spec)

| ID | Item | Status |
|---|---|---|
| HA-1 | Re-verify "no URDF reload / FINALIZED unrecoverable" on Lyrical ros2_control docs | Open (A-4) |
| HA-2 | Chassis interface formal schema (message defs) | To be written with first implementation |
| HA-3 | Swerve controller availability | Unresolved (R03 #3) |
| HA-4 | CiA 402 conformance of chosen drives; `ros2_canopen` maturity | Unresolved (R03 #14) |
| HA-5 | MCU↔compute link choice (CAN vs Ethernet) and protocol | Open — decide with compute selection |
| HA-6 | Safety-state channel physical/electrical definition | Owned by safety case workstream (R08 programme) |
