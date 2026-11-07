# Decision gates — Phase 1 evidence record

> **Status:** Living record of Phase 1 gate evidence. Gates G-A..G-F block the distribution pin and
> Phase 2 code per [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md#phase-1--decision-gates-blocking-all-code).
> Evidence tags per CLAUDE.md; this document records what was checked, when, and what remains open.
> Checked 2026-11 (web sources; see citations inline).

## Summary

| Gate | Question | Status | Evidence |
|---|---|---|---|
| G-A | Compute BSP supports Ubuntu 26.04 (Lyrical base) | **At risk — likely fails natively** | NVIDIA Jetson Orin Nano ships JetPack 7.2 on **Ubuntu 24.04**, not 26.04 |
| G-B | Lyrical driver availability for shortlist | **Partially verified — one blocker found** | `rplidar_ros` released for Lyrical; `sicktoolbox` unmaintained since 2019 |
| G-C | One-week Lyrical+Jetty spike | **Not started — requires ROS environment** | Blocked on G-A/G-B outcomes and a Linux/ROS host |
| G-D | `ros2_control` static-hardware claims on Lyrical | **Verified with API caveats** | Lyrical docs live; Kilted→Lyrical migration documents breaking interface changes |
| G-E | `vda5050_core` v3.0 capability | **Not passed — library lags spec** | `vda5050_interfaces` implements spec **2.0.0** only; master API experimental |
| G-F | Floor survey: μ, Crr, roughness, wheel derating | **Not started — requires physical measurement** | Needs site access and supplier data |

## G-A — Compute platform vs Ubuntu 26.04

- Lyrical Luth targets **Ubuntu Resolute (26.04)** per rosdistro `lyrical/distribution.yaml`
  (`release_platforms.ubuntu: [resolute]`).
- Jetson Orin Nano's current JetPack line (7.2.x, Jetson Linux r39.x) ships **Ubuntu 24.04**
  (NVIDIA Jetson Linux Developer Guide, BSP setup; NVIDIA forum confirmation that Orin Nano receives
  JetPack 7.2 on Ubuntu 24.04 — "orin nano will receive jetpack 7.2 ubuntu 24.04").
- Community container path exists (`jetson-containers` Ubuntu 24.04 images with CUDA) but CUDA
  compatibility inside a 26.04 container on a 24.04 JetPack host is **not guaranteed** — NVIDIA forum
  discussion flags this explicitly.
- **Implication:** either (a) fall back to **Jazzy** (Ubuntu 24.04 Noble — matches JetPack 7.2) for the
  first hardware target, keeping Lyrical for simulation/dev where the host controls the OS; or
  (b) select compute whose BSP tracks Ubuntu 26.04 (unverified — no candidate confirmed); or
  (c) accept a container+RT-kernel architecture, which needs its own spike.
- **Status: gate not passed.** This is now the dominant decision input for TD-2.

## G-B — Driver availability on Lyrical (partial)

Shortlist from [REFERENCE_ROBOT_SPECIFICATION.md](REFERENCE_ROBOT_SPECIFICATION.md) §sensors:

| Component | Lyrical status | Notes |
|---|---|---|
| RPLIDAR S2E-class nav LiDAR | **Available** | `ros-lyrical-rplidar-ros` 2.1.0-5 published on packages.ros.org; supports S2/S2E. Caveat: upstream release is stale (2.1.0 vs 2.1.4 on older distros; rosdistro issue #49653 open). Acceptable for reference robot. |
| SICK nanoScan3 safety scanners | **No maintained ROS driver** | `sicktoolbox` last updated 2019, UNMAINTAINED, not released for Lyrical. Protective-field function runs in the safety domain anyway; ROS raw-data access would need a custom SOPAS-based driver or vendor SDK. **Action:** safety scanners' ROS telemetry is non-blocking (informational); treat as open item. |
| Hokuyo UAM-05LPA (alt safety scanner) | **Unverified** | No Lyrical release confirmed; same safety-domain caveat applies. |
| BMI088-class IMU | **Not a rosdistro concern** | Typically read by onboard MCU and published over serial/CAN — fits the MCU-bridge architecture (HIS §compute). No driver gate. |
| Motor drives (CiA 402 / CAN-FD) | **Custom component required** | `ros2_control` hardware components for these drives are written by Valtoryn regardless of distro; no off-the-shelf dependency. Gate reduces to "ros2_control works on Lyrical" = G-D. |
| Cameras (shortlist TBD) | **Unverified** | Per-part check required when shortlist is fixed. |

- **Status: gate not fully passed.** LiDAR requirement satisfied; safety-scanner ROS telemetry is an
  open item, not a blocker (protective function is outside ROS by design — ADR safety boundary).

## G-C — Lyrical + Jetty spike

- Not executed. Requires a Linux/ROS environment and is downstream of G-A (host OS decision) and
  G-B (component shortlist). The spike scope is fixed by the roadmap: diff_drive + Nav2
  (MPPI/Smac/CollisionMonitor/Docking) + slam_toolbox on Lyrical+Jetty, with an issues list.
- **Status: pending — blocked on G-A decision and a spike host.**

## G-D — ros2_control on Lyrical (verified, with caveats)

- Lyrical documentation is live at `control.ros.org/lyrical` (Getting Started, CLI, release notes,
  migration guides).
- **Breaking changes Kilted → Lyrical that affect any code written against older docs:**
  - `hardware_interface`: `export_state_interfaces()` / `export_command_interfaces()` **removed**;
    interfaces are now created by the framework from the `<ros2_control>` URDF tag; components use
    `set_state`/`get_state`, `set_command`/`get_command`. (migration guide, PRs #3610/#2988)
  - `HardwareInfo` → `HardwareComponentParams` in `initialize()` (#2323, #2589).
  - `StateInterface`/`CommandInterface` no longer constructible from `double*`; handles own values.
  - New handle data types: `float32`, `uint8/16/32`, `int8/16/32`, `bool`, `double` (#2879).
  - Chainable controller API renamed to `on_export_*_interfaces_list()`.
  - Controller-manager behaviour changes: `configure_controller` single-step only (#3196),
    `switch_controller` strict by default (#2742), failing controller deactivates its chain (#2681).
- **Static-hardware model confirmed unchanged:** the Resource Manager loads hardware components as
  pluginlib plugins declared in the robot's `<ros2_control>` description — i.e., the hardware set is
  still fixed at configuration/launch time. The architecture's cold-reconfiguration default
  (MODULARITY_SPEC §7 Tier 0) remains correct.
- **Status: gate passed for the claims that matter** (static hardware set, plugin model, lifecycle).
  Caveat recorded: any implementation must target Lyrical APIs, not Jazzy-era snippets.

## G-E — VDA 5050 v3 library capability (not passed)

- `ros-industrial/vda5050_interfaces`: message definitions follow **VDA 5050 spec 2.0.0** explicitly;
  package self-described as under active development with breaking-change warning.
- `ros-industrial/vda5050_core` (formerly `vda5050_client`): provides client adapter + an
  **experimental** `VDA5050Master` API (warns "public interfaces may change"); master code was moved
  to a standalone package in `vda5050_client` PR #44 (closed 2026-05).
- Official spec is at **v3.0.0** (VDA5050 GitHub; v3.1.0 planned non-breaking).
- **Conclusion:** no mature ROS 2 library implements spec v3.0 yet. Phase 4 gateway work must either
  (a) build a thin VDA 5050 v3 serializer/adapter against `vda5050_interfaces`-style messages,
  (b) contribute v3 support upstream, or (c) implement the JSON/MQTT mapping directly per spec.
  This was already the documented contingency in TECHNOLOGY_DECISIONS TD-9 / roadmap Phase 4.
- **Status: gate not passed — proceed with thin-build plan, revisit before Phase 4.**

## G-F — Floor survey

- Requires physical measurement: concrete-floor rolling resistance `Crr`, µ, roughness, joints,
  slopes, and wheel/caster derating from supplier data. No site access or hardware exists yet.
- Feeding unresolved item A-3 (concrete `Crr`/wheel derating) from RESEARCH_AUDIT.
- **Status: pending — requires site access and hardware procurement.**

## Consequence for the distribution pin (TD-2)

Evidence now favours:

1. **Jazzy LTS as the first-hardware-target distro** (Ubuntu 24.04 ↔ JetPack 7.2 alignment), unless
   compute selection changes (G-A option b/c).
2. **Lyrical retained for development/simulation** where the host OS is controllable, pending the
   G-C spike — or deferred entirely if the team standardises on one distro.
3. Whichever distro is pinned, **all ros2_control code must target that distro's API** — the
   Kilted→Lyrical breaking changes show how much API drift exists between adjacent distros.

This remains a *proposal* for owner decision; the pin is not frozen.
