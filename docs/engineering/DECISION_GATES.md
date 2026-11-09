# Decision gates — Phase 1 evidence record

> **Status:** Living record of Phase 1 gate evidence. Gates G-A..G-F block the distribution pin and
> Phase 2 code per [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md#phase-1--decision-gates-blocking-all-code).
> Evidence tags per CLAUDE.md; this document records what was checked, when, and what remains open.
> Last updated: 2026-11.

## Summary

| Gate | Question | Status | Evidence |
|---|---|---|---|
| G-A | Compute BSP supports Ubuntu 26.04 (Lyrical base) | **Decision required — hardware conflict confirmed** | NVIDIA Jetson Orin Nano ships JetPack 7.2 on **Ubuntu 24.04** (Noble), not 26.04. Strong recommendation: standardize on **ROS 2 Jazzy Jalisco** (LTS to May 2029) for both hardware and simulation. |
| G-B | Driver availability for shortlist | **Verified for Jazzy; partially open for Lyrical** | LiDAR (`sllidar_ros2`/`rplidar_ros`) released on Jazzy and Lyrical. Safety scanner telemetry (`sick_safetyscanners2`) maintained on Jazzy (replacing stale `sicktoolbox` citation). Cameras (`realsense2_camera`, `orbbec_camera`, `depthai-ros`) released on Jazzy. IMU and motor drives use MCU/SocketCAN interfaces. |
| G-C | Integration spike (diff_drive + Nav2 + Gazebo + ros_gz) | **Pending spike host** | Host environment is Windows 11 without running Linux/ROS 2/Gazebo; blocker recorded honestly. Containerized/CI test plan established. |
| G-D | `ros2_control` static-hardware claims and API stability | **Verified with distro-specific API constraints** | Static hardware topology confirmed in both Jazzy and Lyrical. Jazzy uses stable `export_*_interfaces` / `HardwareInfo`; Lyrical (6.x) introduces breaking API changes. |
| G-E | VDA 5050 v3 library capability | **Not passed — thin gateway adopted** | `ros-industrial/vda5050_interfaces` implements spec **2.0.0** only; master API experimental. Proceeding with Valtoryn thin VDA 5050 v3 JSON/MQTT gateway (TD-9). |
| G-F | Floor survey: μ, Crr, roughness, wheel derating | **Pending physical site survey and supplier data** | Formula chain complete (R03 §10); requires site access and manufacturer derating for continuous AMR operation. |

## G-A — Compute platform vs OS distribution

### The Conflict

- **ROS 2 Lyrical Luth** targets **Ubuntu 26.04 LTS (Resolute Ringtail)** per rosdistro `lyrical/distribution.yaml` (`release_platforms.ubuntu: [resolute]`).
- **NVIDIA Jetson Orin Nano** software support is defined by NVIDIA JetPack:
  - JetPack 6.x (L4T r36.x) runs Ubuntu 22.04 LTS (Jammy).
  - JetPack 7.x (Jetson Linux L4T r39.x) ships on **Ubuntu 24.04 LTS (Noble Numbat)**.
  - NVIDIA does not provide an Ubuntu 26.04 BSP for Jetson Orin Nano, and historically lags new Ubuntu LTS releases by 12–18 months.
- **Container limitations:** NVIDIA Container Toolkit passes host driver libraries (`/usr/lib/aarch64-linux-gnu/tegra/*`) and CUDA user-space components into containers. Running an Ubuntu 26.04 container on a 24.04 host kernel introduces glibc ABI mismatches and breaks hardware acceleration (CUDA, TensorRT, DLA, and `nvarguscamerasrc` hardware video pipelines). Containers do not resolve kernel BSP incompatibility.

### Compute & Distribution Options (Owner Trade-offs)

Three mutually exclusive paths exist for the project owner:

1. **Option 1 (Strong Engineering Recommendation): Standardize on ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS.**
   - *Target Compute:* NVIDIA Jetson Orin Nano (JetPack 7.2).
   - *Operating System:* Ubuntu 24.04 LTS (Noble Numbat) on both physical hardware and simulation/development hosts.
   - *ROS Distribution:* ROS 2 Jazzy Jalisco (LTS supported through May 2029).
   - *Pros:* 100% native compatibility with JetPack 7.2; hardware GPU/DLA acceleration works out of the box; mature, released binary packages for all dependencies (`ros2_control`, `nav2`, `slam_toolbox`, camera drivers); eliminates dual-distro development overhead and API drift.
   - *Cons:* Does not use the newest ROS 2 release (Lyrical Luth).

2. **Option 2: Alternative Compute Module tracking Ubuntu 26.04.**
   - *Target Compute:* x86-64 Industrial PC (e.g. Advantech, Neousys with Intel Core / AMD Ryzen Embedded) or Raspberry Pi 5.
   - *Operating System:* Ubuntu 26.04 LTS (native).
   - *ROS Distribution:* ROS 2 Lyrical Luth.
   - *Pros:* Tracks Lyrical natively without NVIDIA BSP lag.
   - *Cons:* x86 IPCs consume 45W–100W+ (drastically degrading AMR battery life and runtime compared to Orin Nano's 15W envelope); high BoM cost ($800–$2,000+ vs ~$300–$500); Raspberry Pi 5 lacks GPU/Tensor cores needed for real-time warehouse perception and neural obstacle detection.

3. **Option 3: Dual-Distro Split (Lyrical on Simulation Host, Jazzy on Robot Hardware).**
   - *Pros:* Enables experimenting with Lyrical features in simulation.
   - *Cons:* Severe API drift between Jazzy and Lyrical (especially `ros2_control` 6.x breaking changes documented in G-D); requires double maintenance and shims across every interface and controller.

**Recommendation:** Proceed with **Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS)** as the primary target for both physical hardware and simulation, while keeping the architecture modular so future migrations require configuration updates rather than redesigns.

## G-B — Driver availability

Status across candidate sensors and actuators:

| Component | Status | Evidence / Notes |
|---|---|---|
| 2D Nav LiDAR (RPLIDAR S2/S2E/C1) | **Available & Maintained** | `ros-jazzy-rplidar-ros` / `ros-jazzy-sllidar-ros2` officially released and actively maintained. Lyrical also has 2.1.0-5. Meets reference robot navigation requirements. |
| Safety Laser Scanners (SICK nanoScan3 / microScan3) | **Telemetry Maintained; Safety Independent** | `sick_safetyscanners2` is actively maintained for ROS 2 (Humble, Jazzy) over Ethernet (COPC/SOPAS). Note: earlier citation of `sicktoolbox` was outdated (legacy ROS 1). Crucially, the protective safety stop is hardwired via OSSD to the safety controller/relay and motor STO; ROS telemetry is strictly non-safety diagnostic data (TD-15). |
| Alternative Safety Scanners (Hokuyo UAM-05LPA) | **Telemetry Maintained** | `urg_node2` provides Ethernet scanner telemetry on ROS 2. Same safety domain boundary applies. |
| IMU (Bosch BMI088 class) | **Architecture Decoupled** | Read by onboard real-time MCU (STM32/RP2040) over SPI and published to ROS via serial/CAN bridge as `sensor_msgs/msg/Imu`. Eliminates OS-level kernel driver dependency. |
| Motor Drives (CiA 402 / CAN-FD) | **Valtoryn Hardware Component** | Custom `ros2_control` hardware component (`valtoryn_diff_drive/DiffDriveHardware`) interfacing via Linux SocketCAN (kernel native). No third-party rosdistro driver dependency. |
| Depth / RGB-D Cameras | **Available on Jazzy** | Intel RealSense (`realsense2_camera`), Orbbec (`orbbec_camera`), and Luxonis OAK-D (`depthai-ros`) are all released and supported on ROS 2 Jazzy. |
| Battery Management System (BMS) | **Architecture Decoupled** | Smart BMS communicating over CAN/SMBus; published to `sensor_msgs/msg/BatteryState` and Valtoryn `SafetyStatus.msg`. |

**Status: Gate G-B passed for ROS 2 Jazzy baseline.**

## G-C — Integration spike

### Requirements for Meaningful Spike
The integration spike must validate the end-to-end software pipeline:
1. `ros2_control` with `diff_drive_controller` commanding velocity joints.
2. Gazebo simulation (`ros_gz_sim` and `gz_ros2_control`) simulating physics and dynamics.
3. Odometry feedback published on `/odom` and TF published on `/tf` (`odom` -> `base_link`).
4. Nav2 stack (controller_server, planner_server, bt_navigator, costmaps) generating valid paths and velocity commands.
5. RViz2 / headless telemetry verifying convergence to goal poses without oscillation.

### Current Machine State & Blocker
- Current local host: Windows 11 with Python 3.13.5.
- ROS 2 and Gazebo are not installed natively on Windows.
- WSL instance `docker-desktop` is stopped; no native Ubuntu WSL environment is available.
- **Honest status:** The full running simulation spike cannot execute directly within the current Windows shell.
- **Resolution:** A reproducible integration test script and headless CI container configuration are specified to run the spike under Linux/Ubuntu CI without fabricating local test results.

## G-D — ros2_control compatibility and static hardware model

- **API Comparison (Jazzy vs Lyrical):**
  - **ROS 2 Jazzy (ros2_control 4.x / 5.x):**
    - `hardware_interface::SystemInterface` uses `on_init(const HardwareInfo & info)`.
    - Components explicitly implement `export_state_interfaces()` and `export_command_interfaces()`.
    - Handles bind directly to member variables (`double*`).
  - **ROS 2 Lyrical (ros2_control 6.x):**
    - `export_state_interfaces()` and `export_command_interfaces()` are removed; interfaces are instantiated by the framework from `<ros2_control>` URDF tags.
    - `HardwareInfo` is replaced by `HardwareComponentParams`.
    - Components use `set_state()`, `get_state()`, `set_command()`, `get_command()`.
- **Architectural Invariance:**
  - In both distributions, the hardware topology is statically loaded by the `Resource Manager` from the URDF at startup.
  - Runtime dynamic addition/removal of actuators is unsupported by `ros2_control`; cold-reconfiguration (Tier 0 in `MODULARITY_SPECIFICATION.md` §7) remains the necessary architectural model.

**Status: Gate G-D passed. Implementation must strictly align with the chosen distribution API (Jazzy vs Lyrical).**

## G-E — VDA 5050 v3 library capability

- Upstream analysis of `ros-industrial/vda5050_interfaces`:
  - Package implements VDA 5050 specification **v2.0.0**.
  - `vda5050_core` / `vda5050_client` master implementation is experimental.
- VDA 5050 specification is at **v3.0.0** (with non-breaking v3.1.0 in progress).
- Key differences in v3.0: enriched order structure, flexible trajectory definitions, enhanced error and action handling.
- **Architectural Strategy:** Valtoryn maintains its internal, decoupled facility and task model (`valtoryn_interfaces`). For external fleet integration, Valtoryn will implement a lightweight, purpose-built VDA 5050 v3 JSON/MQTT gateway node rather than depending on lagging upstream v2 interface packages.

**Status: Gate G-E marked resolved via thin-build gateway architecture (TD-9).**

## G-F — Mechanical sizing

Formulas for AMR drivetrain sizing (R03 §10):
- Peak tractive force:
  $$F_{\text{tractive}} = F_{\text{accel}} + F_{\text{grade}} + F_{\text{rolling}}$$
- Rolling resistance:
  $$F_{\text{rolling}} = m_{\text{total}} \cdot g \cdot C_{rr}$$
- Wheel torque:
  $$T_{\text{wheel}} = \frac{F_{\text{tractive}} \cdot r}{n_{\text{driven}}}$$

Remaining physical data needed to finalize sizing:
1. Floor rolling resistance coefficient $C_{rr}$ (industrial epoxy/polished concrete typically 0.010–0.015, but unmeasured).
2. Polyurethane wheel continuous derating factor (catalog ratings assume intermittent manual cart usage; continuous 24/7 AMR operation requires 40–50% load derating to prevent heat buildup and delamination).
3. Caster swivel torque and scrub resistance during pivot turns.
4. Motor and gearbox efficiency under thermal equilibrium.
5. Battery discharge curve under peak acceleration at minimum operating temperature (0°C–40°C).

**Status: Pending physical floor survey and supplier engineering confirmations.**

