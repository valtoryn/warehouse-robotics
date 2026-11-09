# Work progress — Phase 1 decision gates & Phase 2 implementation

> Persistent checkpoint for multi-session work. Update after each completed milestone.
> Last updated: 2026-11-09.

## Completed work

- **Phase 1 Decision Gates Investigated & Updated (`docs/engineering/DECISION_GATES.md`):**
  - **G-A (Compute BSP vs OS):** Jetson Orin Nano ships JetPack 7.2 on Ubuntu 24.04 LTS (Noble); Lyrical targets Ubuntu 26.04 (Resolute). Investigated three owner options (standardize on Jazzy LTS, switch compute to industrial x86/Pi5, or containerized Lyrical on 24.04 host). Evidence and trade-offs recorded: strongly recommends **ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS** (supported to May 2029) to match JetPack 7.2 natively with full GPU/DLA acceleration.
  - **G-B (Driver Availability):** Verified active drivers on ROS 2 Jazzy for navigation LiDAR (`sllidar_ros2`/`rplidar_ros`), safety scanner telemetry (`sick_safetyscanners2`), depth cameras (`realsense2_camera`, `orbbec_camera`, `depthai-ros`), MCU serial/CAN IMU bridge, and SocketCAN motor drives. Corrected outdated reference from `sicktoolbox` to `sick_safetyscanners2`.
  - **G-C (Integration Spike):** Environment prerequisites defined. Blocker honestly documented: local Windows 11 host lacks native Linux/ROS 2/Gazebo runtime. Reproducible spike runner (`tools/run_spike.py`) created for automated container and CI execution.
  - **G-D (ros2_control):** Verified API differences between Jazzy and Lyrical; confirmed static hardware topology at launch time holds across both distros (Tier 0 cold reconfiguration confirmed).
  - **G-E (VDA 5050 v3):** Confirmed `vda5050_interfaces` implements spec 2.0.0 only; confirmed Valtoryn thin VDA 5050 v3 JSON/MQTT gateway strategy (TD-9).
  - **G-F (Mechanical Sizing):** Sizing calculations documented; blocks final torque/battery selection pending physical site floor survey and continuous AMR wheel derating supplier data.
- **Phase 2 Scaffolding Audit and Demonstrable Defect Fixes:**
  - Fixed syntax error in `src/valtoryn_interfaces/srv/GetCapabilities.srv` and `GetModuleManifest.srv` (removed erroneous leading `---` separator that rendered `.srv` files invalid).
  - Fixed IDL type error in `src/valtoryn_interfaces/msg/FacilityEdge.msg` (replaced invalid `double` with `float64`).
  - Fixed `tools/gen_urdf.py` ElementTree bug where `<plugin text="..."/>` was generated as an attribute instead of inner text (`<plugin>valtoryn_diff_drive/DiffDriveHardware</plugin>`).
  - Fixed wheel cylinder orientation in `tools/gen_urdf.py` (added `rpy="1.570796 0 0"` roll so cylinder aligns with rotation axis Y).
  - Added Gazebo `gz_ros2_control` simulation plugin generation in `tools/gen_urdf.py` when profile is `simulation`.
  - Fixed `tools/gen_launch.py` bug passing URDF path directly to `robot_state_publisher` and `controller_manager` parameters rather than string parameter dictionary `{'robot_description': robot_desc_content}`, and added standard local and global costmaps to Nav2 parameters.
  - Fixed `tools/check_config.py` SemVer parser bug where `0.x.y` caret ranges (e.g. `^0.1.0`) allowed breaking major versions; added compound range parsing (`>=1.0.0 <2.0.0`), unique module instance validation, and Gate C2 `parent_frame` and `mount_type` validation.
- **Phase 2 Implementation:**
  - Created `src/valtoryn_diff_drive/`: ROS 2 `ros2_control` hardware abstraction package with `DiffDriveHardware` system interface (supporting mock simulation integration and physical CAN motor driver backends).
  - Created `tools/check_interfaces.py`: static ROS IDL, package manifest, and BT XML validator.
  - Created `tools/simulate_kinematics.py`: differential drive forward/inverse kinematics simulator with closed-loop trajectory verification.
  - Created `tools/run_spike.py`: automated environment verification and integration spike runner for Gate G-C.
  - Expanded unit test suite (`tests/test_tools.py`) from 16 to 27 unit tests covering SemVer fixes, C2 validation, URDF/launch generation, interface validation, and kinematics simulation.
  - Updated CI workflow (`.github/workflows/checks.yml`) to include interface checks and differential kinematics simulation.

## Files created or modified

- `docs/engineering/DECISION_GATES.md` (updated with full evidence and options)
- `docs/engineering/DEPENDENCY_MATRIX.md` (updated driver and Gazebo rows)
- `docs/WORK_PROGRESS.md` (this checkpoint)
- `src/valtoryn_interfaces/srv/GetCapabilities.srv` (fixed syntax)
- `src/valtoryn_interfaces/srv/GetModuleManifest.srv` (fixed syntax)
- `src/valtoryn_interfaces/msg/FacilityEdge.msg` (fixed float64 type)
- `src/valtoryn_interfaces/package.xml` (maintainer aligned)
- `src/valtoryn_task_bt/package.xml` (maintainer aligned)
- `src/valtoryn_diff_drive/include/valtoryn_diff_drive/diff_drive_hardware.hpp` (new)
- `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp` (new)
- `src/valtoryn_diff_drive/valtoryn_diff_drive.xml` (new)
- `src/valtoryn_diff_drive/package.xml` (new)
- `src/valtoryn_diff_drive/CMakeLists.txt` (new)
- `tools/check_config.py` (fixed SemVer bug, added C2 checks)
- `tools/gen_urdf.py` (fixed plugin text, wheel orientation, gazebo tags)
- `tools/gen_launch.py` (fixed robot_description launch parameters, nav2 costmaps)
- `tools/check_interfaces.py` (new static IDL and manifest validator)
- `tools/simulate_kinematics.py` (new kinematics simulation verification)
- `tools/run_spike.py` (new G-C spike runner)
- `tests/test_tools.py` (expanded to 27 unit tests)
- `.github/workflows/checks.yml` (added new validation steps)

## Validation results

- `python tools/check_docs.py`: 30 files, 486 sources, 0 errors, 0 warnings.
- `python tools/check_config.py`: OK (1 config, 2 manifests).
- `python tools/check_interfaces.py`: OK (7 msgs, 2 srvs, package manifests and BT XML valid).
- `python tools/simulate_kinematics.py config/robots/ref-amr-diff-v1.yaml`: OK (trajectory closure error: 0.002253 m).
- `python -m unittest discover -s tests -v`: 27 tests passed in 0.153s.
- `python tools/run_spike.py`: Correctly identifies Windows host missing Linux/ROS 2 prerequisites and reports Docker container execution instructions without fabricating results.

## Commits and pushes

- Previous: `8ad8bae` docs: update work progress checkpoint after Phase 2 scaffolding
- (New commits pending verification and publishing)

## Remaining work

- **Owner Decision on Distro Pin:** Confirm choice between Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS, recommended) and Option 2/3.
- **Phase 1 Leftovers:** Physical floor survey (G-F) upon site access; G-C spike execution in Linux/Docker environment.
- **Phase 2 Follow-ups:** Lifecycle supervisor node and diagnostics plumbing.

## Exact next action

Stage, commit, and push Phase 1 gate updates, defect fixes, and Phase 2 hardware abstraction components. Report status to project owner.
