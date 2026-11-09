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
  - Fixed `tools/gen_launch.py` bug passing URDF path directly to `robot_state_publisher` and `controller_manager` parameters rather than string parameter dictionary `{'robot_description': robot_desc_content}`, and added standard local and global costmaps to Nav2 parameters.
  - Fixed `tools/check_config.py` SemVer parser bug where `0.x.y` caret ranges (e.g. `^0.1.0`) allowed breaking major versions; added compound range parsing (`>=1.0.0 <2.0.0`), unique module instance validation, and Gate C2 `parent_frame` and `mount_type` validation.
- **Phase 2 Hardware Abstraction & Backend Architecture:**
  - Enforced fail-closed safety rule in `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp`: physical backend (`mock_hardware == false`) fails with error in `on_init()`, `on_activate()`, `read()`, and `write()` until genuine SocketCAN motor drivers are implemented. No fake feedback or placeholder success.
  - Added strict joint uniqueness verification and comprehensive state interface validation (ensuring both `position` and `velocity` interfaces exist per joint).
  - Added finite command check (`std::isfinite`) preventing NaN/Inf motor commands; added buffer zeroing on `on_deactivate()`.
  - Resolved backend selection architecture via `target.backend` (`mock`, `gazebo`, `physical`) in `schemas/robot-config.schema.json` and `tools/check_config.py`:
    - `mock`: uses `valtoryn_diff_drive/DiffDriveHardware` with loopback.
    - `gazebo`: uses standard `gz_ros2_control/GazeboSimSystem` and `<gazebo>` system plugin.
    - `physical`: uses `valtoryn_diff_drive/DiffDriveHardware` with `mock_hardware: false` (fails closed).
  - Standardized reference configuration `config/robots/ref-amr-diff-v1.yaml` on `target.ros_distro: "jazzy"` and `target.backend: "mock"`.
- **Phase 2 Kinematics & Lifecycle Implementation:**
  - Enhanced `tools/simulate_kinematics.py`: added input parameter validation, curvature-preserving proportional command scaling ($R = v/\omega$), and slip validation.
  - Implemented `src/valtoryn_lifecycle/`: Subsystem Lifecycle Supervisor supporting 4-tier startup sequencing (Tier 1 Hardware -> Tier 2 Localisation -> Tier 3 Navigation -> Tier 4 Task), cascading failure shutdown, and diagnostic status reporting (`DiagnosticStatusReport`).
  - Updated `tools/run_spike.py`: truthful preflight environment checks and exit codes (missing Linux/ROS environment exits 2, never reports false success).
- **Test Suite & CI Expansion:**
  - Created `tests/test_lifecycle.py` with 8 tests for lifecycle state transitions, tier ordering, fault cascading, and diagnostics.
  - Expanded `tests/test_tools.py` with tests for backend selection, backend incompatibility, and kinematics edge cases (zero command, pure translation, pure rotation, curvature preservation, asymmetric slip).
  - Total test count expanded to **49 unit tests**, all passing.

## Files created or modified

- `README.md` (updated evidence taxonomy and implementation baseline)
- `docs/ARCHITECTURE_MILESTONE_REPORT.md` (updated with implementation addendum)
- `docs/architecture/HARDWARE_ABSTRACTION.md` (documented backend architecture and fail-closed safety)
- `docs/architecture/TECHNOLOGY_DECISIONS.md` (aligned active baseline on Jazzy Jalisco)
- `docs/engineering/DEVELOPMENT_ROADMAP.md` (updated Phase 2 status)
- `docs/engineering/DECISION_GATES.md` (updated with full evidence and options)
- `docs/engineering/DEPENDENCY_MATRIX.md` (updated driver and Gazebo rows)
- `docs/WORK_PROGRESS.md` (this checkpoint)
- `config/robots/ref-amr-diff-v1.yaml` (updated to jazzy distro and mock backend)
- `modules/com.valtoryn.chassis.differential/manifest.yaml` (updated distro to jazzy, ros2_control >= 4.0)
- `schemas/robot-config.schema.json` (added target.backend enum)
- `src/valtoryn_interfaces/srv/GetCapabilities.srv` (fixed syntax)
- `src/valtoryn_interfaces/srv/GetModuleManifest.srv` (fixed syntax)
- `src/valtoryn_interfaces/msg/FacilityEdge.msg` (fixed float64 type)
- `src/valtoryn_interfaces/package.xml` (maintainer aligned)
- `src/valtoryn_task_bt/package.xml` (maintainer aligned)
- `src/valtoryn_diff_drive/include/valtoryn_diff_drive/diff_drive_hardware.hpp` (fail-closed physical safety, finite checks)
- `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp` (fail-closed physical safety, finite checks, state validation)
- `src/valtoryn_diff_drive/valtoryn_diff_drive.xml` (plugin description)
- `src/valtoryn_diff_drive/package.xml` (package definition)
- `src/valtoryn_diff_drive/CMakeLists.txt` (build rules)
- `src/valtoryn_lifecycle/package.xml` (new lifecycle package)
- `src/valtoryn_lifecycle/setup.py` (new)
- `src/valtoryn_lifecycle/setup.cfg` (new)
- `src/valtoryn_lifecycle/resource/valtoryn_lifecycle` (new)
- `src/valtoryn_lifecycle/valtoryn_lifecycle/__init__.py` (new)
- `src/valtoryn_lifecycle/valtoryn_lifecycle/lifecycle_supervisor.py` (new supervisor node & diagnostics)
- `tools/check_config.py` (fixed SemVer bug, added backend compatibility checks)
- `tools/gen_urdf.py` (fixed plugin text, wheel orientation, backend plugin selection)
- `tools/gen_launch.py` (fixed robot_description launch parameters, nav2 costmaps)
- `tools/check_interfaces.py` (static IDL, manifest, and BT XML validator; updated for lifecycle pkg)
- `tools/simulate_kinematics.py` (strict validation, curvature preservation scaling)
- `tools/run_spike.py` (truthful preflight and exit codes)
- `tests/test_tools.py` (expanded unit tests)
- `tests/test_lifecycle.py` (new lifecycle supervisor unit tests)
- `.github/workflows/checks.yml` (interface checks and kinematics simulation)

## Validation results

- `python tools/check_docs.py`: 30 files, 486 sources, 0 errors, 0 warnings.
- `python tools/check_config.py`: OK (1 config, 2 manifests).
- `python tools/check_interfaces.py`: OK (7 msgs, 2 srvs, package manifests and BT XML valid).
- `python tools/simulate_kinematics.py config/robots/ref-amr-diff-v1.yaml`: OK (trajectory closure error: 0.002253 m).
- `python -m unittest discover -s tests -p "test_*.py"`: 49 tests passed in 0.256s.
- `python tools/run_spike.py`: Correctly identifies Windows host missing Linux/ROS 2 prerequisites (exit code 2) and reports Docker container execution instructions without fabricating results.

## Commits and pushes

- Remote commits published:
  - `4380192` docs(engineering): update Phase 1 decision gates with hardware evidence and options
  - `90ffe60` fix(scaffolding): resolve syntax and logic defects across interfaces, generators, and validators
  - `945dd42` feat: add differential drive hardware component, kinematics simulation, and integration tests
- Local commits staged for publishing:
  - `fix(diff_drive): enforce physical backend safety check and interface validation`
  - `feat(config): add backend selection and kinematics limit scaling`
  - `feat(lifecycle): implement subsystem lifecycle supervisor and diagnostics`
  - `test: expand unit test suite to 49 tests for lifecycle, backends, and kinematics`
  - `docs: reconcile documentation drift and update work progress checkpoint`

## Remaining work

- **Owner Decision on Distro Pin:** Confirm choice between Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS, recommended) and Option 2/3.
- **Phase 1 Leftovers:** Physical floor survey (G-F) upon site access; G-C spike execution in Linux/Docker environment.
- **Phase 2 Follow-ups:** Compile validation in Ubuntu 24.04 container via `colcon build`.

## Exact next action

Stage, commit, and push Phase 2 backend architecture, lifecycle supervision, and documentation updates to `origin/main`. Report status to project owner.
