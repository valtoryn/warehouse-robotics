# Work progress — Phase 1 decision gates & Phase 2 implementation

> Persistent checkpoint for multi-session work. Update after each completed milestone.
> Last updated: 2026-11-10.

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
  - Added explicit package namespace prefixing in `src/valtoryn_interfaces/msg/FacilityMap.msg` for intra-package types (`valtoryn_interfaces/FacilityNode[]`, etc.) ensuring unambiguous generation across all rosidl backends.
  - Added C++17/20 standard specifications to CMakeLists.txt across packages (`valtoryn_interfaces`, `valtoryn_diff_drive`, `valtoryn_task_bt`).
  - Fixed `tools/gen_urdf.py` ElementTree bug where `<plugin text="..."/>` was generated as an attribute instead of inner text (`<plugin>valtoryn_diff_drive/DiffDriveHardware</plugin>`).
  - Fixed wheel cylinder orientation in `tools/gen_urdf.py` (added `rpy="1.570796 0 0"` roll so cylinder aligns with rotation axis Y).
  - Fixed `tools/gen_launch.py` bug passing URDF path directly to `robot_state_publisher` and `controller_manager` parameters rather than string parameter dictionary `{'robot_description': robot_desc_content}`, added `enable_odom_tf: true`, and added standard local and global costmaps to Nav2 parameters.
  - Fixed `tools/check_config.py` SemVer parser bug where `0.x.y` caret ranges (e.g. `^0.1.0`) allowed breaking major versions; added compound range parsing (`>=1.0.0 <2.0.0`), unique module instance validation, and Gate C2 `parent_frame` and `mount_type` validation.
- **Phase 2 Hardware Abstraction & Controller Bringup Root-Cause Fix:**
  - **Diagnosed and resolved GitHub Actions run #11 failure:** `tools/gen_urdf.py` generated `<joint>` tags as children of `<hardware>` instead of `<ros2_control>`. `ros2_control`'s parser requires `<joint>` to be a direct child of `<ros2_control>`; as a result, `DiffDriveHardware::on_init()` received 0 joints, failed with `CallbackReturn::ERROR`, and prevented `diff_drive_controller` from activating and publishing `/diff_drive_controller/odom` and `/tf`.
  - Fixed `tools/gen_urdf.py` so `<joint>` elements are created directly under `<ros2_control>`. Added regression test in `tests/test_tools.py` asserting all joints are children of `<ros2_control>` and `<hardware>` contains 0 joints.
  - Enforced fail-closed safety rule in `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp`: physical backend (`mock_hardware == false`) fails with error in `on_init()`, `on_activate()`, `read()`, and `write()` until genuine SocketCAN motor drivers are implemented. No fake feedback or placeholder success.
  - Added strict joint uniqueness verification, comprehensive state interface validation (ensuring both `position` and `velocity` interfaces exist per joint), and finite command checks (`std::isfinite`).
  - Resolved backend selection architecture via `target.backend` (`mock`, `gazebo`, `physical`) in `schemas/robot-config.schema.json` and `tools/check_config.py`.
- **Phase 2 Real Lifecycle Supervision:**
  - Converted `src/valtoryn_lifecycle/` from local-only state machine into a functioning ROS 2 supervisor with decoupled client transport architecture:
    - `RosLifecycleClient`: communicates via ROS 2 `lifecycle_msgs.srv.ChangeState` and `GetState` services with timeout handling, future spinning, and response verification.
    - `MockLifecycleClient`: deterministic mock transport for offline unit testing of state transitions, timeout handling, and failure injection.
    - `RosLifecycleSupervisorNode`: ROS 2 node publishing periodic `diagnostic_msgs.msg.DiagnosticArray` on `/diagnostics`.
    - Reverse-order shutdown (`shutdown_all`, `deactivate_all`) and dependency fault cascading: failure in a critical lower-tier node triggers deactivation of higher-tier dependents via real service calls. Never claims transitions occurred without node confirmation.
- **Phase 2 Motion Proving & Automated Integration Tests:**
  - Created and refined `tests/test_ros_motion.py`: tests active controllers (`ros2 control list_controllers`), claimed hardware interfaces, forward motion ($x$ displacement, wheel speed matching $v/r$), reverse motion, pure rotation ($\Delta\theta$ yaw change), stopping, command timeout enforcement (wheel speeds zero after `cmd_vel_timeout`), and `/tf` broadcast (`odom -> base_link`).
  - Created and expanded `tests/test_ros_lifecycle.py`: tests full lifecycle state transitions, timeout handling, transition rejection, reverse shutdown, fault cascading, and diagnostics publishing against live background ROS lifecycle service nodes over DDS.
- **Integration Spike Runner & Reproducible CI:**
  - Upgraded `tools/run_spike.py` to a 7-stage pipeline with safe log capture to `launch.log`, early process exit/crash detection, and automated log dumping on failures.
  - Updated GitHub Actions workflow (`.github/workflows/checks.yml`) to add a containerized `ros-jazzy-build-and-test` job running in `osrf/ros:jazzy-desktop`, compiling all packages and executing the integration spike.

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
- `src/valtoryn_interfaces/msg/FacilityMap.msg` (explicit package prefixes)
- `src/valtoryn_interfaces/CMakeLists.txt` (C++17 standard)
- `src/valtoryn_diff_drive/CMakeLists.txt` (C++17 standard)
- `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp` (hardware safety, joint validation, logging)
- `src/valtoryn_task_bt/CMakeLists.txt` (C++17 standard)
- `src/valtoryn_lifecycle/package.xml` (ament_python lifecycle package)
- `src/valtoryn_lifecycle/setup.py` (entry points)
- `src/valtoryn_lifecycle/setup.cfg` (install scripts)
- `src/valtoryn_lifecycle/valtoryn_lifecycle/__init__.py` (exports)
- `src/valtoryn_lifecycle/valtoryn_lifecycle/lifecycle_supervisor.py` (real ROS 2 supervisor + diagnostics + transports)
- `tools/gen_urdf.py` (fixed ros2_control joint hierarchy)
- `tools/gen_launch.py` (enable_odom_tf: true, spawner timeouts)
- `tools/run_spike.py` (7-stage build, launch, motion, and integration test runner with crash detection and log capture)
- `tests/test_tools.py` (added test_ros2_control_joints_are_children_of_ros2_control)
- `tests/test_lifecycle.py` (lifecycle supervisor unit tests)
- `tests/test_ros_motion.py` (automated ROS 2 motion integration tests)
- `tests/test_ros_lifecycle.py` (automated ROS 2 lifecycle integration tests)
- `.github/workflows/checks.yml` (containerized ros-jazzy-build-and-test job)

## Validation results

- `python tools/check_docs.py`: 30 files, 486 sources, 0 errors, 0 warnings.
- `python tools/check_config.py`: OK (1 config, 2 manifests).
- `python tools/check_interfaces.py`: OK (7 msgs, 2 srvs, package manifests and BT XML valid).
- `python tools/simulate_kinematics.py config/robots/ref-amr-diff-v1.yaml`: OK (trajectory closure error: 0.002253 m).
- `python -m unittest discover -s tests -p "test_*.py"`: 68 tests discovered (53 passed offline, 15 skipped pending live ROS 2 host runtime).
- `python tools/run_spike.py`: Correctly identifies Windows host missing Linux/ROS 2 prerequisites (exit code 2) and reports Docker container execution instructions without fabricating results.

## Commits and pushes

- Remote commits published:
  - `4380192` docs(engineering): update Phase 1 decision gates with hardware evidence and options
  - `90ffe60` fix(scaffolding): resolve syntax and logic defects across interfaces, generators, and validators
  - `945dd42` feat: add differential drive hardware component, kinematics simulation, and integration tests
  - `6b0b98f` fix(diff_drive): enforce physical backend safety check and interface validation
  - `3e242a0` feat(config): add backend selection, kinematics limit scaling, and truthful spike runner
  - `6560c6d` feat(lifecycle): implement subsystem lifecycle supervisor and diagnostics
  - `dd27aee` test: expand unit test suite to 49 tests for lifecycle, backends, and kinematics
  - `6cfdb0b` docs: reconcile documentation drift and update work progress checkpoint
  - `596a4ca` feat(lifecycle): implement real ROS 2 lifecycle service client and diagnostics transport
  - `ebd648b` feat(bringup): add automated ROS motion tests, launch TF flags, and multi-stage spike runner
  - `c38abd8` ci: add containerized ROS 2 Jazzy build and integration test workflow
  - `f1663fe` docs: update work progress checkpoint with executable mock bringup milestone
  - `b7ab1e3` fix(urdf): place joints directly under ros2_control and improve spike runner diagnostics
  - `fc90ea3` test(ros): expand live lifecycle tests and refine motion test assertions
  - `dea894d` docs: record run 11 root cause fix and bringup resolution in progress checkpoint
  - `d7129b1` fix(bringup): parse tabular controller and interface status in spike runner and integration tests
- New commits staged for publishing:
  - `docs: update work progress checkpoint with controller status parsing fix`

## Remaining work

- **Owner Decision on Distro Pin:** Confirm choice between Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS, recommended) and Option 2/3.
- **Phase 1 Leftovers:** Physical floor survey (G-F) upon site access.
- **Phase 3 Bring-up:** Author reference warehouse SDF world and configure Nav2 bringup launch pipeline for simulated AMR.

## Exact next action

Stage, commit, and push the tabular controller status parsing fix in `tools/run_spike.py` and `tests/test_ros_motion.py`. Monitor CI run.
