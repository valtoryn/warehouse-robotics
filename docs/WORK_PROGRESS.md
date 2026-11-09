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

- **Phase 3 Autonomous Warehouse Simulation Implementation:**
  - **Simulator Pairing Resolution:** Verified and confirmed official `ros_gz` compatibility. ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS natively targets **Gazebo Harmonic (gz-sim 8)**, supported to May 2029. Corrected stale Jetty references across `docs/architecture/TECHNOLOGY_DECISIONS.md`, `docs/engineering/DEPENDENCY_MATRIX.md`, `docs/engineering/SIMULATION_REQUIREMENTS.md`, and `docs/engineering/DEVELOPMENT_ROADMAP.md`.
  - **`ros2_control` 4.x API Modernization:** Resolved `on_init(const HardwareInfo &)` deprecation warning in `src/valtoryn_diff_drive/` by updating to `CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params)` with `<hardware_interface/types/hardware_component_interface_params.hpp>`.
  - **2D Navigation LiDAR Module:** Created `modules/com.valtoryn.sensor.lidar_2d/manifest.yaml` declaring `/scan`, `sensor_deck` mount, `laser_link` and `laser_frame` TF conventions. Added module to `config/robots/ref-amr-diff-v1.yaml` at pose `[0.28, 0.0, 0.22]`.
  - **Physics & Sensor URDF Generation:** Enhanced `tools/gen_urdf.py` to generate sensor links, visuals, collisions, Gazebo Harmonic `gpu_lidar` sensor tags with `ogre2` rendering engine, and wheel contact surface friction parameters (`mu1`, `mu2`, `kp`, `kd`) for accurate physics simulation.
  - **Authored Warehouse Simulation World (`worlds/warehouse.sdf`):** Built a 20m × 16m warehouse environment featuring perimeter walls, concrete floor, 4 storage rack rows creating 3 longitudinal corridors and 2 lateral transit cross-aisles, 4 pick/drop stations, static pallet stacks, dynamic obstacle with velocity control, and Gazebo Harmonic physics and sensor system plugins.
  - **Occupancy Grid Mapping:** Created `tools/gen_map.py` to generate high-resolution warehouse map (`maps/warehouse_map.pgm` and `maps/warehouse_map.yaml` at 0.05m/cell).
  - **Simulation Bringup Package (`valtoryn_bringup`):** Created a new ROS 2 package containing:
    - `sim_bringup.launch.py`: launches Gazebo Harmonic, spawns reference AMR, bridges `/clock` and `/scan`, activates `joint_state_broadcaster` and `diff_drive_controller`.
    - `nav2.launch.py` and `slam.launch.py`: Nav2 bringup with AMCL localization, Regulated Pure Pursuit controller, Smac2D and Navfn planners, local/global costmaps with `/scan` obstacle layers, and collision monitor.
    - `warehouse_sim.launch.py`: composable top-level simulation launch file.
  - **Executable Behavior Tree & Stubs (`valtoryn_task_bt`):** Implemented C++ action client `NavigateToPoseAction` and explicit non-functional material-handling stubs `PayloadPickStub` and `PayloadDropStub` (ensuring unverified hardware claims are never made), `DockRobotStub`, executable node `valtoryn_task_executor`, and route mission XML `warehouse_route_mission.xml`.
  - **Scenario Verification Runner & Tests:**
    - `tests/test_ros_navigation.py`: 10 automated ROS integration tests for sensor validity, clock, costmap obstacle marking, TF tree verification, straight corridor driving, 90-degree turning, static obstacle avoidance, dynamic obstacle detection, multi-waypoint missions, and BehaviorTree mission definitions. Configured with persistent `tf_buffer`, `use_sim_time: True`, and zero-timestamp goal poses for reliable TF lookups in simulation.
    - `tools/run_scenarios.py`: 4-stage automated scenario runner with preflight detection, artifact generation, simulation lifecycle monitoring, strict regex test execution accounting (failing if 0 tests run or tests are skipped), active lifecycle check on `/bt_navigator`, and JSON report generation (`generated/scenario_report.json`).
  - **CI Workflow Expansion & Root-Cause Bringup Stabilization:**
    - Diagnosed and resolved GitHub Actions runs #20, #21, #22, and #23:
      - Fixed launch tuple error in ROS 2 Jazzy by providing non-empty `dock_plugins: ["simple_charging_dock"]` and `docks: ["station_dock"]` in `nav2_params.yaml`.
      - Resolved Jazzy collision monitor parameter type change by encoding polygon points as string parameter `"[[x, y], ...]"`.
      - Resolved velocity command disconnect: `diff_drive_controller` in `controllers.yaml` set to `use_stamped_vel: false` which listens on `/diff_drive_controller/cmd_vel_unstamped` for `geometry_msgs/msg/Twist`. Aligned `cmd_vel_out_topic` across `collision_monitor_params.yaml`, `nav2_params.yaml`, and `nav2.launch.py` to route velocity commands directly to `/diff_drive_controller/cmd_vel_unstamped`.
    - Added dedicated containerized `gazebo-harmonic-nav2-simulation` job in `.github/workflows/checks.yml` running headless Gazebo Harmonic + Nav2 simulations and uploading scenario metrics artifacts.

## Files created or modified

- `README.md` (updated evidence taxonomy and Phase 3 simulation baseline)
- `docs/ARCHITECTURE_MILESTONE_REPORT.md` (updated with implementation addendum)
- `docs/architecture/HARDWARE_ABSTRACTION.md` (documented backend architecture and fail-closed safety)
- `docs/architecture/TECHNOLOGY_DECISIONS.md` (aligned active baseline on Jazzy Jalisco and Gazebo Harmonic)
- `docs/engineering/DEVELOPMENT_ROADMAP.md` (updated Phase 3 deliverables and completed status)
- `docs/engineering/DECISION_GATES.md` (updated with full evidence and options)
- `docs/engineering/DEPENDENCY_MATRIX.md` (updated driver, Gazebo Harmonic, and Nav2 rows)
- `docs/engineering/SIMULATION_REQUIREMENTS.md` (updated Gazebo Harmonic pairing and simulation targets)
- `docs/WORK_PROGRESS.md` (this checkpoint)
- `config/robots/ref-amr-diff-v1.yaml` (added 2D LiDAR module and pose)
- `modules/com.valtoryn.sensor.lidar_2d/manifest.yaml` (declared 2D navigation LiDAR module)
- `worlds/warehouse.sdf` (authored warehouse world for Gazebo Harmonic)
- `maps/warehouse_map.pgm` & `maps/warehouse_map.yaml` (generated warehouse occupancy grid map)
- `tools/gen_map.py` (tool for generating warehouse occupancy grid map)
- `tools/gen_urdf.py` (added sensor generation, Gazebo gpu_lidar, and wheel friction physics)
- `tools/run_scenarios.py` (automated scenario verification runner)
- `src/valtoryn_diff_drive/include/valtoryn_diff_drive/diff_drive_hardware.hpp` (on_init Jazzy API fix)
- `src/valtoryn_diff_drive/src/diff_drive_hardware.cpp` (on_init Jazzy API fix)
- `src/valtoryn_bringup/` (new ROS 2 package: launch, configs, maps, worlds)
- `src/valtoryn_task_bt/include/valtoryn_task_bt/` (nav2_client_node, payload_stubs, dock_robot_stub headers)
- `src/valtoryn_task_bt/src/` (nav2_client_node, task_executor_node implementation)
- `src/valtoryn_task_bt/bt/ref_amr_mission.xml` (updated with explicit stubs)
- `src/valtoryn_task_bt/bt/warehouse_route_mission.xml` (multi-station warehouse route mission)
- `src/valtoryn_task_bt/CMakeLists.txt` & `package.xml` (built library and executable with behaviortree_cpp and nav2_msgs)
- `tests/test_ros_navigation.py` (automated ROS 2 / Nav2 navigation tests)
- `tests/test_tools.py` (unit tests for sensor URDF generation and Gazebo friction)
- `.github/workflows/checks.yml` (added gazebo-harmonic-nav2-simulation job)

- `tests/test_lifecycle.py` (lifecycle supervisor unit tests)
- `tests/test_ros_motion.py` (automated ROS 2 motion integration tests)
- `tests/test_ros_lifecycle.py` (automated ROS 2 lifecycle integration tests)
- `tests/test_ros_navigation.py` (automated ROS 2 / Nav2 navigation tests)
- `tests/test_tools.py` (unit tests for sensor URDF generation and Gazebo friction)
- `.github/workflows/checks.yml` (added gazebo-harmonic-nav2-simulation job)

- **Phase 3 Simulation Bringup & Verification Root-Cause Fixes:**
  - **Diagnosed and resolved GitHub Actions run #20 launch failure:**
    - Root cause: In `src/valtoryn_bringup/config/nav2_params.yaml`, line 280 contained `dock_plugins: []`. In ROS 2 Jazzy `launch_ros/descriptions/parameter.py`, an empty list in YAML is converted to an empty tuple `()`, which fails primitive scalar type validation (`TypeError: Expected 'value' to be one of [<class 'float'>, <class 'int'>, <class 'str'>, <class 'bool'>, <class 'bytes'>], but got '()' of type '<class 'tuple'>'`).
    - Fix: Removed unused `docking_server` block with `dock_plugins: []` from `nav2_params.yaml`.
  - **Diagnosed and eliminated false-positive scenario verification:**
    - Root cause: In `tests/test_ros_navigation.py`, `setUpClass` called `raise unittest.SkipTest` when `/navigate_to_pose` timed out. `unittest` exited with return code 0 (`Ran 0 tests`). In `tools/run_scenarios.py`, `passed_tests = (test_proc.returncode == 0)` evaluated to `True`, recording all scenarios as `PASSED` in `scenario_report.json` despite zero tests having run.
    - Fix in `test_ros_navigation.py`: When ROS 2 is active, a missing action server is treated as a hard failure (`RuntimeError`), never a skip. Added true Nav2 action status verification (`GoalStatus.STATUS_SUCCEEDED`), pose lookup in `map` coordinate frame via TF (`map -> base_link`) and `/amcl_pose`, and implemented all 5 navigation scenarios.
    - Fix in `tools/run_scenarios.py`:
      - Early termination with `EXIT_SIM_CRASH` (code 3) if simulation process exits prematurely or action server times out (never proceeds to tests).
      - Strict output parsing ensuring `tests_run > 0`, zero failures, and zero skips. Any zero-test or skipped run results in `EXIT_SCENARIO_FAILED` (code 4).
      - Added support for `--runs N` with duration mean and standard deviation reporting across repetitions.
  - **Simulation Bringup Launch Parameter Propagation:**
    - Declared and propagated arguments `urdf_path`, `world`, `x_pose`, `y_pose`, `z_pose`, and `yaw` in `src/valtoryn_bringup/launch/warehouse_sim.launch.py` to `sim_bringup.launch.py`.
  - **Warehouse Map & Target Coordinates Alignment:**
    - Aligned scenario targets with clear warehouse corridors: Straight Corridor Navigation along west transit aisle (`[-7.0, 1.0, -1.57]`), 90-Degree Aisle Turn (`[-5.0, 2.9, 0.0]`), Static Obstacle Avoidance around central pallet stack (`[2.5, 0.0, 0.0]`), Dynamic Obstacle Detection in forward corridor, and Multi-Waypoint Mission Dispatch across stations.

## Validation results

- `python tools/check_docs.py`: 30 files, 486 sources, 0 errors, 0 warnings.
- `python tools/check_config.py`: OK (1 config, 3 manifests: chassis, compute, lidar_2d).
- `python tools/check_interfaces.py`: OK (7 msgs, 2 srvs, 5 package manifests and 2 BT XML valid).
- `python tools/simulate_kinematics.py config/robots/ref-amr-diff-v1.yaml`: OK (trajectory closure error: 0.002253 m).
- `python -m unittest discover -s tests -p "test_*.py"`: 80 tests discovered (55 passed offline, 25 skipped pending live ROS 2 host runtime).
- `python tools/run_scenarios.py --preflight`: Correctly identifies host prerequisites and outputs Docker command for headless Gazebo Harmonic / Nav2 simulation.

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
  - `5486821` docs: update work progress checkpoint with controller status parsing fix
  - `b6be07f` docs: record successful CI run 42677245021 and Gate G-C verification in progress checkpoint
  - `d4b8646` fix(diff_drive): modernize on_init to HardwareComponentInterfaceParams API for ROS 2 Jazzy
  - `507721f` feat(sensor): declare 2D navigation LiDAR module and update URDF generator for Gazebo sensors and friction
  - `0e3bf7a` feat(simulation): author warehouse SDF world and occupancy grid map generator
  - `3c65fe0` feat(bringup): add valtoryn_bringup package with Gazebo Harmonic, Nav2, and SLAM launch pipelines
  - `afbc9ee` feat(bt): implement executable Nav2 action client and explicit material-handling stubs
  - `beac89f` test(simulation): add automated ROS navigation tests, scenario runner, and Gazebo CI job
  - `01068a5` docs: align simulator baseline to Gazebo Harmonic, update roadmap and progress checkpoint
  - `cf222a2` fix(test): isolate mock spike test suite from Nav2 simulation and handle missing simulation server
  - `29b4886` fix(launch): use capitalized boolean string literals for ROS 2 launch condition evaluation
  - `84df2fc` fix(nav2): use double-colon plugin class names for Nav2 planner and behavior plugins
  - `5b7124d` fix(nav2): add collision monitor and velocity smoother parameters and remap cmd_vel
  - `33e3da2` fix(simulation): resolve launch tuple error and eliminate false-positive scenario reports
  - `12d20f2` fix(nav2): format collision monitor polygon points as string for ROS 2 Jazzy
  - `d1fcd2a` fix(nav2): configure docking server plugins and verify bt_navigator active state

## Remaining work

- **Owner Decision on Distro Pin:** Confirm choice between Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS, active baseline) and Option 2/3.
- **Phase 1 Leftovers:** Physical floor survey (G-F) upon site access.
- **Phase 4 Fleet Coordination:** Author facility model, graph exporters, and multi-robot coordination baseline in simulation.

## Exact next action

Verify Phase 3 CI workflow run on GitHub Actions (`checks-and-ros-build`), reviewing `ros-jazzy-build-and-test` and `gazebo-harmonic-nav2-simulation` jobs.
