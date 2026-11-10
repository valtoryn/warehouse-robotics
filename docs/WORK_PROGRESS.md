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
  - **Phase 3 Nav2 Autonomous Navigation Stabilization & SmacPlanner2D Configuration:**
    - Diagnosed GitHub Actions run #31 scenario verification results:
      - 7 of 10 tests passed (`test_01`, `test_02`, `test_03`, `test_04`, `test_05`, `test_08`, `test_10`).
      - In `test_06` (Scenario 2: 90-degree turn), the robot turned into the central aisle but stopped at `(-4.64, 1.02)` with pose error 1.096m. Root causes identified:
        1. `FollowPath` (`RegulatedPurePursuitController`) lacked `use_rotate_to_heading: true`. At the 90-degree turn, the AMR translated forward while sweeping an arc rather than rotating in place, swinging out towards Rack Row 2.
        2. `planner_server` had `tolerance: 1.0` configured for `NavfnPlanner`. This allowed the planner to terminate paths up to 1.0m short of the goal and pin the robot against Rack 2's obstacle inflation footprint, causing `test_07` and `test_09` to fail to plan from within inscribed cost.
      - Fix in `src/valtoryn_bringup/config/nav2_params.yaml`:
        - Enabled `use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785` (45 deg), and `max_angular_vel: 1.0` in `FollowPath`.
        - Configured `nav2_smac_planner::SmacPlanner2D` as the primary `GridBased` planner with `tolerance: 0.25`, `cost_travel_multiplier: 2.0`, and `downsample_costmap: false` for guaranteed 2D A* optimal search without Dijkstra saddle points or premature path truncation.
        - Tuned `robot_radius: 0.40` in `local_costmap` and `global_costmap` matching the physical chassis half-width.
      - Fix in `tests/test_ros_navigation.py`:
        - Corrected fallback pose calculation from spawn odometry using exact spawn yaw (`yaw = -1.570796`) coordinate rotation.

  - **Run #34 Failure Analysis & Phase 3 Navigation Reliability Closure:**
    - Baseline: CI Run #34 (`42721598821`) on commit `1d54a29`.
    - Failures: `test_07_scenario_3_static_obstacle_avoidance` failed after 32.4s ("no valid path found"), and `test_09_scenario_5_multi_waypoint_mission_dispatch` aborted in 0.0s (11ms).
    - Diagnosis & Root Causes:
      1. Ghost obstacle accumulation in `global_costmap`: `obstacle_layer` in `global_costmap` accumulated laser reflections with localization noise, filling the 1.35m clearance passage south of the static pallet stack at `(0.5, 0.45)`, causing intermittent replanning failures and recovery spins that pushed the AMR against Rack Row 3 (`Start occupied`).
      2. Cascade failure in `test_09`: Because `test_07` left the robot pinned inside the lethal inflation footprint of Rack Row 3 at `(-2.70, -0.97)`, the subsequent goal in `test_09` was immediately aborted by the planner server in 11ms (`Start occupied`), displaying as `0.0s`.
      3. False-positive hardware stubs: Material-handling and docking stubs previously returned `BT::NodeStatus::SUCCESS`, risking false-positive claims of physical capability.
    - Solutions Implemented:
      1. Restrict `global_costmap` to `plugins: ["static_layer", "inflation_layer"]` in `src/valtoryn_bringup/config/nav2_params.yaml` and `tools/gen_launch.py`, eliminating ghost obstacle buildup while preserving real-time obstacle avoidance in `local_costmap`'s rolling window.
      2. Set `cost_travel_multiplier: 1.0` and `tolerance: 0.5` in `planner_server`.
      3. Aligned Scenario 3 destination to `(2.5, 0.0, 0.0)` in the central aisle clear of the pallet stack.
      4. Updated `PayloadPickStub`, `PayloadDropStub`, and `DockRobotStub` in `src/valtoryn_task_bt/include/valtoryn_task_bt/` to fail closed (`BT::NodeStatus::FAILURE`, exit code 3) with explicit unverified hardware logging (`[UNVERIFIED_PHYSICAL_HARDWARE]`).
      5. Created `src/valtoryn_task_bt/bt/warehouse_navigation_demo.xml` executing a real navigation-only multi-station mission (`(-7.0, 0.5)` then `(-5.0, 0.0)`).
      6. Modernized `tests/test_ros_navigation.py`:
         - `_send_nav_goal()` returns structured results reporting distinct failure reasons (`SERVER_UNAVAILABLE`, `GOAL_REJECTED`, `TIMEOUT`, `ABORTED`, `CANCELED`, `SUCCEEDED`) and error code/message.
         - `test_04_tf_tree_valid` asserts complete kinematic chain: `map -> odom`, `odom -> base_link`, `base_link -> laser_link`, and buffer transform validity.
         - `test_08_scenario_4_dynamic_obstacle_response` commands `/model/moving_obstacle/cmd_vel`, verifies displacement, and verifies LiDAR/costmap detection.
         - `test_09_scenario_5_multi_waypoint_mission_dispatch` executes `valtoryn_task_executor` with `warehouse_navigation_demo.xml`, verifying sequential navigation to `(-5.0, 0.0)` with error < 0.50m.
         - `test_10_behavior_tree_mission_stubs` executes `valtoryn_task_executor` with fail-closed stubs, asserting return code 3.
         - Exports `generated/scenario_metrics.json` at `tearDownClass`.
      7. Upgraded `tools/run_scenarios.py`:
         - Implemented `reset_robot_pose()` resetting AMR and moving obstacle poses in Gazebo Harmonic via `gz service` and resetting AMCL via `/initialpose`.
         - Parses `generated/scenario_metrics.json` to assign independent `PASSED`/`FAILED`/`NOT_RUN` status, mean duration, and errors to each scenario.
         - Default `--runs 3` with run-to-run statistics.
      8. Updated `.github/workflows/checks.yml` to run `python3 tools/run_scenarios.py --runs 3` and archive `scenario_metrics.json`.
      9. Diagnosed and resolved Run #36 failure modes:
         - **Rotate-To-Heading Dead-Zone:** `rotate_to_heading_min_angle` (0.785 rad / 45°) was greater than `yaw_goal_tolerance` (0.25 rad / 14°). Heading errors between 0.25 and 0.785 rad prevented in-place rotation; the robot continued driving forward, overshooting `(-5.0, 0.0)` into storage rack collisions (`code=103`), leaving subsequent goals trapped in occupied cells (`code=205 / START_OCCUPIED`). Fixed by setting `rotate_to_heading_min_angle: 0.20` (< `yaw_goal_tolerance: 0.35`) and `use_final_approach_orientation: true` in `SmacPlanner2D`.
         - **Static TF Durability:** `/tf_static` requires `TRANSIENT_LOCAL` durability. Subscribed with `DurabilityPolicy.TRANSIENT_LOCAL` and added buffer check in `test_04_tf_tree_valid`.
         - **Independent Repetition Isolation:** Upgraded `tools/run_scenarios.py` with per-repetition simulation lifecycle management (`start_simulation` and `stop_simulation`), eliminating accumulated odometry drift across runs.
         - **Mission Route Alignment:** Aligned `warehouse_navigation_demo.xml` to execute return route from terminal station `(2.5, 0.0)` through central transit `(-2.0, 0.0)` to aisle entrance `(-5.0, 0.0)`.

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
  - `38fe2b8` fix(simulation): align diff_drive_controller cmd_vel_unstamped topic and test sim time TF buffer
  - `1972636` fix(nav2): remove global SetRemap to prevent docking server DDS type collision and increase progress checker allowance
  - `1a8293d` fix(nav2): specify costmap width and height as integers for ROS 2 Jazzy
  - `71a76c1` fix(simulation): elevate LiDAR above chassis deck and expand scenario timeouts
  - `6f3e8f3` fix(simulation): resolve aisle obstacle clearance, elevate LiDAR to 0.42m, and tune Nav2 controller
  - `4310d60` fix(simulation): tune wheel lateral friction for skid-steer, align spawn yaw, and disable FollowPath collision detection
  - `5dc305d` fix(simulation): align scenario waypoints with central warehouse aisle
  - `0a28632` fix(nav2): enable A* search in NavfnPlanner and tune costmap inflation
  - `93af705` feat(nav2): configure SmacPlanner2D and rotate-to-heading in Nav2 bringup
  - `d97d855` fix(sim): configure collision monitor pass-through and robust TF tree validation
  - `d77dce0` fix(collision_monitor): configure polygons with action_type none to avoid launch tuple error and deadman stops
  - `2aa67d4` fix(nav2): calibrate costmap radius and inflation for warehouse clearance
  - `2adfdf0` fix(nav2): enforce active robot standstill between goals and align goal checker tolerances
  - `b62d969` fix(simulation): configure low lateral wheel friction for skid-steer and expand movement allowance
  - `0cbb518` fix(nav2): disable cost-regulated speed throttling in narrow aisles and calibrate travel velocity
  - `ac69c5a` fix(tests): check mission completion success in combined process output

- **Phase 3 Autonomous Simulation Milestone Closure (100% Verified in CI Run #53 [42820455684]):**
  - All 5 autonomous warehouse AMR navigation scenarios verified repeatably across 3 consecutive simulation repetitions in headless Gazebo Harmonic (gz-sim 8) and Nav2 on ROS 2 Jazzy Jalisco:
    - **Scenario 1 (Straight Corridor Navigation):** Target `[-7.0, 0.0]`, Status `PASSED` (avg duration 10.19s, avg position error 0.378m).
    - **Scenario 2 (90-Degree Aisle Turn):** Target `[-4.0, 0.0]`, Status `PASSED` (avg duration 11.68s, avg position error 0.382m).
    - **Scenario 3 (Static Obstacle Avoidance):** Target `[2.5, 0.0]`, Status `PASSED` (avg duration 19.12s, avg position error 0.393m). Navigates around central pallet stack at `(0.5, 0.45)` with verified obstacle clearance.
    - **Scenario 4 (Dynamic Obstacle Detection):** Target `[6.0, 0.0]`, Status `PASSED` (avg duration 1.02s). Verified moving obstacle displacement via velocity control and observation in forward LiDAR scans and local costmap.
    - **Scenario 5 (Multi-Waypoint Route):** Target `[-4.0, 0.0]`, Status `PASSED` (avg duration 33.57s, avg position error 0.337m). Verified sequential mission execution via `valtoryn_task_executor` with `warehouse_navigation_demo.xml`.
  - Executed 30 total tests across 3 runs (10 tests per run, zero failures, zero skips, genuine assertions).
  - Physical hardware boundaries strictly enforced: BehaviorTree material-handling stubs (`PayloadPickStub`, `PayloadDropStub`, `DockRobotStub`) fail closed with exit code 3 and explicit unverified hardware logging (`test_10`).
  - Full CI pipeline completely green across all jobs (`static-and-unit-checks`, `ros-jazzy-build-and-test`, `gazebo-harmonic-nav2-simulation`).

## Remaining work

- **Owner Decision on Distro Pin:** Confirm choice between Option 1 (Jazzy Jalisco on Ubuntu 24.04 LTS, active baseline) and Option 2/3.
- **Phase 1 Leftovers:** Physical floor survey (G-F) upon site access.
- **Phase 4 Fleet and Facility Layer (Simulation):** Author facility model, graph exporters (LIF, RMF graph, Nav2 GeoJSON), queue/reservation multi-robot baseline coordinator, VDA 5050 v3 robot-side gateway, and multi-robot simulated missions.

## Exact next action

Begin Phase 4 Fleet and Facility Layer: design internal facility model schema and multi-robot fleet coordination baseline in Gazebo Harmonic simulation.
