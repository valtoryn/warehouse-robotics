# Architecture milestone report

> **Date:** 2026-11-07 · **Scope:** audit existing research → architecture synthesis → engineering
> specifications. **Status:** documentation milestone complete; nothing was implemented at report time.
>
> **Implementation Addendum (2026-11-09):** Phase 2 platform implementation has commenced.
> Implemented components include `valtoryn_interfaces`, `valtoryn_diff_drive`, `valtoryn_lifecycle`, `valtoryn_task_bt`,
> configuration tooling (`check_config.py`, `gen_urdf.py`, `gen_launch.py`, `simulate_kinematics.py`, `check_interfaces.py`),
> and 49 unit tests. The sections below record the baseline repository state at the completion of the original
> 2026-11-07 architecture milestone.

## 1. What was inspected

- Full repository: git status/log/remote (clean `main`, in sync with `origin/main`), README, CLAUDE.md,
  CONTRIBUTING.md, SECURITY.md, CITATION.cff, LICENSE, `tools/check_docs.py` (read and executed:
  0 errors, 0 warnings at baseline), all eight research documents R01–R08 read in full,
  `docs/references/SOURCES.md` (486 sources, generated/consistent per checker).
- No source code, tests, CI, or build configuration exists — the repository is documentation-only.

## 2. Research corrections

- **No demonstrable factual errors** found in R01–R08; all discrepancies found were already self-disclosed
  in the documents (Kilted EOL Nov/Dec 2026, Nav2 marathon table inconsistency, REP-2000 lag,
  PDF-extraction pairings, summarised sources).
- Minor coverage gap noted (not edited, to preserve research): TEB omitted from R06 §3.6 controller table.
- Cross-document issues resolved at architecture level rather than by editing research:
  - R06/R07 facility-asset model split → unified facility model (MODULARITY_SPECIFICATION §9).
  - R07→R08 tugger/trailer safety delegation gap → recorded as open item A-6.
  - ROS distro never pinned in research → conditional recommendation Lyrical/Jazzy with gates G-A..G-F.
- Full findings: `docs/RESEARCH_AUDIT.md` (10 blocking uncertainties A-1..A-10).

## 3. Architecture decisions (all *proposed*, not implemented)

ADR-001..012 in `docs/architecture/ARCHITECTURE_DECISION_RECORDS.md`: ROS 2 middleware; Lyrical primary /
Jazzy fallback (conditional on gates); ros2_control HAL; Nav2 behind interface; Gazebo Jetty; Fast DDS default
+ Zenoh spike; VDA 5050 v3 external-only + internal facility model; minimal fleet coordinator + optional
Open-RMF; declarative config + generated manifest (thin layer on ROS); certified safety domain outside ROS 2;
C++20 core; E0–E4 evidence discipline.

## 4. Engineering specifications completed

| Document | Content |
|---|---|
| `architecture/PRODUCT_REQUIREMENTS.md` | FR-M/A/F/S/D, NFR-1..7, constraints C-1..6 |
| `architecture/SYSTEM_ARCHITECTURE.md` | 8 layers + safety domain, interfaces, data ownership, config, lifecycle tiers, backends |
| `architecture/MODULARITY_SPECIFICATION.md` | 3 modularity levels, module taxonomy, manifest, C1–C5 gates, identity, facility model |
| `architecture/HARDWARE_ABSTRACTION.md` | ros2_control boundary, chassis contract, backend conformance, error handling |
| `architecture/TECHNOLOGY_DECISIONS.md` | TD-1..19, rejected/deferred, gates G-A..G-F, version baseline |
| `architecture/ARCHITECTURE_DECISION_RECORDS.md` | ADR-001..012 |
| `engineering/REFERENCE_ROBOT_SPECIFICATION.md` | Diff-drive AMR: architecture, drivetrain, sensors, power, compute, payload, open items RS-1..10 |
| `engineering/HARDWARE_INTERFACE_SPECIFICATION.md` | 9 interface classes, standardisation rules, sim parity contract |
| `engineering/SIMULATION_REQUIREMENTS.md` | SR-1..25, physics/sensor/scenario requirements, F1–F20 adoption, E0–E4 path |
| `engineering/VALIDATION_STRATEGY.md` | Test pyramid, evidence rules, subsystem coverage, hardware campaign, release gates |
| `engineering/DEVELOPMENT_ROADMAP.md` | Phases 0–7, dependency map, acceptance criteria, non-goals |

## 5. Unresolved technical questions (blocking items)

- Distribution pin: Gates A–C (compute BSP, driver availability, spike) unpassed → Lyrical vs Jazzy open.
- Concrete-floor `Crr`, powered-vehicle wheel derating, caster swivel resistance → motor/battery sizing.
- ros2_control Lyrical re-verification of static-hardware claims (A-4).
- ISO 3691-4:2023 normative content (paywalled) → safety case, not software architecture.
- Trailer/towing hazard analysis (R07→R08 delegation gap, A-6).
- VDA 5050 v3 open implementation maturity (`vda5050_core` at v2.0.0).
- Gazebo Jetty ↔ Open-RMF/`gz_ros2_control` compatibility.
- MiR250 PDF-extraction pairings to re-verify; summarised-source facts (R01-S04/S05/S18) to re-fetch.
- All simulation fidelity thresholds (F1–F20) pending hardware data.

## 6. Validation performed

- `python tools/check_docs.py` after every commit: 0 errors, 0 warnings (27 markdown files, 486 sources).
- Internal links and citation IDs validated by the checker; evidence tags applied consistently;
  every proposed capability marked proposed/not-implemented; no safety claims made.
- `--external` URL check: not run this session (486 sources; automated checks are inconclusive-prone);
  recommended before release.

## 7. Files changed

New: `docs/RESEARCH_AUDIT.md`, `docs/WORK_PROGRESS.md`, `docs/architecture/*.md` (6),
`docs/engineering/*.md` (5), this report. No existing files modified except WORK_PROGRESS.md updates.

## 8. Commits and verified pushes

| Commit | Content | Remote verified |
|---|---|---|
| `53d7554` | Research audit + progress file | yes (`git ls-remote`) |
| `171c805` | Six architecture documents | yes |
| `b139d31` | Reference robot specification | yes |
| `5c15f7e` | Hardware interface specification | yes |
| `558c659` | Simulation requirements | yes |
| `c23792d` | Validation strategy | yes |
| `3ae2ff2` | Progress checkpoint | yes |
| `38fa897` | Development roadmap | yes |
| (this report) | Milestone report | pending at write time |

All commits use the owner's configured identity; no AI attribution added; no history rewritten.

## 9. Next implementation milestone

**Phase 1 decision gates** (per `DEVELOPMENT_ROADMAP.md`): compute BSP check (G-A), driver availability
survey (G-B), Lyrical/Jetty spike (G-C), ros2_control Lyrical re-verification (G-D), `vda5050_core` v3
assessment (G-E), floor survey (G-F). Then Phase 2 platform skeleton: workspace, interface packages,
config schema + generator, mock + Gazebo backends.

---

## 10. Phase 3 Closure Addendum — Verified Autonomous Simulation & Mission Execution (2026-11-10)

### 10.1 Milestone Objectives Completed
- Pinned and verified baseline: **ROS 2 Jazzy Jalisco**, **Ubuntu 24.04 LTS**, **Gazebo Harmonic (gz-sim 8)**, and **Nav2**.
- Single-robot autonomous navigation brought to verified, repeatable execution in an authored 20m x 16m warehouse facility (`worlds/warehouse.sdf`).
- Global costmap ghost obstacle accumulation resolved by restricting global costmap to `static_layer` and `inflation_layer`, leaving dynamic obstacle detection to `local_costmap`'s rolling window.
- Fail-closed BehaviorTree mission architecture:
  - `valtoryn_task_executor` executes real navigation-only missions (`warehouse_navigation_demo.xml`) across multiple waypoints.
  - Non-functional hardware stubs (`PayloadPickStub`, `PayloadDropStub`, `DockRobotStub`) fail closed (`BT::NodeStatus::FAILURE`, exit code 3) with explicit logging, preventing false-positive material handling or docking claims.
- Dynamic obstacle ground-truth velocity control and sensor reflection verified via `/model/moving_obstacle/cmd_vel` bridging.
- Scenario verification runner (`tools/run_scenarios.py`) upgraded with:
  - Independent per-scenario metrics parsing from `generated/scenario_metrics.json`.
  - Automated simulation and AMCL pose reset (`reset_robot_pose`) between repetitions.
  - Multi-run verification (`--runs 3`) with run-to-run statistics and zero false-positive completions.
