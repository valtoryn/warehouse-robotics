# Work progress — architecture + Phase 1 milestone

> Persistent checkpoint for multi-session work. Update after each completed milestone.
> Last updated: 2026-11-07.

## Completed work

- Repository and remote inspected: `main` clean, in sync with `origin/main`. Docs-only repo (no source code,
  tests, CI, or config beyond `.gitattributes`/`.gitignore`).
- `python tools/check_docs.py`: 0 errors, 0 warnings at baseline (14 files, 486 sources).
- All eight research documents audited (R01–R05 read directly; R06–R08 via delegated full read with spot
  verification). Findings in `docs/RESEARCH_AUDIT.md`.
- **Milestone A complete:** `docs/RESEARCH_AUDIT.md` — per-document findings classified
  Confirmed / Correction required / Incomplete evidence / Conflicting evidence / Unresolved / Recommendation.
  No demonstrable factual errors found requiring research edits; discrepancies were already self-disclosed.
- **Milestone B complete (documentation):** six architecture documents written under `docs/architecture/`:
  - `PRODUCT_REQUIREMENTS.md` — FR-M/A/F/S/D, NFR-1..7, constraints C-1..6, success criteria.
  - `SYSTEM_ARCHITECTURE.md` — 8 layers + safety domain, interfaces, data ownership, config mgmt,
    lifecycle/tiers, capability declarations, backend boundaries, error handling, versioning.
  - `MODULARITY_SPECIFICATION.md` — 3 modularity levels, module taxonomy, config schema, manifest fields,
    C1–C5 gates, identity model, reconfiguration tiers, chassis interface, facility model, versioning.
  - `HARDWARE_ABSTRACTION.md` — ros2_control boundary, chassis kinematics contract, actuator/sensor/power/
    compute boundaries, backend conformance contract, error handling, open items HA-1..6.
  - `TECHNOLOGY_DECISIONS.md` — TD-1..19 proposed decisions, rejected/deferred list, gates G-A..G-F,
    version baseline.
  - `ARCHITECTURE_DECISION_RECORDS.md` — ADR-001..012 with requirements/alternatives/evidence/trade-offs/
    recommendation/risks/unresolved each.

## Files created or modified

- `docs/RESEARCH_AUDIT.md`
- `docs/WORK_PROGRESS.md`
- `docs/architecture/PRODUCT_REQUIREMENTS.md`
- `docs/architecture/SYSTEM_ARCHITECTURE.md`
- `docs/architecture/MODULARITY_SPECIFICATION.md`
- `docs/architecture/HARDWARE_ABSTRACTION.md`
- `docs/architecture/TECHNOLOGY_DECISIONS.md`
- `docs/architecture/ARCHITECTURE_DECISION_RECORDS.md`

## Validation results

- `python tools/check_docs.py`: 0 errors, 0 warnings (16 markdown files, 486 sources) after audit+progress.
- Architecture docs use the same evidence tags; all proposed decisions marked *proposed*, not implemented.

## Commits and pushes

- `53d7554` docs: audit R01-R08 research and record findings — pushed, verified on remote (`git ls-remote`).
- `171c805` docs: add system architecture and modularity specification — pushed, verified on remote.
- (engineering commit pending at time of writing)

## Next milestone — Phase 1 decision gates + Phase 2 skeleton start

- `docs/engineering/DECISION_GATES.md` — gate evidence record created.
  - **G-A not passed:** Jetson Orin Nano ships JetPack 7.2 on Ubuntu 24.04; Lyrical needs Ubuntu 26.04.
    Evidence favours Jazzy for first hardware target or a compute re-selection; pin still open (owner decision).
  - **G-B partial:** `rplidar_ros` released for Lyrical (2.1.0-5, stale upstream caveat);
    `sicktoolbox` unmaintained since 2019 → safety-scanner ROS telemetry is an open item
    (protective function lives in safety domain, so non-blocking). Cameras/motor drives per-part pending.
  - **G-C pending:** one-week Lyrical+Jetty spike needs a Linux/ROS host; blocked on G-A.
  - **G-D passed with caveats:** Lyrical ros2_control docs live; Kilted→Lyrical breaking changes
    recorded (export APIs removed, HardwareInfo→HardwareComponentParams). Static hardware model
    confirmed unchanged — cold-reconfiguration default stands.
  - **G-E not passed:** `vda5050_interfaces` implements spec 2.0.0 only; master API experimental.
    Thin-build/contingency plan confirmed for Phase 4.
  - **G-F pending:** needs physical floor survey + supplier data.
- Phase 2 partial scaffold (ROS-independent parts only):
  - `schemas/robot-config.schema.json`, `schemas/module-manifest.schema.json` (JSON Schema 2020-12).
  - `config/robots/ref-amr-diff-v1.yaml` — reference robot config (simulation profile, placeholder dims).
  - `modules/com.valtoryn.chassis.differential/manifest.yaml`,
    `modules/com.valtoryn.compute.jetson_orin/manifest.yaml` — example manifests.
  - `tools/check_config.py` — schema + C2-lite validator (needs PyYAML + jsonschema).
- README status block updated to reflect scaffolding (no robot software/sim/hardware yet).

## Remaining work

- **Phase 2 proper (blocked on G-A/G-C):** colcon workspace, `valtoryn_interfaces` ROS packages,
  config generator emitting URDF/launch/params, mock + Gazebo backends, lifecycle supervisor,
  CI. Do NOT start ROS packages until the distro pin decision (Jazzy vs Lyrical) is made —
  ros2_control APIs differ between them (see DECISION_GATES G-D).
- Phase 1 leftovers: G-C spike execution, G-F floor survey, camera/motor driver per-part checks,
  owner decision on distro pin.
- `python tools/check_docs.py --external` where practical (486 URLs; inconclusive-prone).

## Exact next action

Owner decision needed on the distro pin (see DECISION_GATES.md "Consequence" section). Once pinned:
create the colcon workspace and `valtoryn_interfaces` package skeleton per SYSTEM_ARCHITECTURE layer 3.
