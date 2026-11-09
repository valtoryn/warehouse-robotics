# Valtoryn Warehouse Robotics

An open-source effort to build a **fully modular autonomous warehouse robotics platform**: one software platform
that can run on different robot chassis, locomotion types, sensor sets and payload modules, from simulation through
hardware-in-the-loop to physical deployment.

> **Project status: research, architecture, declarative tooling, Phase 2 platform packages, and Phase 3 simulation.**
> The repository contains research, architecture, and engineering specifications, declarative configuration tooling
> (`schemas/`, `config/`, `modules/`, `tools/`), platform packages (`valtoryn_interfaces`, `valtoryn_diff_drive`,
> `valtoryn_lifecycle`, `valtoryn_task_bt`, `valtoryn_bringup`), an authored warehouse digital twin (`worlds/warehouse.sdf`),
> and Nav2 navigation and SLAM configurations.
> Platform software components are validated by unit test suites offline and by automated ROS 2 Jazzy and
> Gazebo Harmonic integration suites in CI.
> No capability has been physically validated on hardware. Nothing here is safety-rated or certified for use around people.

## Vision

The long-term direction (a statement of intent, not of current capability):

- Multiple robot chassis and locomotion configurations (not only differential drive).
- Interchangeable hardware and sensor modules; modular payload and actuator systems.
- Hardware abstraction and standardised, versioned interfaces.
- Autonomous perception, localisation, mapping, navigation and control.
- Physics-based simulation and digital twins as a validation environment (simulation is not the product).
- Warehouse task execution, material handling and multi-robot fleet coordination.
- Hardware-in-the-loop testing and physical deployment.

The first planned reference implementation is a modular **differential-drive AMR**. Software modularity comes first;
mechanical modularity is to be introduced progressively.

## Evidence levels used throughout this repository

Documents distinguish six states and never describe a capability at a higher level than the evidence supports:

| Level | Meaning |
|---|---|
| Confirmed requirement | Stated by the project owner or derived from a cited external requirement |
| Proposed architecture | A design recommendation in the documentation |
| Implemented | Code exists in this repository and is covered by automated unit/static tests |
| Compiled | Code has been built into binary artifacts under the target ROS toolchain |
| Simulated | Behaviour demonstrated under a physics-based simulator (e.g. Gazebo Sim) |
| Physically validated | Behaviour demonstrated on real physical hardware, with recorded evidence |

Currently, platform configuration tooling, kinematics models, interface packages, hardware abstraction components, lifecycle supervisor logic, Gazebo Harmonic simulation, and Nav2 navigation packages are at the *Implemented* level locally and verified under *Compiled* and *Simulated* (E0/E1) levels in the continuous integration environment (ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS with Gazebo Harmonic). No capability is at the *Physically validated* level.

## Repository guide

| Document | Purpose |
|---|---|
| [CLAUDE.md](CLAUDE.md) | Permanent project rules for AI-assisted sessions (attribution, integrity, publishing) |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute, and the evidence standards for documentation |
| [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) | Community standards (Contributor Covenant 2.1) |
| [SECURITY.md](SECURITY.md) | Security and safety-disclaimer policy |
| [CITATION.cff](CITATION.cff) | How to cite this project |
| [LICENSE](LICENSE) | Apache License 2.0 |
| [tools/check_docs.py](tools/check_docs.py) | Documentation checker: internal links, citation integrity, URL reachability |

Research, architecture and engineering documents are added under `docs/` as each unit is completed and verified.

## License

Apache License 2.0 (see [LICENSE](LICENSE)), confirmed by the repository owner. It matches the licence used by the
core ROS 2 ecosystem and includes an explicit patent grant. Third-party dependencies carry their own licences (for
example, SLAM Toolbox is LGPL-2.1); a dependency-licence audit is still to be done before any release.
