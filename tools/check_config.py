#!/usr/bin/env python3
"""Validate Valtoryn robot configs and module manifests against the JSON Schemas.

Implements gate C1 (schema conformance) plus a C2-lite static check:
  * every module type in a robot config resolves to a manifest under modules/;
  * the config's version range matches the manifest's module_version;
  * mount IDs (occupied_mounts) are claimed at most once per robot;
  * modules marked unvalidated=true are reported (they cannot pass gate C3 on
    motion/safety paths, but are allowed in simulation configs).

Requires: PyYAML and jsonschema (Python dev tools; no runtime dependency on ROS).

Usage:
  python tools/check_config.py                          # validate all configs + manifests
  python tools/check_config.py config/robots/ref-amr-diff-v1.yaml

Exit status is non-zero if any error is found.
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
SCHEMAS = ROOT / "schemas"
CONFIG_SCHEMA = SCHEMAS / "robot-config.schema.json"
MANIFEST_SCHEMA = SCHEMAS / "module-manifest.schema.json"
CONFIG_DIR = ROOT / "config" / "robots"
MODULES_DIR = ROOT / "modules"


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def validate(instance, schema: dict, label: str) -> list[str]:
    validator = Draft202012Validator(schema)
    errors = []
    for err in sorted(validator.iter_errors(instance), key=lambda e: list(e.absolute_path)):
        loc = "/".join(str(p) for p in err.absolute_path) or "<root>"
        errors.append(f"{label}: {loc}: {err.message}")
    return errors


def parse_range(spec: str):
    """Minimal SemVer-range check for ^x.y.z / x.y.z. Returns (gte, lt) or None."""
    spec = spec.strip()
    if spec.startswith("^"):
        parts = spec[1:].split(".")
        if len(parts) < 3:
            return None
        major, minor, patch = (int(p.split("-")[0].split("+")[0]) for p in parts[:3])
        return (f"{major}.{minor}.{patch}", f"{major + 1}.0.0" if major else None)
    parts = spec.split(".")
    if len(parts) >= 3:
        try:
            nums = [int(p.split("-")[0].split("+")[0]) for p in parts[:3]]
            v = ".".join(str(n) for n in nums)
            # bare x.y.z means exactly that version
            lt = f"{nums[0]}.{nums[1]}.{nums[2] + 1}"
            return (v, lt)
        except ValueError:
            return None
    return None


def version_tuple(v: str):
    core = v.split("-")[0].split("+")[0]
    return tuple(int(p) for p in core.split(".")[:3])


def range_satisfied(spec: str, version: str) -> bool:
    rng = parse_range(spec)
    if rng is None:
        return True  # unparseable range: warn only
    gte, lt = rng
    vt = version_tuple(version)
    if gte and vt < version_tuple(gte):
        return False
    if lt and vt >= version_tuple(lt):
        return False
    return True


def manifest_path_for(module_type: str) -> Path:
    # module_type com.valtoryn.chassis.differential -> modules/com.valtoryn.chassis.differential/manifest.yaml
    return MODULES_DIR / module_type / "manifest.yaml"


def check_config(path: Path, config_schema: dict, manifest_schema: dict) -> list[str]:
    errors: list[str] = []
    try:
        config = load_yaml(path)
    except yaml.YAMLError as exc:
        return [f"{path}: YAML parse error: {exc}"]

    errors += validate(config, config_schema, rel(path))
    if errors:
        return errors

    modules = config["modules"]
    seen_mounts: dict[str, str] = {}
    manifests: dict[str, dict] = {}

    for mod in modules:
        mpath = manifest_path_for(mod["type"])
        if not mpath.exists():
            errors.append(f"{path.name}: module '{mod['type']}' has no manifest at {rel(mpath)}")
            continue
        try:
            manifest = load_yaml(mpath)
        except yaml.YAMLError as exc:
            errors.append(f"{mpath}: YAML parse error: {exc}")
            continue
        errors += validate(manifest, manifest_schema, str(rel(mpath)))
        if errors:
            continue
        manifests[mod["type"]] = manifest

        if not range_satisfied(mod["version"], manifest["module_version"]):
            errors.append(
                f"{path.name}: module '{mod['type']}' version range '{mod['version']}' "
                f"does not match manifest version {manifest['module_version']}"
            )

        for mount in (mod.get("mount") or {}).get("occupied_mounts", []):
            if mount in seen_mounts:
                errors.append(
                    f"{path.name}: mount '{mount}' claimed by both "
                    f"'{seen_mounts[mount]}' and '{mod['instance']}' (gate C2)"
                )
            else:
                seen_mounts[mount] = mod["instance"]

        if mod.get("unvalidated") and config["target"]["profile"] in ("hardware", "hil"):
            errors.append(
                f"{path.name}: module '{mod['instance']}' is unvalidated but target profile "
                f"is {config['target']['profile']} — cannot pass gate C3 on motion/safety paths"
            )

    return errors


def check_all_manifests(manifest_schema: dict) -> list[str]:
    errors = []
    for mpath in sorted(MODULES_DIR.glob("*/manifest.yaml")):
        try:
            manifest = load_yaml(mpath)
        except yaml.YAMLError as exc:
            errors.append(f"{mpath}: YAML parse error: {exc}")
            continue
        errors += validate(manifest, manifest_schema, str(rel(mpath)))
    return errors


def main() -> int:
    config_schema = load_yaml(CONFIG_SCHEMA)
    manifest_schema = load_yaml(MANIFEST_SCHEMA)

    configs = [Path(p) for p in sys.argv[1:]] or sorted(CONFIG_DIR.glob("*.yaml"))
    all_errors: list[str] = []
    for cfg in configs:
        all_errors += check_config(cfg, config_schema, manifest_schema)
    all_errors += check_all_manifests(manifest_schema)

    if all_errors:
        print("CONFIG CHECK FAILED")
        for e in all_errors:
            print(f"  ERROR {e}")
        return 1
    print(f"config check OK: {len(configs)} config(s), manifests under {MODULES_DIR.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
