#!/usr/bin/env python3
"""Validate facility-model documents (schema + structural routing checks).

Checks every facilities/*.yaml against schemas/facility-map.schema.json and then
applies the structural rules in valtoryn_facility.facility_model.FacilityMap.validate():
unique IDs, known edge endpoints, no self-loops, valid zone polygons, assets
attached to existing nodes, no route node inside an active keep-out zone, and
warnings for isolated/unreachable nodes and duplicate lanes.

Usage:
  python tools/check_facility.py
  python tools/check_facility.py facilities/warehouse_ref.yaml
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_facility"))

from valtoryn_facility.facility_model import (  # noqa: E402
    SCHEMA_RELPATH,
    FacilityMap,
    schema_errors,
)


def check_document(path: Path, schema: dict) -> tuple[list[str], list[str]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        return [f"{path.name}: document is not a mapping"], []
    errors = [f"{path.name}: {e}" for e in schema_errors(document, schema)]
    facility = FacilityMap.from_dict(document)
    structural_errors, warnings = facility.validate()
    errors.extend(f"{path.name}: {e}" for e in structural_errors)
    warnings = [f"{path.name}: {w}" for w in warnings]
    return errors, warnings


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    schema = yaml.safe_load((ROOT / SCHEMA_RELPATH).read_text(encoding="utf-8"))
    if args:
        paths = [Path(a) for a in args]
    else:
        paths = sorted((ROOT / "facilities").glob("*.yaml"))

    if not paths:
        print("ERROR: no facility documents found")
        return 1

    errors: list[str] = []
    warnings: list[str] = []
    for path in paths:
        if not path.exists():
            errors.append(f"{path}: does not exist")
            continue
        e, w = check_document(path, schema)
        errors.extend(e)
        warnings.extend(w)

    for warning in warnings:
        print("WARNING:", warning)
    for error in errors:
        print("ERROR:  ", error)
    print(
        f"facility check: {len(paths)} document(s), "
        f"{len(errors)} error(s), {len(warnings)} warning(s)"
    )
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
