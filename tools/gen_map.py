#!/usr/bin/env python3
"""Generate a 2D occupancy grid map (PGM and YAML) for the reference warehouse world.

Matches the exact geometry authored in worlds/warehouse.sdf.
Resolution: 0.05 m/pixel.
Origin: [-12.0, -10.0, 0.0]
Map dimensions: 24.0m x 20.0m (480 x 400 pixels).

Values:
  254 (0xFE): free space
  0   (0x00): occupied obstacle (walls, racks, static pallets)
  205 (0xCD): unknown space (outside perimeter)
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAP_DIR = ROOT / "maps"


def generate_map(out_dir: Path | None = None) -> tuple[Path, Path]:
    target_dir = out_dir or MAP_DIR
    target_dir.mkdir(parents=True, exist_ok=True)

    resolution = 0.05
    origin_x = -12.0
    origin_y = -10.0
    width_m = 24.0
    height_m = 20.0

    width = int(width_m / resolution)   # 480
    height = int(height_m / resolution) # 400

    # Initialize with unknown outside perimeter
    grid = [[205 for _ in range(width)] for _ in range(height)]

    def to_pixel(x: float, y: float) -> tuple[int, int]:
        px = int((x - origin_x) / resolution)
        py = int((y - origin_y) / resolution)
        return px, py

    # Free interior: -10 to +10, -8 to +8
    px_min, py_min = to_pixel(-10.0, -8.0)
    px_max, py_max = to_pixel(10.0, 8.0)
    for r in range(py_min, py_max):
        for c in range(px_min, px_max):
            grid[r][c] = 254

    def fill_box(x_center: float, y_center: float, sx: float, sy: float, val: int = 0):
        c_min, r_min = to_pixel(x_center - sx / 2.0, y_center - sy / 2.0)
        c_max, r_max = to_pixel(x_center + sx / 2.0, y_center + sy / 2.0)
        for r in range(max(0, r_min), min(height, r_max + 1)):
            for c in range(max(0, c_min), min(width, c_max + 1)):
                grid[r][c] = val

    # Perimeter walls
    fill_box(0.0, 8.0, 20.3, 0.3)    # North
    fill_box(0.0, -8.0, 20.3, 0.3)   # South
    fill_box(10.0, 0.0, 0.3, 16.3)   # East
    fill_box(-10.0, 0.0, 0.3, 16.3)  # West

    # Storage Racks
    fill_box(0.0, 4.2, 9.0, 0.9)     # Rack Row 1
    fill_box(0.0, 1.6, 9.0, 0.9)     # Rack Row 2
    fill_box(0.0, -1.6, 9.0, 0.9)    # Rack Row 3
    fill_box(0.0, -4.2, 9.0, 0.9)    # Rack Row 4

    # Static Obstacles
    fill_box(0.5, 0.0, 0.9, 0.9)     # Pallet stack in central aisle
    fill_box(-2.5, 6.2, 0.8, 0.8)    # Staging crate in north bay

    # Write PGM (P5 binary format, bottom-to-top or top-to-bottom)
    # Standard ROS 2 map_server expects P5 binary with row 0 at the TOP of the image,
    # which corresponds to y_max in world coordinates.
    # In world coordinates, r=height-1 is y_max, r=0 is y_min.
    pgm_path = target_dir / "warehouse_map.pgm"
    with pgm_path.open("wb") as fh:
        header = f"P5\n{width} {height}\n255\n".encode("ascii")
        fh.write(header)
        # Write rows top to bottom (row height-1 down to 0)
        for r in range(height - 1, -1, -1):
            fh.write(bytes(grid[r]))

    # Write YAML metadata
    yaml_path = target_dir / "warehouse_map.yaml"
    yaml_content = f"""image: {pgm_path.name}
mode: trinary
resolution: {resolution:.2f}
origin: [{origin_x:.1f}, {origin_y:.1f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path.write_text(yaml_content, encoding="utf-8")
    print(f"Generated {pgm_path} ({width}x{height}) and {yaml_path}")
    return pgm_path, yaml_path


if __name__ == "__main__":
    generate_map()

