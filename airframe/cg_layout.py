"""Recalculate the FT Explorer sport-wing mass, CG, and lumped inertia estimate.

Coordinates follow a conventional aircraft body frame: x is positive aft, y is
positive right, and z is positive up.  The origin is the main-wing leading edge.
All CSV positions are design locations rather than measured installation coordinates.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


TARGET_CG_MM = 57.0
BATTERY_NAME = "Gens Ace 4000 mAh 3S battery"


def read_rows(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for raw in csv.DictReader(stream):
            rows.append(
                {
                    "component": raw["component"],
                    "mass_g": float(raw["mass_g"]),
                    "x_mm": float(raw["x_mm_from_wing_le"]),
                    "y_mm": float(raw["y_mm"]),
                    "z_mm": float(raw["z_mm"]),
                    "uncertainty_g": float(raw["uncertainty_g"]),
                }
            )
    return rows


def solve_battery_x(rows: list[dict[str, float | str]], target_mm: float) -> float:
    battery = next(row for row in rows if row["component"] == BATTERY_NAME)
    other_mass = sum(float(row["mass_g"]) for row in rows if row is not battery)
    other_moment = sum(
        float(row["mass_g"]) * float(row["x_mm"])
        for row in rows
        if row is not battery
    )
    battery_mass = float(battery["mass_g"])
    return (target_mm * (other_mass + battery_mass) - other_moment) / battery_mass


def calculate(rows: list[dict[str, float | str]]) -> dict[str, float]:
    total_g = sum(float(row["mass_g"]) for row in rows)
    cg = {
        axis: sum(float(row["mass_g"]) * float(row[f"{axis}_mm"]) for row in rows)
        / total_g
        for axis in ("x", "y", "z")
    }

    # Point-mass inertias are useful for layout trades, but the 334 g structural
    # assembly is lumped at one point.  These are therefore not flight-dynamics
    # quality inertias yet.
    inertia = {"Ixx": 0.0, "Iyy": 0.0, "Izz": 0.0, "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0}
    for row in rows:
        mass_kg = float(row["mass_g"]) / 1000.0
        x = (float(row["x_mm"]) - cg["x"]) / 1000.0
        y = (float(row["y_mm"]) - cg["y"]) / 1000.0
        z = (float(row["z_mm"]) - cg["z"]) / 1000.0
        inertia["Ixx"] += mass_kg * (y * y + z * z)
        inertia["Iyy"] += mass_kg * (x * x + z * z)
        inertia["Izz"] += mass_kg * (x * x + y * y)
        inertia["Ixy"] -= mass_kg * x * y
        inertia["Ixz"] -= mass_kg * x * z
        inertia["Iyz"] -= mass_kg * y * z

    return {"mass_kg": total_g / 1000.0, **{f"cg_{k}_mm": v for k, v in cg.items()}, **inertia}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).with_name("mass_properties.csv"),
        help="component mass table",
    )
    parser.add_argument("--target-cg-mm", type=float, default=TARGET_CG_MM)
    parser.add_argument(
        "--battery-x-mm",
        type=float,
        help="override battery-center location relative to wing leading edge",
    )
    args = parser.parse_args()

    rows = read_rows(args.csv)
    solved_x = solve_battery_x(rows, args.target_cg_mm)
    battery = next(row for row in rows if row["component"] == BATTERY_NAME)
    battery["x_mm"] = solved_x if args.battery_x_mm is None else args.battery_x_mm
    result = calculate(rows)

    print(f"Total mass: {result['mass_kg']:.4f} kg")
    print(f"Solved battery center: {solved_x:.2f} mm from wing LE (positive aft)")
    print(
        "CG: "
        f"x={result['cg_x_mm']:.2f}, y={result['cg_y_mm']:.2f}, "
        f"z={result['cg_z_mm']:.2f} mm from wing LE"
    )
    print("Lumped point-mass inertia estimate about CG [kg m^2]:")
    print(
        f"  Ixx={result['Ixx']:.6f}  Iyy={result['Iyy']:.6f}  "
        f"Izz={result['Izz']:.6f}"
    )
    print(
        f"  Ixy={result['Ixy']:.6f}  Ixz={result['Ixz']:.6f}  "
        f"Iyz={result['Iyz']:.6f}"
    )
    print("Battery sensitivity: 10 mm pack travel changes CG by "
          f"{float(battery['mass_g']) / (result['mass_kg'] * 1000.0) * 10.0:.2f} mm")


if __name__ == "__main__":
    main()
