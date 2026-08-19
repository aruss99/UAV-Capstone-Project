"""Readers for the VSPAero output files, and the axis convention conversion.

Two things live here because they are the places a silent error would poison
everything downstream:

1.  The ``.polar`` and ``.lod`` parsers.
2.  ``to_aircraft_axes``, which converts VSPAero's OpenVSP-axis output into the
    standard aircraft body axes the Simulink AirFrame expects.

AXIS CONVENTION -- established by measurement, not assumed.  OpenVSP model axes
are x aft, y right, z up.  The polar's CFxtot/CFztot are body-axis forces in
those axes and CLtot/CDtot are wind-axis; that identification is confirmed
numerically by ``check_axis_convention``, which reproduces the file's own CLtot
and CDtot from CFxtot/CFztot through the alpha rotation.

Standard aircraft body axes are x forward, y right, z down, i.e. a 180 degree
rotation about y.  A vector (x, y, z) therefore maps to (-x, y, -z), giving

    CX = -CFxtot     CY = +CFytot     CZ = -CFztot
    Cl = -CMxtot     Cm = +CMytot     Cn = -CMztot

The same relation is visible in the VSPAero polars under DroneModelv6, whose
CMl/CMm/CMn columns equal -CMx/+CMy/-CMz exactly.
"""

from __future__ import annotations

import math
from pathlib import Path


class PolarPoint(dict):
    """One solved flight point.  Keys are the polar column names."""

    @property
    def alpha(self) -> float:
        return self["AoA"]

    @property
    def beta(self) -> float:
        return self["Beta"]


def read_polar(path: Path) -> list[PolarPoint]:
    """Parse a VSPAero .polar file into a list of solved points."""
    lines = [l for l in Path(path).read_text().splitlines() if l.strip()]
    header = None
    points: list[PolarPoint] = []
    for line in lines:
        fields = line.split()
        if "Beta" in fields and "AoA" in fields and "CLtot" in fields:
            header = fields
            continue
        if header is None or len(fields) != len(header):
            continue
        try:
            values = [float(f) for f in fields]
        except ValueError:
            continue
        points.append(PolarPoint(zip(header, values)))
    if not points:
        raise RuntimeError(f"no solved points parsed from {path}")
    return points


def check_axis_convention(points: list[PolarPoint], tol: float = 2e-4) -> float:
    """Re-derive CLtot/CDtot/CStot from the body-axis forces; return worst error.

    In OpenVSP axes (x aft, z up), with incidence a and sideslip b:

        CL = CFz*cos(a) - CFx*sin(a)
        CD = CFx*cos(a)*cos(b) - CFy*sin(b) + CFz*sin(a)*cos(b)
        CS = CFx*cos(a)*sin(b) + CFy*cos(b) + CFz*sin(a)*sin(b)

    These were fitted against the solver's own wind-axis columns over the full
    static grid and reproduce them to ~1e-13, which is what identifies CFxtot /
    CFytot / CFztot as body-axis forces in OpenVSP axes.  Raises if the identity
    stops holding, because every table below depends on it.
    """
    worst = 0.0
    for p in points:
        a, b = math.radians(p["AoA"]), math.radians(p["Beta"])
        ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
        fx, fy, fz = p["CFxtot"], p["CFytot"], p["CFztot"]
        cl = fz * ca - fx * sa
        cd = fx * ca * cb - fy * sb + fz * sa * cb
        cs = fx * ca * sb + fy * cb + fz * sa * sb
        worst = max(worst, abs(cl - p["CLtot"]), abs(cd - p["CDtot"]), abs(cs - p["CStot"]))
    if worst > tol:
        raise RuntimeError(
            f"axis convention check failed: worst residual {worst:.3e} > {tol:.1e}. "
            "VSPAero's force columns are not what this module assumes."
        )
    return worst


def to_aircraft_axes(p: PolarPoint) -> dict[str, float]:
    """Convert one polar point into standard aircraft body-axis coefficients.

    Returns CX, CY, CZ, Cl, Cm, Cn plus the wind-axis CL/CD for cross-checks.
    Moments are about the CG supplied to VSPAero.
    """
    return {
        "alpha": p["AoA"],
        "beta": p["Beta"],
        "CX": -p["CFxtot"],
        "CY": p["CFytot"],
        "CZ": -p["CFztot"],
        "Cl": -p["CMxtot"],
        "Cm": p["CMytot"],
        "Cn": -p["CMztot"],
        "CL": p["CLtot"],
        "CD": p["CDtot"],
        "CDi": p["CDi"],
        "CDo": p["CDo"],
    }


def read_lod(path: Path) -> list[dict]:
    """Parse a VSPAero .lod spanwise-loading file.

    Returns one dict per solved case: {'alpha', 'beta', 'strips': [...]},
    where each strip carries its vortex sheet index, span station and section Cl.
    """
    text = Path(path).read_text().splitlines()
    cases: list[dict] = []
    pending: dict[str, float] = {}
    header: list[str] | None = None
    current: dict | None = None

    for line in text:
        fields = line.split()
        if len(fields) >= 2 and fields[0].endswith("_"):
            try:
                pending[fields[0].rstrip("_")] = float(fields[1])
            except ValueError:
                pass
            continue
        if fields[:2] == ["Iter", "VortexSheet"]:
            header = fields
            current = {"alpha": pending.get("AoA"), "beta": pending.get("Beta"), "strips": []}
            cases.append(current)
            pending = {}
            continue
        if header is None or current is None or len(fields) != len(header):
            continue
        try:
            values = [float(f) for f in fields]
        except ValueError:
            continue
        row = dict(zip(header, values))
        current["strips"].append(
            {
                "sheet": int(row["VortexSheet"]),
                "yavg": row["Yavg"],
                "s_over_b": row["SoverB"],
                "chord": row["Chord"],
                "Cl": row["Cl"],
            }
        )
    if not cases:
        raise RuntimeError(f"no cases parsed from {path}")
    return cases


WING_SHEET = 1  # vortex sheet index of the main wing in this model


def wing_section_cl_extremes(case: dict, sheet: int = WING_SHEET) -> tuple[float, float]:
    """Return (min, max) section Cl over the main wing for one solved case."""
    cls = [s["Cl"] for s in case["strips"] if s["sheet"] == sheet]
    if not cls:
        raise RuntimeError("no strips found for the requested vortex sheet")
    return min(cls), max(cls)
