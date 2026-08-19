"""Reconstruct the FT Explorer sport-wing section from its full-scale plan.

This is a geometric reconstruction, not a claim that the folded foam wing was
manufactured to an analytic airfoil.  The four developed upper-skin lengths are
read directly from page 5 of ``FT_Explorer_v1_Full-Size.pdf``.  The supplier's
nominal 3/16-inch foam board and the double-layer folded spar set the maximum
outer depth.

The build video confirms a flat lower skin, two upper-skin score creases, the
top skin pressed onto the spar, and the aileron cut only after closing the wing.
It does not expose a ruler-normal root section, so the two aft facet angles are
retained below as explicit reconstruction assumptions.  Change those values if
a square-on root photograph or a physical section becomes available.
"""

from __future__ import annotations

from math import cos, hypot, radians, sin, sqrt
from pathlib import Path


POINTS_PER_FACET = 6

# Developed lengths from the supplier's full-scale vector plan, page 5.
PLAN_POINTS_PER_INCH = 72.0
UPPER_SEGMENTS_IN = (
    153.54 / PLAN_POINTS_PER_INCH,
    116.16 / PLAN_POINTS_PER_INCH,
    147.00 / PLAN_POINTS_PER_INCH,
    122.82 / PLAN_POINTS_PER_INCH,
)

# The folded spar is two nominal foam-board layers tall.  Its outside height is
# the bottom skin plus two spar layers plus the top skin: four nominal layers.
FOAM_BOARD_IN = 3.0 / 16.0
MAX_OUTER_DEPTH_IN = 4.0 * FOAM_BOARD_IN

# Bounded assumptions from the official closing sequence: the first aft panel
# falls gently off the spar and the next panel descends to the aileron hinge.
FACET_2_ANGLE_DEG = -5.0
FACET_3_ANGLE_DEG = -15.0


def reconstruct_vertices() -> list[tuple[float, float]]:
    """Return dimensional (x, z) vertices from leading to trailing edge."""

    first, second, third, aileron = UPPER_SEGMENTS_IN
    spar_x = sqrt(first**2 - MAX_OUTER_DEPTH_IN**2)
    x2 = spar_x + second * cos(radians(FACET_2_ANGLE_DEG))
    z2 = MAX_OUTER_DEPTH_IN + second * sin(radians(FACET_2_ANGLE_DEG))
    hinge_x = x2 + third * cos(radians(FACET_3_ANGLE_DEG))
    hinge_z = z2 + third * sin(radians(FACET_3_ANGLE_DEG))
    if hinge_z < 0 or hinge_z > aileron:
        raise ValueError("Facet assumptions cannot close the aileron to the flat bottom")
    trailing_x = hinge_x + sqrt(aileron**2 - hinge_z**2)
    return [
        (0.0, 0.0),
        (spar_x, MAX_OUTER_DEPTH_IN),
        (x2, z2),
        (hinge_x, hinge_z),
        (trailing_x, 0.0),
    ]


def interpolate_facets(
    vertices: list[tuple[float, float]], points_per_facet: int = POINTS_PER_FACET
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index, (start, end) in enumerate(zip(vertices[:-1], vertices[1:])):
        for sample in range(points_per_facet):
            if index and sample == 0:
                continue
            fraction = sample / (points_per_facet - 1)
            points.append(
                (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
            )
    return points


def write_openvsp_airfoil(path: Path) -> None:
    vertices = reconstruct_vertices()
    chord = vertices[-1][0]
    upper = [(x / chord, z / chord) for x, z in interpolate_facets(vertices)]
    lower = [(index / 20.0, 0.0) for index in range(21)]

    lines = [
        "FT EXPLORER SPORT WING RECONSTRUCTED GEOM AIRFOIL FILE",
        "Plan-derived folded foam-board section; see reconstruct_sport_airfoil.py",
        "0\tSym Flag (0 - No, 1 - Yes)",
        f"{len(upper)}\tNum Pnts Upper",
        f"{len(lower)}\tNum Pnts Lower",
    ]
    lines.extend(f"{x: .8f} {z: .8f}" for x, z in upper)
    lines.append("")
    lines.extend(f"{x: .8f} {z: .8f}" for x, z in lower)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")

    segment_errors = [
        abs(hypot(b[0] - a[0], b[1] - a[1]) - expected)
        for a, b, expected in zip(vertices[:-1], vertices[1:], UPPER_SEGMENTS_IN)
    ]
    if max(segment_errors) > 1e-10:
        raise AssertionError("Reconstructed surface does not preserve plan lengths")

    print(f"Airfoil file: {path}")
    print(f"Chord: {chord:.4f} in / {chord * 25.4:.1f} mm")
    print(
        "Maximum thickness: "
        f"{MAX_OUTER_DEPTH_IN / chord:.4%} at {vertices[1][0] / chord:.4%} chord"
    )
    print(f"Aileron chord: {(chord - vertices[-2][0]) / chord:.4%}")
    print("Vertices [x/c, z/c]:")
    for x, z in vertices:
        print(f"  {x / chord:.8f}, {z / chord:.8f}")


if __name__ == "__main__":
    write_openvsp_airfoil(Path(__file__).with_name("ft_explorer_sport_reconstructed.af"))
