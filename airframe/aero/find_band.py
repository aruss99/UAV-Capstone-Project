"""Locate the validated linear alpha band from the VSPAero spanwise loading.

VSPAero's vortex-lattice solution is linear and has no stall, so it will happily
report a lift coefficient at any angle of attack.  The tables must therefore be
restricted to a band where the solution is physically meaningful, and the band
has to come from a stated criterion rather than a guess.

CRITERION: the band ends where the most heavily loaded strip on the main wing
reaches the estimated section maximum lift coefficient.  That is a real
measurement of the VLM solution (the spanwise loading) against an ESTIMATED
section limit.  The section limits are the weakest numbers in this build and are
stated explicitly, with a sensitivity sweep printed below them.

Run after run_cases.py explore.
"""

from __future__ import annotations

from pathlib import Path

import ft_explorer_conditions as C
import vspaero_io as V

HERE = Path(__file__).resolve().parent
EXPLORE = HERE / "raw" / "explore_alpha"

# Estimated section limits for the reconstructed flat-bottom folded foam section
# at chord Reynolds number ~165,000.
#
# Positive: thin, cambered, sharp-ish leading edge at low Reynolds number.  1.10
# is typical for a flat-bottom trainer section in this Reynolds range.
#
# Negative: a flat-bottom section stalls negatively at much smaller magnitude
# than it does positively, because the effective camber works against the flow
# on the pressure side.  -0.70 is the estimate used here.
SECTION_CLMAX = 1.10
SECTION_CLMIN = -0.70


def crossing(xs: list[float], ys: list[float], level: float) -> float | None:
    """First x at which y crosses level, by linear interpolation."""
    for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
        if (y0 - level) * (y1 - level) <= 0 and y0 != y1:
            return x0 + (level - y0) * (x1 - x0) / (y1 - y0)
    return None


def band(clmax: float = SECTION_CLMAX, clmin: float = SECTION_CLMIN) -> tuple[float, float]:
    lod = V.read_lod(EXPLORE / "ft_explorer_aero.lod")
    alphas = [c["alpha"] for c in lod]
    lo = [V.wing_section_cl_extremes(c)[0] for c in lod]
    hi = [V.wing_section_cl_extremes(c)[1] for c in lod]

    upper = crossing(alphas, hi, clmax)
    lower = crossing(alphas, lo, clmin)
    return (lower if lower is not None else min(alphas),
            upper if upper is not None else max(alphas))


def main() -> None:
    pts = V.read_polar(EXPLORE / "ft_explorer_aero.polar")
    worst = V.check_axis_convention(pts)
    print(f"axis convention verified, worst residual {worst:.2e}\n")

    acs = [V.to_aircraft_axes(p) for p in pts]
    a = [x["alpha"] for x in acs]
    cl = [x["CL"] for x in acs]
    cm = [x["Cm"] for x in acs]

    # Fit the lift and moment slopes over the inner, unambiguously linear part.
    inner = [i for i, ang in enumerate(a) if -6.0 <= ang <= 6.0]
    i0, i1 = inner[0], inner[-1]
    cla = (cl[i1] - cl[i0]) / (a[i1] - a[i0])
    cma = (cm[i1] - cm[i0]) / (a[i1] - a[i0])
    al0 = a[i0] - cl[i0] / cla

    print("Whole-aircraft linear characteristics (fitted over -6..+6 deg):")
    print(f"  CL_alpha        {cla:8.5f} /deg   ({cla * 57.29578:6.3f} /rad)")
    print(f"  Cm_alpha        {cma:8.5f} /deg   ({cma * 57.29578:6.3f} /rad)")
    print(f"  alpha_L0        {al0:8.3f} deg")
    print(f"  static margin   {-cma / cla * 100:8.2f} % MAC   ({'STABLE' if cma < 0 else 'UNSTABLE'})")

    alpha_cruise = al0 + C.CL_CRUISE / cla
    print(f"  trim alpha at CL={C.CL_CRUISE:.4f}: {alpha_cruise:.2f} deg\n")

    lo_b, hi_b = band()
    print(f"Validated band from section-Cl criterion "
          f"(Clmax {SECTION_CLMAX:+.2f}, Clmin {SECTION_CLMIN:+.2f}):")
    print(f"  alpha  {lo_b:+.2f} .. {hi_b:+.2f} deg")
    print(f"  cruise alpha {alpha_cruise:+.2f} deg sits "
          f"{alpha_cruise - lo_b:.1f} deg above the lower limit and "
          f"{hi_b - alpha_cruise:.1f} deg below the upper\n")

    print("Sensitivity of the band to the estimated section limits:")
    print(f"  {'Clmax':>7} {'Clmin':>7} {'alpha lo':>10} {'alpha hi':>10}")
    for cx in (1.00, 1.10, 1.20):
        for cn in (-0.60, -0.70, -0.80):
            l, h = band(cx, cn)
            print(f"  {cx:7.2f} {cn:7.2f} {l:10.2f} {h:10.2f}")

    print("\nSpanwise extremes used:")
    lod = V.read_lod(EXPLORE / "ft_explorer_aero.lod")
    print(f"  {'alpha':>7} {'CL':>9} {'wing Cl min':>12} {'wing Cl max':>12}")
    for x, c in zip(acs, lod):
        mn, mx = V.wing_section_cl_extremes(c)
        flag = ""
        if mx > SECTION_CLMAX or mn < SECTION_CLMIN:
            flag = "  <-- outside"
        print(f"  {x['alpha']:7.1f} {x['CL']:9.4f} {mn:12.4f} {mx:12.4f}{flag}")


if __name__ == "__main__":
    main()
