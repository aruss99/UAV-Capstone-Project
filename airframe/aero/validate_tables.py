"""Cross-check the generated coefficient set against independent estimates.

Every check here compares a generated number against something derived a
different way -- lifting-line theory, a tail-volume estimate, a symmetry
argument, or the HL-20 tables.  A check that merely restates the generator
would prove nothing.

Writes tables/VALIDATION.txt and exits non-zero if any hard check fails.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import build_tables as B
import ft_explorer_conditions as C
import vspaero_io as V

HERE = Path(__file__).resolve().parent
OUT = HERE / "tables"

report: list[str] = []
failures: list[str] = []


def say(line: str = "") -> None:
    report.append(line)
    print(line)


def check(name: str, ok: bool, detail: str, hard: bool = True) -> None:
    tag = "PASS" if ok else ("FAIL" if hard else "WARN")
    say(f"  [{tag}] {name}: {detail}")
    if not ok and hard:
        failures.append(name)


def main() -> None:
    cd0, comps = B.parasite_cd0()
    stat = B.static_tables(cd0)
    damp_alpha, damp = B.damping_tables()
    ctrl_alpha, ctrl, diag = B.control_tables()
    alphas, betas = C.alpha_grid(), C.beta_grid()
    ia0 = alphas.index(0.0)
    jb0 = betas.index(0.0)

    say("FT Explorer low-speed coefficient set - validation")
    say("=" * 66)
    say()

    # ---------------------------------------------------------- lift slope
    say("1. Lift-curve slope against lifting-line theory")
    cz = [stat["CZ"][i][jb0] for i in range(len(alphas))]
    # CL ~ -CZ at small alpha; fit over the inner band
    i1, i2 = alphas.index(-4.0), alphas.index(4.0)
    cla_deg = (-cz[i2] + cz[i1]) / (alphas[i2] - alphas[i1])
    cla_rad = cla_deg * 180.0 / math.pi
    ar = C.ASPECT_RATIO
    a0 = 2 * math.pi
    ll = a0 / (1 + a0 / (math.pi * ar * 0.98))
    say(f"     generated CL_alpha (whole aircraft) = {cla_rad:.3f} /rad ({cla_deg:.5f} /deg)")
    say(f"     lifting-line, wing alone, AR {ar:.2f}  = {ll:.3f} /rad")
    check("CL_alpha vs lifting line",
          ll * 0.95 <= cla_rad <= ll * 1.35,
          f"{cla_rad:.3f} /rad sits {100*(cla_rad/ll-1):+.1f}% from the wing-alone value; "
          "the aircraft includes a horizontal tail, so a modest excess is expected")

    # -------------------------------------------------------- static margin
    say()
    say("2. Longitudinal static stability")
    cm = [stat["Cm"][i][jb0] for i in range(len(alphas))]
    cma_deg = (cm[i2] - cm[i1]) / (alphas[i2] - alphas[i1])
    sm = -cma_deg / cla_deg
    say(f"     Cm_alpha = {cma_deg:.6f} /deg,  static margin = {100*sm:.2f} % MAC")
    check("Cm_alpha negative", cma_deg < 0, f"{cma_deg:.6f} /deg -- statically stable")
    check("static margin plausible", 0.05 <= sm <= 0.40,
          f"{100*sm:.1f}% MAC (a trainer normally sits at 15-25%)")

    # neutral point vs tail volume
    xnp_c = C.XCG_IN / C.CREF_IN + sm
    say(f"     implied neutral point = {xnp_c:.3f} cref aft of the nose "
        f"= {(C.XCG_IN + sm*C.CREF_IN):.3f} in")

    # ---------------------------------------------------- lateral stability
    say()
    say("3. Lateral-directional static stability (signs)")
    cyb = (stat["CY"][ia0][betas.index(4.0)] - stat["CY"][ia0][betas.index(-4.0)]) / 8.0
    clb = (stat["Cl"][ia0][betas.index(4.0)] - stat["Cl"][ia0][betas.index(-4.0)]) / 8.0
    cnb = (stat["Cn"][ia0][betas.index(4.0)] - stat["Cn"][ia0][betas.index(-4.0)]) / 8.0
    say(f"     CY_beta = {cyb:+.6f} /deg    Cl_beta = {clb:+.6f} /deg    Cn_beta = {cnb:+.6f} /deg")
    check("Cn_beta > 0 (directionally stable)", cnb > 0, f"{cnb:+.6f} /deg")
    check("Cl_beta < 0 (stable dihedral effect)", clb < 0, f"{clb:+.6f} /deg")
    check("CY_beta < 0", cyb < 0, f"{cyb:+.6f} /deg")
    say("     NOTE: the wing has zero dihedral, so Cl_beta comes almost entirely from the")
    say("     vertical tail above the CG and is correspondingly small.")

    # -------------------------------------------------------------- damping
    say()
    say("4. Rate derivatives against physical expectation")
    i0d = damp_alpha.index(0.0)
    for k, lo, hi in (("Clp", -1.0, -0.2), ("Cmq", -30.0, -4.0), ("Cnr", -0.5, -0.01)):
        v = damp[k][i0d]
        check(f"{k} in a physical range", lo <= v <= hi, f"{v:+.4f} (expected {lo} .. {hi})")
    say(f"     Clr = {damp['Clr'][i0d]:+.4f}   Cnp = {damp['Cnp'][i0d]:+.4f}")

    # cross-check the stability run against the static grid
    say()
    say("5. Stability run vs static grid (independent solves of the same quantity)")
    for name, from_stab, from_grid in (
        ("Cm_alpha /deg", damp["Cm_alpha"][i0d] * math.pi / 180.0, cma_deg),
        ("Cn_beta  /deg", damp["Cn_beta"][i0d] * math.pi / 180.0, cnb),
        ("CY_beta  /deg", damp["CY_beta"][i0d] * math.pi / 180.0, cyb),
    ):
        rel = abs(from_stab - from_grid) / max(abs(from_grid), 1e-9)
        check(f"{name} agreement", rel < 0.12,
              f"stability run {from_stab:+.6f} vs grid {from_grid:+.6f} ({100*rel:.1f}% apart)")

    # ------------------------------------------------------ control signs
    say()
    say("6. Control derivative signs and authority")
    ic0 = ctrl_alpha.index(0.0)
    clda = ctrl["aileron"]["Cl"][ic0]
    cmde = ctrl["elevator"]["Cm"][ic0]
    cndr = ctrl["rudder"]["Cn"][ic0]
    cnda = ctrl["aileron"]["Cn"][ic0]
    check("Cl_da > 0", clda > 0, f"{clda:+.6f} /deg")
    check("Cm_de < 0", cmde < 0, f"{cmde:+.6f} /deg")
    check("Cn_dr > 0", cndr > 0, f"{cndr:+.6f} /deg")
    say(f"     Cn_da = {cnda:+.6f} /deg at alpha 0 "
        f"({'PROVERSE' if cnda > 0 else 'ADVERSE'} yaw)")
    say("     Cn_da across the band:")
    say("       " + "  ".join(f"{a:+.0f}:{ctrl['aileron']['Cn'][i]:+.5f}"
                              for i, a in enumerate(ctrl_alpha) if i % 2 == 0))

    # -------------------------------------------- lateral controllability
    say()
    say("7. Aileron yaw coupling, and whether the rudder can counter it")
    say("     This is the quantity that decides the lateral result: in the HL-20")
    say("     plant Cnda is adverse and comparable to or larger than Clda,")
    say("     with no rudder input at all, so the lateral axis could not close.")
    worst_ratio = 0.0
    worst_a = None
    for i, a in enumerate(ctrl_alpha):
        cl_i = ctrl["aileron"]["Cl"][i]
        cn_i = ctrl["aileron"]["Cn"][i]
        if abs(cl_i) > 1e-9:
            r = abs(cn_i) / abs(cl_i)
            if r > worst_ratio:
                worst_ratio, worst_a = r, a
    say(f"     new airframe:  |Cn_da/Cl_da| <= {worst_ratio:.4f} (worst at alpha {worst_a:+.0f} deg)")
    arch_lo = 9.250e-4 / 2.564e-3
    arch_hi = 2.820e-3 / 5.859e-4
    say(f"     HL-20: |Cnda/Clda| = {arch_lo:.2f} .. {arch_hi:.2f}")
    # full-throw aileron yaw vs full-throw rudder yaw
    thr = 12.0
    yaw_from_ail = max(abs(ctrl["aileron"]["Cn"][i]) for i in range(len(ctrl_alpha))) * thr
    yaw_from_rud = abs(cndr) * thr
    say(f"     yaw from full aileron ({thr:.0f} deg): {yaw_from_ail:.5f}")
    say(f"     yaw from full rudder  ({thr:.0f} deg): {yaw_from_rud:.5f}")
    check("rudder can counter full-throw aileron yaw", yaw_from_rud > yaw_from_ail,
          f"rudder authority is {yaw_from_rud/max(yaw_from_ail,1e-9):.1f}x what the aileron demands")
    say("     CAVEAT: the fuselage, boom and motor pod carry no vortex-lattice loading,")
    say("     so directional stability and rudder sidewash are both optimistic here.")

    # --------------------------------------- comparison with the HL-20 tables
    say()
    say("8. Control authority vs the HL-20 tables")
    say("     The HL-20 plant is a Mach-0.6 lifting body; these ratios are the")
    say("     reason its deflection demands carry no meaning for this airframe.")
    for label, new, old_lo, old_hi in (
        ("Cm_de", cmde, -1.926e-3, 3.326e-5),
        ("Cl_da", clda, 5.859e-4, 2.564e-3),
    ):
        old_mag = max(abs(old_lo), abs(old_hi))
        say(f"     {label}: new {new:+.6f} /deg vs HL-20 {old_lo:+.2e} .. {old_hi:+.2e} "
            f"-> {abs(new)/old_mag:.1f}x the authority per degree")

    # ---------------------------------------------- trim and elevator demand
    say()
    say("9. Trim at cruise, and the elevator needed to hold it")
    cz0 = -stat["CZ"][ia0][jb0]
    a_trim = (C.CL_CRUISE - cz0) / cla_deg
    cm_trim = cm[ia0] + cma_deg * a_trim
    de_trim = -cm_trim / cmde
    say(f"     CL required            {C.CL_CRUISE:.4f}")
    say(f"     trim alpha             {a_trim:+.2f} deg")
    say(f"     Cm at that alpha       {cm_trim:+.5f}")
    say(f"     elevator to trim       {de_trim:+.2f} deg")
    check("trim alpha inside the validated band",
          C.ALPHA_MIN_DEG <= a_trim <= C.ALPHA_MAX_DEG,
          f"{a_trim:+.2f} deg in [{C.ALPHA_MIN_DEG:+.0f}, {C.ALPHA_MAX_DEG:+.0f}]")
    check("elevator to trim within the 12 deg throw", abs(de_trim) <= 12.0,
          f"|{de_trim:.2f}| deg against a 12 deg throw")

    # ------------------------------------------------------------- symmetry
    say()
    say("10. Symmetry of the static grid (a solver sanity check)")
    worst_sym = 0.0
    worst_at = None
    for i in range(len(alphas)):
        for j, b in enumerate(betas):
            jm = betas.index(-b)
            for c, parity in (("CZ", +1), ("Cm", +1), ("CX", +1),
                              ("CY", -1), ("Cl", -1), ("Cn", -1)):
                d = abs(stat[c][i][j] - parity * stat[c][i][jm])
                if d > worst_sym:
                    worst_sym, worst_at = d, (alphas[i], b, c)
    check("beta symmetry", worst_sym < 5e-4,
          f"worst mismatch {worst_sym:.2e} at alpha={worst_at[0]:+.0f} beta={worst_at[1]:+.0f} "
          f"on {worst_at[2]}")

    # ------------------------------------------------------------- parity
    say()
    say("11. Control-derivative parity and linearity diagnostics")
    say("     'dropped' is the parity component NOT carried by the table shape;")
    say("     'linearity' is the change in the derivative between a 6 and 12 deg step.")
    for axis in ("aileron", "elevator", "rudder"):
        say(f"     {axis}:")
        for c in B.COEFFS:
            mag = max(abs(v) for v in ctrl[axis][c])
            if mag < 1e-9:
                continue
            dr = diag[axis]["dropped_parity"][c]
            ln = diag[axis]["linearity"][c]
            say(f"        {c:3s} |max| {mag:.6f}   dropped {dr:.2e}   linearity {ln:.2e}")

    # ------------------------------------------------------------ drag
    say()
    say("12. Drag build-up")
    say(f"     CD0 total {cd0:.5f}, fully turbulent (0% laminar assumed)")
    for n, v in comps:
        say(f"        {n:26s} {v:.5f}  ({100*v/cd0:4.1f}%)")
    cdi = C.CL_CRUISE**2 / (math.pi * C.ASPECT_RATIO * 0.85)
    say(f"     induced at cruise (e=0.85) {cdi:.5f}")
    say(f"     total at cruise            {cd0+cdi:.5f}   ->  L/D = {C.CL_CRUISE/(cd0+cdi):.2f}")
    check("cruise L/D plausible for a foam trainer", 5.0 <= C.CL_CRUISE/(cd0+cdi) <= 14.0,
          f"{C.CL_CRUISE/(cd0+cdi):.2f}", hard=False)
    say("     WARNING: the forward fuselage, boom and motor pod are approximate PODs and")
    say(f"     contribute {100*sum(v for n,v in comps if 'APPROX' in n)/cd0:.0f}% of CD0. "
        "That share is not validated geometry.")

    say()
    say("=" * 66)
    if failures:
        say(f"RESULT: {len(failures)} hard check(s) FAILED: {', '.join(failures)}")
    else:
        say("RESULT: all hard checks passed.")

    OUT.mkdir(exist_ok=True)
    (OUT / "VALIDATION.txt").write_text("\n".join(report) + "\n", encoding="ascii")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
