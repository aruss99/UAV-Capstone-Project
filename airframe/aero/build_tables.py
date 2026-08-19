"""Assemble the FT Explorer low-speed coefficient set from the VSPAero output.

Reads only files under raw/, so every number in the tables traces to a solver
output on disk.  Writes:

    tables/ft_explorer_aero_tables.m   MATLAB/Octave source defining every table
    tables/*.csv                       the same tables, for inspection
    tables/VALIDATION.txt              cross-checks and the parity residuals

CONVENTIONS, all matching the HL-20 tables the Simulink AirFrame consumes
(established by reading the model):

  * Standard aircraft body axes: x forward, y right, z down.
  * Coefficient vector order [CX CY CZ Cl Cm Cn].
  * alpha and beta in DEGREES; static tables indexed (alpha, beta), alpha first.
  * Control derivatives are PER DEGREE of surface deflection, tabulated against
    alpha only, and are multiplied by the deflection in the plant.
  * Moments are about the CG supplied to VSPAero (57 mm aft of the wing LE).

PARITY.  The AirFrame multiplies the aileron CX/CZ/Cm derivatives by |delta|
and the CY/Cl/Cn derivatives by signed delta, which is the correct symmetry for
an antisymmetric surface on a symmetric aircraft.  The same split is applied to
the rudder.  The elevator is symmetric, so all three of its non-zero derivatives
are odd in delta and use signed deflection.
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path

import ft_explorer_conditions as C
import vspaero_io as V

HERE = Path(__file__).resolve().parent
RAW = HERE / "raw"
OUT = HERE / "tables"

COEFFS = ("CX", "CY", "CZ", "Cl", "Cm", "Cn")

# Which coefficients are EVEN in the deflection (multiplied by |delta|) for each
# surface.  Everything else is odd (multiplied by signed delta).
EVEN = {
    "aileron": ("CX", "CZ", "Cm"),
    "rudder": ("CX", "CZ", "Cm"),
    "elevator": (),
}


# --------------------------------------------------------------- helpers
def polar(case: str) -> list[V.PolarPoint]:
    return V.read_polar(RAW / case / "ft_explorer_aero.polar")


def by_alpha(case: str) -> dict[float, dict[str, float]]:
    return {round(p["AoA"], 6): V.to_aircraft_axes(p) for p in polar(case)}


def parasite_cd0() -> tuple[float, list[tuple[str, float]]]:
    """Total CD0 and its per-component breakdown from the OpenVSP build-up."""
    csv = RAW / "parasite" / "ft_explorer_aero_ParasiteBuildUp.csv"
    comps: list[tuple[str, float]] = []
    total = None
    for line in csv.read_text().splitlines():
        f = [x.strip() for x in line.split(",")]
        if len(f) >= 13 and f[0] and not f[0].startswith(("Component", "PARASITE", "Mach", "Temp", "Lam")):
            try:
                comps.append((f[0], float(f[11])))
            except ValueError:
                pass
        if "Totals:" in line:
            total = float(f[f.index("Totals:") + 2])
    if total is None:
        raise RuntimeError(f"no total found in {csv}")
    return total, comps


# ------------------------------------------------------- static tables
def static_tables(cd0: float) -> dict[str, list[list[float]]]:
    """(alpha, beta) tables for all six body-axis coefficients.

    The VSPAero solution is inviscid, so the parasite drag is added here.  Drag
    acts opposite the flight path; resolved into body axes at incidence alpha
    that is CX -= CD0*cos(alpha), CZ -= CD0*sin(alpha).
    """
    pts = V.read_polar(RAW / "static_grid" / "ft_explorer_aero.polar")
    V.check_axis_convention(pts)

    alphas = C.alpha_grid()
    betas = C.beta_grid()
    index = {(round(p["AoA"], 6), round(p["Beta"], 6)): V.to_aircraft_axes(p) for p in pts}

    tables = {c: [[0.0] * len(betas) for _ in alphas] for c in COEFFS}
    for i, a in enumerate(alphas):
        for j, b in enumerate(betas):
            key = (round(a, 6), round(b, 6))
            if key not in index:
                raise RuntimeError(f"static grid missing point alpha={a} beta={b}")
            v = dict(index[key])
            ar = math.radians(a)
            v["CX"] -= cd0 * math.cos(ar)
            v["CZ"] -= cd0 * math.sin(ar)
            for c in COEFFS:
                tables[c][i][j] = v[c]
    return tables


# ------------------------------------------------------ damping (.stab)
STAB_ROWS = {  # aircraft-convention row label in the .stab derivative block
    "CY": "CFy",
    "Cl": "CMl",
    "Cn": "CMn",
    "CZ": "CFz",
    "Cm": "CMm",
    "CX": "CFx",
}


def damping_tables() -> tuple[list[float], dict[str, list[float]]]:
    """Rate derivatives vs alpha, read from the stability run.

    VSPAero reports the p/q/r columns already in the standard non-dimensional
    form (per pb/2V, qc/2V, rb/2V).  That is confirmed by magnitude: the roll
    damping comes out at -0.59 and the pitch damping at -10.6, which are
    ordinary values; treating them as per rad/time and rescaling by 2V/b would
    give -2.1 and -292, the second of which is not physical.
    """
    text = (RAW / "damping" / "ft_explorer_aero.stab").read_text().splitlines()

    alphas: list[float] = []
    wanted = {
        "Clp": ("CMl", "p"), "Clr": ("CMl", "r"), "Cl_beta": ("CMl", "Beta"),
        "Cmq": ("CMm", "q"), "Cm_alpha": ("CMm", "Alpha"),
        "Cnp": ("CMn", "p"), "Cnr": ("CMn", "r"), "Cn_beta": ("CMn", "Beta"),
        "CYp": ("CFy", "p"), "CYr": ("CFy", "r"), "CY_beta": ("CFy", "Beta"),
        "CZq": ("CFz", "q"), "CZ_alpha": ("CFz", "Alpha"),
    }
    out: dict[str, list[float]] = {k: [] for k in wanted}

    header: list[str] | None = None
    pending_alpha: float | None = None
    for line in text:
        f = line.split()
        if len(f) >= 2 and f[0] == "AoA_":
            pending_alpha = float(f[1])
        if f[:2] == ["Coef", "Total"]:
            header = f
            if pending_alpha is not None:
                alphas.append(pending_alpha)
            continue
        if header is None or not f:
            continue
        if f[0] in {"CFx", "CFy", "CFz", "CMx", "CMy", "CMz", "CL", "CD", "CS", "CMl", "CMm", "CMn"}:
            # columns: Coef Total Alpha Beta p q r Mach U ConGrp...
            names = ["Coef", "Total", "Alpha", "Beta", "p", "q", "r", "Mach", "U"]
            vals = dict(zip(names, [f[0]] + [float(x) for x in f[1:9]]))
            for key, (row, col) in wanted.items():
                if f[0] == row:
                    out[key].append(vals[col])
    n = len(alphas)
    for k, v in out.items():
        if len(v) != n:
            raise RuntimeError(f"damping: {k} has {len(v)} entries for {n} alphas")
    return alphas, out


# --------------------------------------------- control derivatives
def control_tables() -> tuple[list[float], dict[str, dict[str, list[float]]], dict]:
    """Per-degree control derivatives vs alpha, with a linearity diagnostic."""
    alphas = C.alpha_grid()
    derivs: dict[str, dict[str, list[float]]] = {}
    diag: dict = {}

    for axis in ("aileron", "elevator", "rudder"):
        zero = by_alpha(f"ctrl_{axis}_p0")
        big_p, big_m = by_alpha(f"ctrl_{axis}_p12"), by_alpha(f"ctrl_{axis}_m12")
        sml_p, sml_m = by_alpha(f"ctrl_{axis}_p6"), by_alpha(f"ctrl_{axis}_m6")

        derivs[axis] = {c: [] for c in COEFFS}
        diag[axis] = {"linearity": {c: 0.0 for c in COEFFS}, "dropped_parity": {c: 0.0 for c in COEFFS}}

        for a in alphas:
            k = round(a, 6)
            for c in COEFFS:
                d12_odd = (big_p[k][c] - big_m[k][c]) / 24.0
                d6_odd = (sml_p[k][c] - sml_m[k][c]) / 12.0
                d12_even = ((big_p[k][c] + big_m[k][c]) / 2.0 - zero[k][c]) / 12.0
                d6_even = ((sml_p[k][c] + sml_m[k][c]) / 2.0 - zero[k][c]) / 6.0

                if c in EVEN[axis]:
                    use, other, ref = d12_even, d12_odd, d6_even
                else:
                    use, other, ref = d12_odd, d12_even, d6_odd

                derivs[axis][c].append(use)
                diag[axis]["linearity"][c] = max(diag[axis]["linearity"][c], abs(use - ref))
                diag[axis]["dropped_parity"][c] = max(diag[axis]["dropped_parity"][c], abs(other))

    return alphas, derivs, diag


# ------------------------------------------------------------- emit
def fmt_matrix(name: str, rows: list[list[float]]) -> str:
    body = ";\n    ".join(" ".join(f"{v:+.8e}" for v in r) for r in rows)
    return f"{name} = [\n    {body}\n];\n"


def fmt_vector(name: str, vals: list[float]) -> str:
    return f"{name} = [{' '.join(f'{v:+.8e}' for v in vals)}];\n"


def main() -> None:
    OUT.mkdir(exist_ok=True)
    cd0, comps = parasite_cd0()
    stat = static_tables(cd0)
    damp_alpha, damp = damping_tables()
    ctrl_alpha, ctrl, diag = control_tables()

    alphas, betas = C.alpha_grid(), C.beta_grid()

    lines = [
        "% FT Explorer sport wing - low-speed aerodynamic coefficient set",
        "% GENERATED by aero/build_tables.py from VSPAero output under aero/raw/.",
        "% Do not edit by hand; regenerate instead.",
        "%",
        f"% Condition: sea level, {C.V_CRUISE} m/s, Mach {C.MACH:.5f}, Re_cref {C.RE_CREF:.0f}",
        f"% Reference: Sref {C.SREF_M2:.6f} m^2, bref {C.BREF_M:.5f} m, cref {C.CREF_M:.6f} m",
        f"% CG:        57 mm aft of wing LE (model x = {C.XCG_IN} in)",
        f"% Validity:  alpha {C.ALPHA_MIN_DEG:+.0f}..{C.ALPHA_MAX_DEG:+.0f} deg, "
        f"beta {C.BETA_MIN_DEG:+.0f}..{C.BETA_MAX_DEG:+.0f} deg. OUTSIDE THIS BAND THE",
        "%            TABLES ARE NOT VALID - the vortex-lattice solution has no stall.",
        "% Axes:      x forward, y right, z down. Angles in DEGREES.",
        "% Controls:  per DEGREE of deflection. CX/CZ/Cm of aileron and rudder",
        "%            multiply |delta|; all other control terms multiply signed delta.",
        "",
        f"Sref = {C.SREF_M2:.8e};   % m^2",
        f"bref = {C.BREF_M:.8e};   % m",
        f"cref = {C.CREF_M:.8e};   % m",
        f"mass_nominal = {C.MASS_KG:.6f};   % kg, +/- 0.07",
        f"CD0_parasite = {cd0:.8e};   % included in CX0/CZ0 below",
        "",
        fmt_vector("alpha_vec", alphas) + "% deg",
        fmt_vector("beta_vec", betas) + "% deg",
        "",
        "% ---- static coefficients, indexed (alpha, beta) ----",
    ]
    for c in COEFFS:
        lines.append(fmt_matrix(f"{c}0", stat[c]))

    lines += ["% ---- rate derivatives vs alpha (standard non-dimensional form) ----",
              fmt_vector("alpha_vec_damping", damp_alpha) + "% deg"]
    for k in ("Clp", "Clr", "Cmq", "Cnp", "Cnr", "CYp", "CYr", "CZq"):
        lines.append(fmt_vector(k, damp[k]))

    lines += ["% ---- static derivatives vs alpha, from the same stability run ----"]
    for k in ("CY_beta", "Cl_beta", "Cn_beta", "Cm_alpha", "CZ_alpha"):
        lines.append(fmt_vector(k, damp[k]) + "% per rad")

    lines += ["", "% ---- control derivatives, per degree, vs alpha ----",
              fmt_vector("alpha_vec_control", ctrl_alpha) + "% deg"]
    suffix = {"aileron": "da", "elevator": "de", "rudder": "dr"}
    for axis, sfx in suffix.items():
        for c in COEFFS:
            vals = ctrl[axis][c]
            if max(abs(v) for v in vals) < 1e-12:
                continue
            lines.append(fmt_vector(f"{c}{sfx}", vals))

    (OUT / "ft_explorer_aero_tables.m").write_text("\n".join(lines), encoding="ascii")

    # ------------------------------------------------------------ CSVs
    def write_csv(name: str, header: list[str], rows: list[list]) -> None:
        with (OUT / name).open("w", encoding="ascii") as fh:
            fh.write(",".join(header) + "\n")
            for r in rows:
                fh.write(",".join(f"{v:.8g}" if isinstance(v, float) else str(v) for v in r) + "\n")

    for c in COEFFS:
        write_csv(f"static_{c}0.csv",
                  ["alpha_deg"] + [f"beta_{b:+.0f}" for b in betas],
                  [[a] + stat[c][i] for i, a in enumerate(alphas)])

    write_csv("damping.csv",
              ["alpha_deg", "Clp", "Clr", "Cmq", "Cnp", "Cnr", "CYp", "CYr", "CZq"],
              [[damp_alpha[i]] + [damp[k][i] for k in ("Clp", "Clr", "Cmq", "Cnp", "Cnr", "CYp", "CYr", "CZq")]
               for i in range(len(damp_alpha))])

    for axis, sfx in suffix.items():
        write_csv(f"control_{axis}.csv",
                  ["alpha_deg"] + [f"{c}{sfx}" for c in COEFFS],
                  [[ctrl_alpha[i]] + [ctrl[axis][c][i] for c in COEFFS] for i in range(len(ctrl_alpha))])

    write_csv("parasite_breakdown.csv", ["component", "Cd", "percent"],
              [[n, v, 100.0 * v / cd0] for n, v in comps])

    print(f"wrote {OUT}")
    print(f"  static grid {len(alphas)} alpha x {len(betas)} beta")
    print(f"  damping     {len(damp_alpha)} alpha")
    print(f"  control     {len(ctrl_alpha)} alpha x 3 surfaces")
    print(f"  CD0         {cd0:.5f}")

    # ------------------------------------------------------------ .mat
    m = OUT / "ft_explorer_aero_tables.m"
    mat = OUT / "ft_explorer_aero_tables.mat"
    octave = Path(r"C:/octave-8.4.0-w64-64/mingw64/bin/octave-cli.exe")
    if octave.exists():
        r = subprocess.run(
            [str(octave), "--no-gui", "--eval",
             f"run('{m.as_posix()}'); clear ans; save('-v7','{mat.as_posix()}');"],
            capture_output=True, text=True, cwd=OUT)
        if r.returncode != 0:
            print("  .mat generation FAILED:\n" + r.stderr[:2000])
        else:
            print(f"  wrote {mat.name}")
    else:
        print("  octave not found; .mat not generated")

    return diag


if __name__ == "__main__":
    main()
