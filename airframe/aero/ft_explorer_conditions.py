"""Flight condition and reference set for the FT Explorer low-speed aero build.

Single source of truth.  Every VSPAero run and every derived table is generated
from the values in this module, so the condition can be changed in one place and
the whole set regenerated.

Geometry lives in ``../ft_explorer_sport.vsp3`` and is stated in INCHES.  VSPAero
reference lengths and CG must therefore also be given in inches.  Coefficients
are non-dimensional and depend only on Mach and Reynolds number, so the
dimensional ``Vinf``/``Rho`` handed to VSPAero are not used for anything and are
recorded here only to keep the run definition complete.

Cruise speed of 13 m/s is a DERIVED value, not a measured one.
"""

from __future__ import annotations

from math import sqrt

# ---------------------------------------------------------------- units
IN_TO_M = 0.0254
IN2_TO_M2 = IN_TO_M**2

# ------------------------------------------------------- ISA sea level
RHO = 1.225                 # kg/m^3
TEMP_K = 288.15             # K
SPEED_OF_SOUND = 340.294    # m/s
MU = 1.7894e-5              # Pa.s   dynamic viscosity
NU = MU / RHO               # m^2/s  kinematic viscosity
G = 9.80665                 # m/s^2

# --------------------------------------------------------- flight point
# Derived, not measured.  Chosen to sit mid-band of the 120k-220k chord
# Reynolds number range specified for this airframe, and comfortably
# above stall.  Cross-checks are computed at the bottom of this module.
V_CRUISE = 13.0             # m/s

# ------------------------------------------------- geometry (inches)
# These mirror the OpenVSP model.  The runner re-reads Sref/bref/cref from the
# model and asserts agreement, so a geometry edit cannot silently
# desynchronise this file.
SREF_IN2 = 414.929
BREF_IN = 57.0
CREF_IN = 7.279464

# CG from the OpenVSP MassProp result: 57 mm aft of the wing leading edge,
# which is the supplier-specified balance point.
XCG_IN = 12.7441
YCG_IN = 0.0
ZCG_IN = -0.1217

MASS_KG = 0.8765            # nominal all-up, +/- 0.07 kg

# ------------------------------------------------------ derived, SI
SREF_M2 = SREF_IN2 * IN2_TO_M2
BREF_M = BREF_IN * IN_TO_M
CREF_M = CREF_IN * IN_TO_M
ASPECT_RATIO = BREF_M**2 / SREF_M2

MACH = V_CRUISE / SPEED_OF_SOUND
RE_CREF = V_CRUISE * CREF_M / NU
QBAR = 0.5 * RHO * V_CRUISE**2

WEIGHT_N = MASS_KG * G
WING_LOADING = WEIGHT_N / SREF_M2
CL_CRUISE = WEIGHT_N / (QBAR * SREF_M2)

# ------------------------------------------------- validated linear band
# Set by find_band.py from the VSPAero spanwise loading against an estimated
# section Clmax/Clmin.  The criterion gives -9.5 .. +9.5 deg; the band below is
# the conservative integer envelope that stays inside the criterion under EVERY
# section limit in that script's sensitivity sweep (Clmax 1.0-1.2,
# Clmin -0.6..-0.8), whose worst case is -8.51 .. +8.56 deg.
# Trim alpha at cruise is +1.60 deg, so the band is centred on where the
# aircraft actually flies.
ALPHA_MIN_DEG = -8.0
ALPHA_MAX_DEG = 8.0
ALPHA_STEP_DEG = 1.0

BETA_MIN_DEG = -10.0
BETA_MAX_DEG = 10.0
BETA_STEP_DEG = 1.0

# Damping derivatives are tabulated on a coarser alpha grid than the static
# tables.
ALPHA_DAMPING_DEG = [-8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0]

# Control deflections, in degrees.  The supplier specifies a 12 deg throw, so
# the sweep brackets it and allows a linearity check inside the throw.
CONTROL_DEFLECTIONS_DEG = [-12.0, -6.0, 0.0, 6.0, 12.0]

# Estimated 2D section maximum lift for the reconstructed flat-bottom folded
# foam section at this Reynolds number.  This is the weakest number in the
# whole build; it sets the validated band and nothing else.
SECTION_CLMAX_EST = 1.10


def alpha_grid() -> list[float]:
    n = round((ALPHA_MAX_DEG - ALPHA_MIN_DEG) / ALPHA_STEP_DEG) + 1
    return [ALPHA_MIN_DEG + i * ALPHA_STEP_DEG for i in range(n)]


def beta_grid() -> list[float]:
    n = round((BETA_MAX_DEG - BETA_MIN_DEG) / BETA_STEP_DEG) + 1
    return [BETA_MIN_DEG + i * BETA_STEP_DEG for i in range(n)]


def stall_speed(clmax: float = SECTION_CLMAX_EST) -> float:
    return sqrt(2.0 * WEIGHT_N / (RHO * SREF_M2 * clmax))


def summary() -> str:
    lines = [
        "FT Explorer sport wing - low-speed aero condition",
        "=" * 56,
        f"  cruise speed        {V_CRUISE:10.3f} m/s      (DERIVED, not measured)",
        f"  Mach                {MACH:10.5f}",
        f"  Reynolds (cref)     {RE_CREF:10.0f}",
        f"  dynamic pressure    {QBAR:10.3f} Pa",
        f"  density             {RHO:10.4f} kg/m^3   (ISA sea level)",
        "",
        f"  Sref                {SREF_M2:10.5f} m^2      ({SREF_IN2:.3f} in^2)",
        f"  bref                {BREF_M:10.5f} m        ({BREF_IN:.3f} in)",
        f"  cref                {CREF_M:10.5f} m        ({CREF_IN:.4f} in)",
        f"  aspect ratio        {ASPECT_RATIO:10.3f}",
        f"  CG (model x)        {XCG_IN:10.4f} in       (57 mm aft of wing LE)",
        "",
        f"  mass                {MASS_KG:10.4f} kg       (+/- 0.07)",
        f"  weight              {WEIGHT_N:10.4f} N",
        f"  wing loading        {WING_LOADING:10.3f} N/m^2",
        f"  CL at cruise        {CL_CRUISE:10.4f}",
        f"  stall speed est.    {stall_speed():10.3f} m/s     (at CLmax {SECTION_CLMAX_EST})",
        f"  cruise / stall      {V_CRUISE / stall_speed():10.3f}",
        "",
        f"  alpha band          {ALPHA_MIN_DEG:+.1f} .. {ALPHA_MAX_DEG:+.1f} deg, step {ALPHA_STEP_DEG}",
        f"  beta band           {BETA_MIN_DEG:+.1f} .. {BETA_MAX_DEG:+.1f} deg, step {BETA_STEP_DEG}",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print(summary())
