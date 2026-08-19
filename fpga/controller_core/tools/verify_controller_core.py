#!/usr/bin/env python3
"""Generate and verify deterministic vectors for the FT Explorer controller.

Commands:
    python verify_controller_core.py generate VECTORS EXPECTED REPORT
    python verify_controller_core.py check EXPECTED ACTUAL

The fixed-point reference is bit-exact with ``ftx_controller_core.v``.  A
separate floating-point implementation uses the unquantized Simulink gains and
reports the quantization difference over the same 1,200-sample sequence.
"""

from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

SCALE = 1 << 10
COEFF_SCALE = 1 << 14

P_PHI_Q = 81920
P_THETA_Q = 819
P_BETA_Q = -819
I_PHI_DT_Q = 1638
I_THETA_DT_Q = 41
I_BETA_DT_Q = -20
KP_Q = 4989
KQ_Q = -41028
KR_Q = 147675
LAG_A_Q = 13414
LAG_B_Q = 2970

RATE_LIMIT_Q = 2 * SCALE
SURFACE_LIMIT_Q = 12 * SCALE
INTEGRAL_LIMIT_Q24 = 4 * (1 << 24)

P_PHI = 5.0
P_THETA = 0.05
P_BETA = -0.05
I_PHI = 5.0
I_THETA = 0.125
I_BETA = -0.0625
KP = 0.304503055367
KQ = -2.50416204121
KR = 9.01334991223
DT = 0.02
LAG_A = math.exp(-10.0 * DT)
LAG_B = 1.0 - LAG_A


def clamp(value: int | float, lower: int | float, upper: int | float):
    return lower if value < lower else upper if value > upper else value


def q_from_float(value: float) -> int:
    return int(clamp(round(value * SCALE), -32768, 32767))


def rounded_shift_q14(product: int) -> int:
    magnitude = (abs(product) + (1 << 13)) >> 14
    value = -magnitude if product < 0 else magnitude
    return int(clamp(value, -32768, 32767))


def qmul(signal_q10: int, coefficient_q14: int) -> int:
    return rounded_shift_q14(signal_q10 * coefficient_q14)


@dataclass
class FixedState:
    integral_phi_q24: int = 0
    integral_theta_q24: int = 0
    integral_beta_q24: int = 0
    lag_phi_q10: int = 0
    lag_theta_q10: int = 0
    lag_beta_q10: int = 0


@dataclass
class FloatState:
    integral_phi: float = 0.0
    integral_theta: float = 0.0
    integral_beta: float = 0.0
    lag_phi: float = 0.0
    lag_theta: float = 0.0
    lag_beta: float = 0.0


def fixed_axis(
    error_q10: int,
    rate_q10: int,
    integral_q24: int,
    lag_q10: int,
    p_q14: int,
    i_dt_q14: int,
    rate_gain_q14: int,
) -> tuple[int, int, int]:
    proportional_q10 = qmul(error_q10, p_q14)
    pi_sum_q10 = proportional_q10 + (integral_q24 >> 14)
    command_q10 = int(clamp(pi_sum_q10, -RATE_LIMIT_Q, RATE_LIMIT_Q))

    delta_q24 = error_q10 * i_dt_q14
    driving_further = (
        (pi_sum_q10 > RATE_LIMIT_Q and delta_q24 > 0)
        or (pi_sum_q10 < -RATE_LIMIT_Q and delta_q24 < 0)
    )
    if not driving_further:
        integral_q24 = int(
            clamp(
                integral_q24 + delta_q24,
                -INTEGRAL_LIMIT_Q24,
                INTEGRAL_LIMIT_Q24,
            )
        )

    lag_q10 = int(
        clamp(qmul(lag_q10, LAG_A_Q) + qmul(command_q10, LAG_B_Q), -32768, 32767)
    )
    surface_q10 = int(
        clamp(qmul(lag_q10 - rate_q10, rate_gain_q14), -SURFACE_LIMIT_Q, SURFACE_LIMIT_Q)
    )
    return surface_q10, integral_q24, lag_q10


def fixed_step(state: FixedState, vector: tuple[int, ...]) -> tuple[int, int, int]:
    p, q, r, phi, theta, beta, phi_cmd, theta_cmd, beta_cmd = vector
    da, state.integral_phi_q24, state.lag_phi_q10 = fixed_axis(
        int(clamp(phi_cmd - phi, -32768, 32767)),
        p,
        state.integral_phi_q24,
        state.lag_phi_q10,
        P_PHI_Q,
        I_PHI_DT_Q,
        KP_Q,
    )
    de, state.integral_theta_q24, state.lag_theta_q10 = fixed_axis(
        int(clamp(theta_cmd - theta, -32768, 32767)),
        q,
        state.integral_theta_q24,
        state.lag_theta_q10,
        P_THETA_Q,
        I_THETA_DT_Q,
        KQ_Q,
    )
    dr, state.integral_beta_q24, state.lag_beta_q10 = fixed_axis(
        int(clamp(beta_cmd - beta, -32768, 32767)),
        r,
        state.integral_beta_q24,
        state.lag_beta_q10,
        P_BETA_Q,
        I_BETA_DT_Q,
        KR_Q,
    )
    return da, de, dr


def float_axis(
    error: float,
    rate: float,
    integral: float,
    lag: float,
    p_gain: float,
    i_gain: float,
    rate_gain: float,
) -> tuple[float, float, float]:
    pi_sum = p_gain * error + integral
    command = clamp(pi_sum, -2.0, 2.0)
    delta = i_gain * error * DT
    driving_further = (pi_sum > 2.0 and delta > 0.0) or (pi_sum < -2.0 and delta < 0.0)
    if not driving_further:
        integral = clamp(integral + delta, -4.0, 4.0)
    lag = LAG_A * lag + LAG_B * command
    surface = clamp(rate_gain * (lag - rate), -12.0, 12.0)
    return surface, integral, lag


def float_step(state: FloatState, vector_q: tuple[int, ...]) -> tuple[float, float, float]:
    p, q, r, phi, theta, beta, phi_cmd, theta_cmd, beta_cmd = (
        value / SCALE for value in vector_q
    )
    da, state.integral_phi, state.lag_phi = float_axis(
        phi_cmd - phi, p, state.integral_phi, state.lag_phi, P_PHI, I_PHI, KP
    )
    de, state.integral_theta, state.lag_theta = float_axis(
        theta_cmd - theta,
        q,
        state.integral_theta,
        state.lag_theta,
        P_THETA,
        I_THETA,
        KQ,
    )
    dr, state.integral_beta, state.lag_beta = float_axis(
        beta_cmd - beta, r, state.integral_beta, state.lag_beta, P_BETA, I_BETA, KR
    )
    return da, de, dr


def make_vectors() -> list[tuple[int, ...]]:
    vectors: list[tuple[int, ...]] = []
    rng = random.Random(20260813)

    for index in range(600):
        time_s = index * DT
        phi_command = 0.0 if index < 100 else math.radians(2.0 if index < 350 else -2.0)
        theta_command = 1.6 if index < 200 else 3.6 if index < 450 else 1.6
        beta_command = 0.0

        phi = math.radians(0.3 * math.sin(0.8 * time_s))
        theta = 1.6 + 0.4 * math.sin(0.45 * time_s)
        beta = 2.0 * math.sin(1.7 * time_s)
        p = 0.015 * math.cos(0.8 * time_s)
        q = 0.025 * math.cos(0.45 * time_s)
        r = 0.04 * math.cos(1.7 * time_s)
        vectors.append(
            tuple(
                q_from_float(value)
                for value in (
                    p,
                    q,
                    r,
                    phi,
                    theta,
                    beta,
                    phi_command,
                    theta_command,
                    beta_command,
                )
            )
        )

    # Deterministic bounded fuzz covers signs, rate-command saturation and
    # surface saturation without leaving the controller's Q5.10 domain.
    for _ in range(600):
        vectors.append(
            tuple(
                q_from_float(value)
                for value in (
                    rng.uniform(-1.5, 1.5),
                    rng.uniform(-1.5, 1.5),
                    rng.uniform(-1.5, 1.5),
                    rng.uniform(-0.25, 0.25),
                    rng.uniform(-8.0, 8.0),
                    rng.uniform(-10.0, 10.0),
                    rng.uniform(-0.20, 0.20),
                    rng.uniform(-8.0, 8.0),
                    rng.uniform(-8.0, 8.0),
                )
            )
        )
    return vectors


def generate(vector_path: Path, expected_path: Path, report_path: Path) -> None:
    vectors = make_vectors()
    fixed_state = FixedState()
    float_state = FloatState()
    maximum_error = [0.0, 0.0, 0.0]
    total_squared_error = [0.0, 0.0, 0.0]

    with vector_path.open("w", encoding="ascii", newline="\n") as vector_file, expected_path.open(
        "w", encoding="ascii", newline="\n"
    ) as expected_file:
        for vector in vectors:
            fixed_output = fixed_step(fixed_state, vector)
            float_output = float_step(float_state, vector)
            vector_file.write(" ".join(str(value) for value in vector) + "\n")
            expected_file.write(" ".join(str(value) for value in fixed_output) + "\n")
            for channel in range(3):
                error = fixed_output[channel] / SCALE - float_output[channel]
                maximum_error[channel] = max(maximum_error[channel], abs(error))
                total_squared_error[channel] += error * error

    rms_error = [math.sqrt(value / len(vectors)) for value in total_squared_error]
    report = (
        f"vectors={len(vectors)}\n"
        f"format=signed Q5.10 ports, signed Q5.14 coefficients\n"
        f"max_abs_error_deg={maximum_error[0]:.6f},{maximum_error[1]:.6f},{maximum_error[2]:.6f}\n"
        f"rms_error_deg={rms_error[0]:.6f},{rms_error[1]:.6f},{rms_error[2]:.6f}\n"
    )
    report_path.write_text(report, encoding="ascii")
    print(report, end="")


def check(expected_path: Path, actual_path: Path) -> None:
    expected = expected_path.read_text(encoding="ascii").splitlines()
    actual = actual_path.read_text(encoding="ascii").splitlines()
    if len(expected) != len(actual):
        raise SystemExit(f"FAIL: expected {len(expected)} RTL rows, received {len(actual)}")
    mismatches = [(index + 1, e, a) for index, (e, a) in enumerate(zip(expected, actual)) if e != a]
    if mismatches:
        for index, expected_row, actual_row in mismatches[:10]:
            print(f"row {index}: expected {expected_row}; actual {actual_row}")
        raise SystemExit(f"FAIL: {len(mismatches)} of {len(expected)} RTL rows differ")
    print(f"RTL_FIXED_POINT_MATCH=PASS ({len(expected)} vectors)")


def main() -> None:
    if len(sys.argv) == 5 and sys.argv[1] == "generate":
        generate(Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]))
    elif len(sys.argv) == 4 and sys.argv[1] == "check":
        check(Path(sys.argv[2]), Path(sys.argv[3]))
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
