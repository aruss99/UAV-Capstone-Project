function [gyro_m, acc_m, V_m, Vz_m, sb_out] = uav_sensors_step(pqr_t, acc_t, Vb_t, Ve_t, P, sb_in)
%UAV_SENSORS_STEP  BMI323 / MS4525DO-class sensor models.
%
%   SIMULATION ONLY. This block represents physical hardware; it is not part
%   of the estimator and would never be synthesised. It is kept separate from
%   uav_estimator_step.m so the estimator stays a clean HDL-targetable
%   artifact.
%
%   STATELESS, for the same reason as the estimator: `persistent` variables in
%   an external function called from a MATLAB Function block are not
%   Simulink-managed state and advance on every solver evaluation rather than
%   once per sample. The gyro bias random walk is therefore carried in sb.
%
%   Truth in, measurements out:
%     pqr_t   body rates            (rad/s)   -> gyro
%     acc_t   AirFrame "Accels"     (m/s^2)   -> accelerometer
%             NOTE: Accels is ALREADY SPECIFIC FORCE
%             (= vdot_b + omega x v_b - g_body). Do not subtract gravity again.
%     Vb_t    body velocity         (m/s)     -> airspeed magnitude
%     Ve_t    earth velocity (NED)  (m/s)     -> vertical speed
%
%   P  = [Ts; GYRO_LSB; GYRO_B0(1:3); GYRO_RW; GYRO_SIG;
%         ACC_LSB; ACC_BIAS(1:3); ACC_SIG; PITOT_FRAC; VZ_SIG]
%   sb = [bias_rw(1:3); initialised_flag]
%
%   The airspeed channel is modelled at the true-airspeed level with a
%   fractional error rather than modelling the pitot -> dynamic pressure ->
%   density -> TAS chain: the plant descends far enough that a standard
%   atmosphere would be extrapolating.

%#codegen
Ts         = P(1);
GYRO_LSB   = P(2);
GYRO_B0    = P(3:5);
GYRO_RW    = P(6);
GYRO_SIG   = P(7);
ACC_LSB    = P(8);
ACC_BIAS   = P(9:11);
ACC_SIG    = P(12);
PITOT_FRAC = P(13);
VZ_SIG     = P(14);

bias_rw = sb_in(1:3);
init    = sb_in(4);
if init < 0.5
    bias_rw = zeros(3,1);
end

% ---- gyro: residual bias after ground calibration + in-run random walk -----
bias_rw = bias_rw + GYRO_RW*sqrt(Ts)*randn(3,1);
g_meas  = pqr_t(:) + GYRO_B0 + bias_rw + GYRO_SIG*randn(3,1);
gyro_m  = round(g_meas/GYRO_LSB)*GYRO_LSB;

% ---- accelerometer: bias + noise + 16-bit quantisation ---------------------
a_meas  = acc_t(:) + ACC_BIAS + ACC_SIG*randn(3,1);
acc_m   = round(a_meas/ACC_LSB)*ACC_LSB;

% ---- airspeed and vertical speed -------------------------------------------
V_true  = sqrt(Vb_t(1)^2 + Vb_t(2)^2 + Vb_t(3)^2);
V_m     = V_true*(1 + PITOT_FRAC*randn);
Vz_m    = Ve_t(3) + VZ_SIG*randn;

sb_out  = [bias_rw; 1];
end
