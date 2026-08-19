function uav_set_est_rate(ts)
%UAV_SET_EST_RATE  Set the sensor and estimator sample rate consistently.
%
%   uav_set_est_rate(0.01)   % run the sensor/estimator layer at 100 Hz
%
%   EST_TS does three things at once:
%     1. it is the SampleTime of the Sensor Models / Estimator / Blend charts
%        and of the Unit Delays that hold their state, so it sets how often
%        the blocks are stepped;
%     2. it is the FIRST ELEMENT of EST_P, which the estimator uses as its
%        integration step - q = q + Ts*qd, the V_lp low-pass, and the gyro
%        bias integrator all scale by it;
%     3. it is the first element of SENS_P, and the gyro and accelerometer
%        noise densities are converted to per-sample sigmas by sqrt(1/(2*Ts)).
%
%   Assigning EST_TS alone changes block scheduling without updating the
%   parameter vectors. This function updates all three uses together.
%
%   Operates on the base workspace, and expects uav_setup_v7_est.m to have run
%   first so the rate-INdependent constants exist.

d2r = pi/180;
g_  = 9.80665;

get = @(n) evalin('base', n);

% ---- rate-dependent sensor noise -------------------------------------------
% Densities are per sqrt(Hz); the per-sample sigma is density*sqrt(BW) with
% BW = 1/(2*Ts). Faster sampling therefore means MORE noise per sample, which
% is the honest trade against the shorter dead time.
GYRO_SIG = 0.007*d2r*sqrt(1/(2*ts));
ACC_SIG  = 150e-6*g_*sqrt(1/(2*ts));

SENS_P = [ts; get('GYRO_LSB'); get('GYRO_B0'); get('GYRO_RW'); GYRO_SIG; ...
          get('ACC_LSB'); get('ACC_BIAS'); ACC_SIG; get('PITOT_FRAC'); get('VZ_SIG')];

EST_P  = [ts; get('EST_KP'); get('EST_KI'); get('EST_TAU_VD'); get('EST_GATE_SIG'); ...
          get('mass'); get('Sref'); get('EST_CYBETA'); get('EST_RHO'); get('EST_EULER0')];

assignin('base','EST_TS',   ts);
assignin('base','GYRO_SIG', GYRO_SIG);
assignin('base','ACC_SIG',  ACC_SIG);
assignin('base','SENS_P',   SENS_P);
assignin('base','EST_P',    EST_P);

fprintf('uav_set_est_rate: EST_TS = %g s (%g Hz); SENS_P and EST_P rebuilt to match\n', ts, 1/ts);
end
