%% uav_setup_v7_est - workspace for DroneModelv7_EST (closed loop + estimator)
%
% Runs uav_setup_v7_cl first (plant workspace + the closed-loop parameters),
% then adds the sensor and estimator parameters.
%
% Every value here is derived in the comments at its point of use.

thisDir = fileparts(mfilename('fullpath'));
run(fullfile(thisDir,'uav_setup_v7_cl.m'));

d2r = pi/180;
g_  = 9.80665;

%% ---- sensor models ---------------------------------------------------------
% BMI323-class IMU. Gyro bias is modelled POST ground calibration: 0.8 deg/s
% is a turn-on offset, and a vehicle that carries turn-on bias into flight has
% a procedure problem, not an estimator problem.
EST_TS      = Tc;                 % estimator runs at the control rate, 5 Hz
GYRO_RANGE  = 500;                % deg/s, +-
GYRO_LSB    = (2*GYRO_RANGE/65536)*d2r;
GYRO_B0     = [0.05; -0.03; 0.02]*d2r;
GYRO_RW     = 0.01*d2r;           % rad/s per sqrt(s), in-run random walk
ACC_RANGE   = 4;                  % g, +-
ACC_LSB     = (2*ACC_RANGE*g_)/65536;
ACC_BIAS    = [0.10; -0.15; 0.08];
PITOT_FRAC  = 0.01;               % 1% of true airspeed
VZ_SIG      = 0.5;                % m/s on vertical speed
% GYRO_SIG, ACC_SIG, SENS_P and EST_P depend on the sample rate. Rebuild them
% together through uav_set_est_rate whenever EST_TS changes.

%% ---- estimator gains -------------------------------------------------------
% Complementary-filter and air-data reconstruction gains.
EST_KP       = 0.20;
EST_KI       = 0.005;
EST_TAU_VD   = 1.00;
EST_GATE_SIG = 0.05;

B_ = load('body_data.mat');
EST_CYBETA = B_.CYbeta;           % -0.01242, and that smallness is the story
EST_RHO    = 0.9046;              % kg/m^3, nominal at 10,000 ft
clear B_

% Initial attitude for the filter. The simulation starts mid-flight, so there
% is no static window in which to coarse-align off the accelerometer - and
% levelling off an unpowered decelerating airframe gives -80 deg of pitch
% against a true +11.4 deg. This is the equivalent of a completed pre-flight
% alignment: the plant's own initial attitude.
EST_EULER0 = init_euler_orien(:);

% Build the two rate-dependent parameter vectors. Single definition, so the
% rate cannot drift out of step with them.
uav_set_est_rate(EST_TS);

%% ---- channel blend ---------------------------------------------------------
% Per-channel switch between the estimate and plant truth:
%   1 = feed the controller the ESTIMATE, 0 = feed it TRUTH.
% Order: [pqr, alpha, beta, euler].
EST_BLEND = [1; 1; 1; 1];

fprintf('uav_setup_v7_est: estimator %g Hz, KP=%g KI=%g TAU=%g GATE=%g\n', ...
        1/EST_TS, EST_KP, EST_KI, EST_TAU_VD, EST_GATE_SIG);
fprintf('uav_setup_v7_est: EST_BLEND [pqr alpha beta euler] = %s\n', mat2str(EST_BLEND'));
