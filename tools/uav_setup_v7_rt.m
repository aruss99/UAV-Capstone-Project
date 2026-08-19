%% uav_setup_v7_rt - workspace for DroneModelv7_RT (the re-tunable controller)
%
% Runs uav_setup_v7_est first, then adds the saturation limits, PI output
% limits and gain scale factors DroneModelv7_RT needs.

thisDir = fileparts(mfilename('fullpath'));
run(fullfile(thisDir,'uav_setup_v7_est.m'));

%% ---- sample rates -----------------------------------------------------------
% Feedback path (sensors, estimator and latency) at 100 Hz:
uav_set_est_rate(0.01);   % NOT assignin - this rebuilds SENS_P and EST_P too

% Controller sample time. build_rt_v7.m expresses the lag coefficients as
% functions of Tc so their dynamics remain consistent with this value.
Tc = 0.2;

%% ---- surface saturation -----------------------------------------------------
% The control derivatives in actuator_data.mat are ~1e-3 per unit, and 25 units
% of elevator gives a pitching-moment coefficient change of -0.034 against a
% Cm0 that ranges to 0.049 - the same order as the baseline moment. These are
% per-DEGREE derivatives, so unlimited commands of 100+ units are meaningless.
RT_SURF_LIM = 25;        % deg, per surface

%% ---- PI output limits -------------------------------------------------------
% Each PI produces a body-rate command that is differenced against the measured
% rate. P = 0.05 on a 5 deg alpha error gives 0.25, differenced against a rate
% in rad/s, so 0.25 rad/s ~ 14 deg/s. A limit of 0.5 is roughly twice the
% demand the commanded step produces, which bounds windup without clipping
% normal operation.
RT_LIM_ALPHA = 0.5;
RT_LIM_BETA  = 0.5;
RT_LIM_PHI   = 0.5;

%% ---- retune handles ---------------------------------------------------------
% Scale factors on the baseline gains, all 1.0, so a freshly built
% DroneModelv7_RT differs from DroneModelv7_EST ONLY by the limiters above.
% That makes "what did saturation alone buy?" a measurable question before any
% gain is touched.
%
% P gains are S-Function constants in the model and are scaled by Gain blocks
% that build_rt_v7.m inserts. I and rate gains are workspace tables, scaled here.
RT_PA_SCALE = 1.0;       % P alpha  (baseline 0.05)
RT_PB_SCALE = 1.0;       % P beta   (baseline -0.05)
RT_PP_SCALE = 1.0;       % P phi    (baseline 0.05)

RT_KQ_SCALE = 1.0;       % pitch-rate feedback  (baseline -279.2 .. -173.5)
RT_KA_SCALE = 1.0;       % alpha integral       (baseline 0.0423 .. 0.1104)
RT_KP_SCALE = 1.0;       % roll-rate feedback
RT_KR_SCALE = 1.0;       % yaw-rate feedback    (baseline -1155.3 .. -713.9)

% uav_setup_v7.m has already loaded the gain tables into these names, so this
% scales them rather than replacing them.
Kq     = RT_KQ_SCALE * Kq;
Kalpha = RT_KA_SCALE * Kalpha;
Kp     = RT_KP_SCALE * Kp;
Kr     = RT_KR_SCALE * Kr;

fprintf(['uav_setup_v7_rt: Tc = %g s (%g Hz control), EST_TS = %g s (%g Hz feedback)\n' ...
         'uav_setup_v7_rt: surface limit +-%g deg, PI limits [%g %g %g]\n' ...
         'uav_setup_v7_rt: gain scales P[%g %g %g] Kq %g Kalpha %g Kp %g Kr %g\n'], ...
        Tc, 1/Tc, EST_TS, 1/EST_TS, RT_SURF_LIM, RT_LIM_ALPHA, RT_LIM_BETA, RT_LIM_PHI, ...
        RT_PA_SCALE, RT_PB_SCALE, RT_PP_SCALE, RT_KQ_SCALE, RT_KA_SCALE, RT_KP_SCALE, RT_KR_SCALE);
