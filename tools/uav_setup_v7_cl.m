%% uav_setup_v7_cl - workspace for the CLOSED-LOOP model DroneModelv7_CL
%
% Runs uav_setup_v7 first, then adds the three parameters the closed-loop
% model needs: the actuator mapping, its polarity, and the command vector.

thisDir = fileparts(mfilename('fullpath'));
run(fullfile(thisDir,'uav_setup_v7.m'));

%% ---- actuator mapping matrix ----------------------------------------------
% Maps the controller's 3-element output u_swm to the airframe's 2 control
% inports:  [delta_a; delta_e] = ACT_MAP * u_swm
%
% The channel-to-axis assignment follows from the controller wiring:
%
%   u_swm(1) = Kp * (p - p_cmd),  p_cmd from PI phi   -> ROLL
%   u_swm(2) = Kq * (q - q_cmd),  q_cmd from PI Alpha -> PITCH
%   u_swm(3) = Kr * (r - r_cmd),  r_cmd from PI Beta  -> YAW
%
% Row 2 is forced: AirFrame inport 2 is "Elevator Deflection", and
% The plant has one lateral actuator for two lateral controller channels.
% ACT_MAP uses positional output ordering; ACT_MAP_V6 exposes the alternate
% sideslip-to-aileron mapping for comparison.
ACT_MAP_SIGN = -1;

ACT_MAP    = ACT_MAP_SIGN * [1 0 0     % delta_a <- roll channel  (positional)
                             0 1 0];   % delta_e <- pitch channel
ACT_MAP_V6 = ACT_MAP_SIGN * [0 0 1     % delta_a <- sideslip channel (v6-style)
                             0 1 0];   % delta_e <- pitch channel

% The controller computes rate minus rate command. The negative mapping sign
% converts that convention to negative feedback for the plant derivative signs.

%% ---- command vector -------------------------------------------------------
% A test input, so the loop has something to hold.
%
% NOTE THE UNIT SPLIT, which is a real property of the controller:
%   element 1 (phi command)   is compared against phi straight off the airframe
%                             -> RADIANS
%   element 2 (alpha command) is compared against Alpha after a rad->deg
%                             Angle Conversion block -> DEGREES
%   element 3 (beta command)  likewise -> DEGREES
% Each channel is a Step block: value DEMAND0 before T_STEP, DEMAND1 after.
% Setting DEMAND0 == DEMAND1 makes that channel a constant.
%
% The default holds the initial condition: wings level, and the alpha the model
% actually starts at, atand(9.626/202.67) = 2.72 deg from init_velo_body.
alpha_trim = atand(init_velo_body(3)/init_velo_body(1));

DEMAND0 = [0; alpha_trim; 0];   % [phi rad; alpha deg; beta deg] before the step
DEMAND1 = [0; alpha_trim; 0];   % after the step
T_STEP  = [20; 20; 20];         % step times (s); model runs t = 10..100

fprintf('uav_setup_v7_cl: ACT_MAP =\n');
disp(ACT_MAP);
fprintf('uav_setup_v7_cl: DEMAND0 = [%g rad; %g deg; %g deg]  ->  DEMAND1 = [%g; %g; %g] at t = [%g %g %g]\n', ...
    DEMAND0, DEMAND1, T_STEP);
