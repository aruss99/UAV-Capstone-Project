%UAV_SETUP_V7_FTX_ATT  Workspace for the FT Explorer pitch-attitude controller.
%
% The aerodynamic plant, sensor models, estimator, rate loops, actuator mapping
% and limits are inherited from DroneModelv7_FTX.  This variant makes two
% explicit flight-test decisions:
%   1. body pitch attitude theta replaces angle of attack in the longitudinal
%      outer loop; and
%   2. the selected FT Power Pack B propulsion is represented by constant
%      EFFECTIVE propulsive power along body +x.
% Theta is expressed in degrees at the PI input, so the existing rate-loop
% units remain unchanged.

run(fullfile(fileparts(mfilename('fullpath')), 'uav_setup_v7_ftx.m'));

%% ---- propulsion -----------------------------------------------------------
% Selected hardware:
%   FT Power Pack B Radial v2
%   FT Radial 2212B, 1050 kV; FT 25 A ESC; HQ 9x4.5 Standard CCW propeller.
%
% There is no motor/prop dynamometer point, and the ESC current rating is not
% shaft power.  Instead, choose the effective power required to balance cruise
% drag:
%
%     P_eff = D*V = (W/(L/D))*V
%
% with L/D = 8.45 at the cruise condition.  P_eff is useful
% power delivered to the air, not battery electrical power or motor shaft
% power.  The 1 kg thrust cap is a numerical low-speed guard based on the
% manufacturer's stated 4S capability; it is inactive near the 13 m/s point.
FTX_PROP_MOTOR = 'FT Radial 2212B 1050 kV';
FTX_PROP_PROP  = 'HQ 9x4.5 Standard CCW';
FTX_PROP_LD    = 8.45;
FTX_PROP_POWER = (mass*9.80665/FTX_PROP_LD) * FTX_V; % W, effective
FTX_PROP_VMIN  = 0.10;                  % m/s, divide-by-zero guard only
FTX_PROP_TMAX  = 1.00*9.80665;          % N, low-speed numerical cap
FTX_PROP_P     = [FTX_PROP_POWER; FTX_PROP_VMIN; FTX_PROP_TMAX];

% The model starts level at the cruise angle of attack.  This is the attitude
% the controller holds unless a driver supplies a small test command.
FTX_THETA_HOLD = init_euler_orien(2) * 180/pi;  % deg

% The longitudinal gains carry over from the alpha-hold baseline.  The alias
% keeps pitch-attitude tuning separate from that baseline.
FTX_KTHETA0 = FTX_KA0;
Kalpha = RT_KA_SCALE * FTX_KTHETA0 * ones(8,5);

DEMAND0 = [0; FTX_THETA_HOLD; 0];       % [phi rad; theta deg; beta deg]
DEMAND1 = DEMAND0;
T_STEP  = [20; 20; 20];

fprintf(['uav_setup_v7_ftx_att: pitch-attitude hold at theta %.2f deg; ' ...
         'alpha remains a monitored envelope variable\n'], FTX_THETA_HOLD);
fprintf(['uav_setup_v7_ftx_att: %s + %s, constant effective power %.2f W ' ...
         '(T = P/V, cap %.2f N)\n'], ...
        FTX_PROP_MOTOR, FTX_PROP_PROP, FTX_PROP_POWER, FTX_PROP_TMAX);
