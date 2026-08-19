%% uav_setup_v7 - base workspace for DroneModelv7
%
% Populates the base workspace variables the model needs to load and run.
% Run this, then sim('DroneModelv7').

%% ---- aero / body / actuator data from DroneModelv7/data ----
load body_data.mat        % CX0 CZ0 Cm0 Cn0 Inertia CYbeta Clbeta alpha_vec_0 beta_vec_0 ...
load actuator_data.mat    % CXda CXde CYda CZda CZde Clda Cmda Cmde Cnda
load more_data.mat        % Clp Clr Cmq Cnp Cnr alpha_vec_damping bref d_ref

%% ---- geometry, mass, initial conditions ----
Sref  = 26.612;
b_ref = 4.2337;
dref  = 8.6076;
x_cg  = -4.7864;
y_cg  = 0;
z_cg  = 0.054;

init_inertia      = eye(3);
init_pos_inertial = [-12072; 0; -3048];
init_velo_body    = [202.67; 0; 9.626];
init_euler_orien  = [0; 0.19945; 0];
init_body_rot     = [0; 0; 0];
mass              = 11739;

d2r      = pi/180;
m2ft     = 3.28084;
Altitude = 10000/m2ft;
Mach     = 0.6;

%% ---- trim grid ----
alpha_vec = -10:5:25;    % 8 points
beta_vec  = -10:5:10;    % 5 points
[alpha, beta] = ndgrid(alpha_vec, beta_vec);

%% ---- controller prelookup breakpoints ----
% The Controller's Prelookup/Prelookup1 blocks take a_vec and b_vec. The gain
% tables are 8x5 and the trim grid is 8 alpha by 5 beta, so they are the same
% vectors.
a_vec = alpha_vec;
b_vec = beta_vec;

%% ---- gain schedules ----
% 8x5 tables, alpha down and beta across. They vary smoothly in alpha and are
% nearly flat in beta; the transpose does not have that property.
run(fullfile(fileparts(mfilename('fullpath')), '..', 'out', 'gain_tables.m'));

Kp     = Kp_1;        % rate-feedback gain, roll rate p
Kq     = Kq_1;        % rate-feedback gain, pitch rate q
Kr     = Kr_1;        % rate-feedback gain, yaw rate r
Kalpha = K_Alpha_1;   % integral gain, alpha channel   ("I alpha" block)
Kbeta  = K_Beta_1;    % integral gain, beta channel    ("I Beta" block)
Kphi   = K_Phi_1;     % integral gain, phi channel     ("I Phi" block)

% The P gains are not lookups. The Controller's "P alpha", "P beta" and
% "P phi" MATLAB Function blocks return constants:
%   P alpha ->  0.05
%   P beta  -> -0.05
%   P phi   ->  0.05

%% ---- controller sample time ----
% The PI controllers in hdl/HDLDroneModel3.v are gated by an enable named
% enb_1_20000000_1, asserting once every 20,000,000 cycles of the 1e-08 s
% clock in the same header: a 100 MHz clock and a 5 Hz control update. The
% clock rate is not the loop rate.
Tc = 0.2;

fprintf('uav_setup_v7: workspace ready. Tc = %g s (%.4g Hz control loop)\n', Tc, 1/Tc);
