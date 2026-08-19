%% uav_setup_v7_ftx - workspace for DroneModelv7_FTX (the FT Explorer plant)
%
% Runs uav_setup_v7_rt first (gains, limiters, estimator, rates), then REPLACES
% every plant parameter with the FT Explorer sport wing: geometry, mass,
% inertia, flight condition, and the full three-surface low-speed coefficient
% set from airframe/aero/.
%
% Kept separate from uav_setup_v7_rt.m so the plant substitution stays visible
% in one file. The plant is the FT Explorer V1/MKR2 official sport wing; the
% coefficient set is built by airframe/aero/build_tables.py and the inertia by
% airframe/shell_inertia.py.
%
% *** VALIDITY BAND ***
%   alpha -8..+8 deg, beta -10..+10 deg.  The coefficient set is a linear
%   vortex-lattice solution with NO STALL and is meaningless outside it.
%   Commands and validation cases must remain inside this band.

thisDir = fileparts(mfilename('fullpath'));
run(fullfile(thisDir,'uav_setup_v7_rt.m'));

wb_ = fileparts(thisDir);

%% ---- drop the HL-20 plant --------------------------------------------------
% uav_setup_v7 loaded body_data/actuator_data/more_data. Those variables share
% names with the replacement set (CX0, Cmde, Clp, ...), so a survivor would be
% invisible. Clear them, then assert below that everything the model needs came
% back with FT Explorer dimensions.
clear CX0 CY0 CZ0 Cl0 Cm0 Cn0 CYbeta Clbeta Inertia
clear alpha_vec_0 beta_vec_0 alpha_vec_Cn0 beta_vec_Cn0
clear CXda CYda CZda Clda Cmda Cnda CXde CYde CZde Clde Cmde Cnde
clear Clp Clr Cmq Cnp Cnr alpha_vec_damping bref d_ref dref Sref b_ref
clear alpha_vec beta_vec

%% ---- the FT Explorer coefficient set ---------------------------------------
FTX_TABLES = fullfile(wb_,'airframe','aero','tables','ft_explorer_aero_tables.mat');
if ~exist(FTX_TABLES,'file')
    error('uav_setup_v7_ftx:noTables', ...
        ['FT Explorer coefficient set not found at\n  %s\n' ...
         'Regenerate it with airframe/aero/run_cases.py followed by ' ...
         'airframe/aero/build_tables.py.'], FTX_TABLES);
end
load(FTX_TABLES);   % CX0 CY0 CZ0 Cl0 Cm0 Cn0, alpha_vec beta_vec,
                    % CXda..Cndr, alpha_vec_control, damping + alpha_vec_damping,
                    % Sref bref cref mass_nominal

%% ---- damping parameter vector ----------------------------------------------
% The HL-20 AirFrame terminates its own pqr and V inputs and carries NO
% aerodynamic damping. build_ftx_v7.m adds a Damping Increments block that uses
% them; this is the parameter vector it reads, packed the way SENS_P and EST_P
% are packed for the estimator.
%
% Rate derivatives are per RADIAN of normalised rate (p*b/2V), which is the
% convention the VSPAero stability run reports. The CONTROL derivatives in the
% same table file are per DEGREE of deflection. Confusing the two scales the
% damping by 57.
%
% Order must match uav_damping_step.m: bref, cref, alpha grid, then
% Clp Clr Cmq Cnp Cnr CYp CYr CZq.
DMP_P = [ bref; cref; alpha_vec_damping(:)
          Clp(:); Clr(:); Cmq(:); Cnp(:); Cnr(:); CYp(:); CYr(:); CZq(:) ];

% uav_damping_step.m hard-codes N = 9 so that every index into DMP_P is a
% compile-time constant -- a computed slice makes the block's output
% variable-size, which the Aerodynamic Forces and Moments block then rejects
% with a dimensions error pointing at its own internals. If the stability run is
% ever re-gridded, this assert fires instead of the offsets silently shifting.
assert(numel(alpha_vec_damping) == 9, ...
    ['uav_setup_v7_ftx: damping grid is %d points, but uav_damping_step.m ' ...
     'is compiled for 9. Update N there to match.'], numel(alpha_vec_damping));
assert(numel(DMP_P) == 2 + 9*numel(alpha_vec_damping), ...
    'uav_setup_v7_ftx: DMP_P is mis-packed');

%% ---- geometry --------------------------------------------------------------
% Sref, bref and cref come straight out of the table file, so they cannot drift
% away from the condition the coefficients were solved at.
%
% NOTE DroneModelv7_RT passes `dref` into the Aerodynamic Forces and Moments
% block's `cbar` port -- 8.6076 m on the HL-20, a body reference length.
% build_ftx_v7.m repoints that mask parameter at `cref`, because the new
% pitching-moment coefficients are non-dimensionalised on the mean chord.
dref  = cref;      % kept defined: several scripts still reference it
b_ref = bref;

%% ---- mass and inertia ------------------------------------------------------
% From airframe/shell_inertia.py. Distributed foam-board shell model, calibrated
% to the same 334 g structural residual the BOM carries, with the CG held on the
% supplier-specified 57 mm because the aero tables were generated at it.
%
% This is an inertia MATRIX, not a list of products of inertia: the off-diagonal
% entries are the negated products. OpenVSP reports the products, and getting
% that backwards yields a plausible, wrong tensor.
mass = mass_nominal;                    % 0.8765 kg

FTX_PXZ = 3.058312e-03;                 % product of inertia, kg m^2
Inertia = [ 3.667119e-02   0            -FTX_PXZ
            0              3.112778e-02  0
           -FTX_PXZ        0             6.548569e-02 ];

% Iyy sensitivity scale. A value of 1.0 selects the nominal shell-model value.
FTX_IYY_SCALE = 1.0;
Inertia(2,2)  = FTX_IYY_SCALE * Inertia(2,2);

%% ---- flight condition ------------------------------------------------------
% Sea level, 13 m/s. This places the chord Reynolds number near the middle of
% the modelled 120,000-220,000 range.
FTX_V     = 13.0;                       % m/s true airspeed
Mach      = FTX_V / 340.294;            % 0.0382

% Release altitude. The AirFrame subsystem has NO PROPULSION -- it takes three
% deflections and nothing else -- so it flies as a glider and descends
% throughout any run. Starting at sea level puts
% it below 0 m within a second, where the COESA model extrapolates and the state
% derivative goes non-finite. 200 m clears a 60 s glide (about 92 m at L/D 8.45)
% with margin.
%
% The coefficient set is solved at sea-level ISA. Over a 200 m descent density
% varies by under 2%, which moves trim alpha by about 0.03 deg -- far inside the
% set's own uncertainty. Stated rather than silently absorbed.
FTX_ALT   = 200;                        % m
Altitude  = FTX_ALT;

% Trim solved from the coefficient tables: the alpha that produces the
% CL required to hold 0.8765 kg at 13 m/s, and the elevator that zeroes Cm there.
FTX_ALPHA_TRIM = 1.60;                  % deg
FTX_DE_TRIM    = -0.56;                 % deg

% Release level at the trim alpha (gamma = 0, so theta = alpha). With no thrust,
% the aircraft transitions from this initial condition into a glide.
init_velo_body    = FTX_V * [cosd(FTX_ALPHA_TRIM); 0; sind(FTX_ALPHA_TRIM)];
init_euler_orien  = [0; FTX_ALPHA_TRIM*pi/180; 0];
init_body_rot     = [0; 0; 0];
init_pos_inertial = [0; 0; -FTX_ALT];   % z is DOWN, so this is FTX_ALT up
init_inertia      = eye(3);

x_cg = 0; y_cg = 0; z_cg = 0;           % moments are already about the 57 mm CG

%% ---- control rate ----------------------------------------------------------
% The plant short-period mode is 17.22 rad/s (2.74 Hz). A 50 Hz controller
% keeps that mode and the inner rate loops well below the sampling frequency.
Tc = 0.02;

%% ---- re-tuned gains --------------------------------------------------------
% The baseline gain tables are systune output against an 11,739 kg lifting body
% at Mach 0.6 and carry no meaning here. These replace them outright.
%
% FLAT, NOT SCHEDULED, and the data says so. Over the validated band the new
% control derivatives vary by 1.4% (Cm_de), 2.5% (Cl_da) and 10.3% (Cn_dr),
% against 142% and 99% for the HL-20's Cmde and Clda over its grid. The 40-point
% schedule exists because that airframe needs it; this one does not. The
% schedule's alpha axis also runs -10..25 deg, most of which is outside the
% band where these coefficients mean anything.
%
% Baselines are computed from the plant rather than typed in, so they follow the
% tables if those are ever regenerated.

rho_alt = 1.225 * (1 - 2.25577e-5*FTX_ALT)^4.25588;
qbar_0  = 0.5 * rho_alt * FTX_V^2;
ia      = @(v) interp1(alpha_vec_control(:), v(:), FTX_ALPHA_TRIM);
id      = @(v) interp1(alpha_vec_damping(:), v(:), FTX_ALPHA_TRIM);

% Angular acceleration per degree of surface, and per rad/s of own-axis rate.
qd_de = qbar_0*Sref*cref*ia(Cmde) / Inertia(2,2);
qd_q  = qbar_0*Sref*cref*id(Cmq)*cref/(2*FTX_V) / Inertia(2,2);
pd_da = qbar_0*Sref*bref*ia(Clda) / Inertia(1,1);
pd_p  = qbar_0*Sref*bref*id(Clp)*bref/(2*FTX_V) / Inertia(1,1);
rd_dr = qbar_0*Sref*bref*ia(Cndr) / Inertia(3,3);
rd_r  = qbar_0*Sref*bref*id(Cnr)*bref/(2*FTX_V) / Inertia(3,3);

% Inner rate loops. ACT_MAP is -I, so the surface command is K*(cmd - rate) and
% the closed rate loop is  d(rate)/dt = (rate_d + surf_d*K)*rate + ...,
% giving a first-order bandwidth of -(rate_d + surf_d*K). Solve for K.
% Bandwidth parameters keep the gains tied to the aerodynamic tables and
% inertia. They reproduce Kq -2.50416, Kp +0.30450 and Kr +9.01335.
FTX_WQ = 20.403;   % rad/s, pitch-rate loop (open-loop pitch damping 11.54)
FTX_WP = 35.000;   % rad/s, roll-rate loop  (open-loop roll subsidence 32.11)
FTX_WR =  8.451;   % rad/s, yaw-rate loop   (open-loop yaw damping 2.14)

% The roll-loop bandwidth is limited to approximately one tenth of the sample
% rate to preserve sampled-data margin.

FTX_KQ0 = (FTX_WQ + qd_q) / qd_de;    % negative: Cm_de < 0
FTX_KP0 = (FTX_WP + pd_p) / pd_da;    % positive: Cl_da > 0
FTX_KR0 = (FTX_WR + rd_r) / rd_dr;    % positive: Cn_dr > 0

% Outer attitude loops. P_alpha = 0.05 survives untouched, and it is worth
% saying why rather than leaving it looking like an oversight: alpha is compared
% in DEGREES while the rate command is in rad/s, so 0.05 is 2.87 rad/s of
% bandwidth -- an order below the pitch-rate loop, which is exactly where an
% outer loop belongs. That reasoning transfers to this aircraft unchanged.
%
% P_phi does NOT transfer, and for a unit reason. Phi is compared in RADIANS
% (uav_setup_v7_cl.m records the split), so 0.05 there is 0.05 rad/s of
% bandwidth -- a 20 second roll-attitude response. It needs ~5 to sit an order
% below the roll-rate loop, hence the scale of 100 below.
RT_PA_SCALE = 1.0;      % alpha, 0.05 -> 0.05   [rad/s per deg]
RT_PB_SCALE = 1.0;      % beta,  -0.05
RT_PP_SCALE = 100.0;    % phi,   0.05 -> 5.0    [rad/s per rad]

% Flat integral gains place the PI zero below the outer-loop bandwidth.
FTX_KA0  =  0.125;
FTX_KB0  = -0.0625;
FTX_KPH0 =  5.00;   % P_phi is 5.0 after RT_PP_SCALE, so this is also Ti ~ 1 s

% Gain scale factors. All 1.0 selects the nominal values above.
RT_KQ_SCALE = 1.0;  RT_KP_SCALE = 1.0;  RT_KR_SCALE = 1.0;
RT_KA_SCALE = 1.0;  RT_KB_SCALE = 1.0;  RT_KPH_SCALE = 1.0;

one85 = ones(8,5);
Kq     = RT_KQ_SCALE  * FTX_KQ0  * one85;
Kp     = RT_KP_SCALE  * FTX_KP0  * one85;
Kr     = RT_KR_SCALE  * FTX_KR0  * one85;
Kalpha = RT_KA_SCALE  * FTX_KA0  * one85;
Kbeta  = RT_KB_SCALE  * FTX_KB0  * one85;
Kphi   = RT_KPH_SCALE * FTX_KPH0 * one85;

% PI output limits are rate commands in rad/s. The band is +-8 deg of alpha and
% the outer loop runs at 2.87 rad/s, so a full-band error asks for about
% 0.4 rad/s; 2.0 bounds windup well clear of normal operation.
RT_LIM_ALPHA = 2.0;
RT_LIM_BETA  = 2.0;
RT_LIM_PHI   = 2.0;

%% ---- controller prelookup grid ---------------------------------------------
% The gain schedule is tabulated over the HL-20 trim grid (alpha -10:5:25,
% beta -10:5:10). The new aero is valid only over -8..+8, so the schedule's
% alpha axis runs well outside the plant's validity. Left as-is here, because
% re-gridding the schedule is a re-tuning decision rather than a plant one.
a_vec = -10:5:25;    % the controller's gain-schedule grid, NOT the aero grid
b_vec = -10:5:10;

%% ---- actuator mapping ------------------------------------------------------
% The HL-20 plant has two inputs and three control channels, which is why
% ACT_MAP is 2x3 there and the lateral mapping is undecidable. THIS airframe has
% all three surfaces, so the mapping is one-to-one and the ambiguity is gone:
%
%   u_swm(1) roll  -> aileron    (Cl_da = +0.00891 /deg)
%   u_swm(2) pitch -> elevator   (Cm_de = -0.02191 /deg)
%   u_swm(3) yaw   -> rudder     (Cn_dr = +0.00115 /deg)  <- no HL-20 counterpart
%
% Each channel computes rate minus rate command. Negative mapping signs produce
% negative feedback with the surface derivative conventions used here.
ACT_MAP_SIGN = -1;
FTX_YAW_SIGN = -1;

ACT_MAP = [ ACT_MAP_SIGN  0             0
            0             ACT_MAP_SIGN  0
            0             0             FTX_YAW_SIGN ];

% Surface travel. The supplier specifies 12 deg on all three surfaces; the
% +-25 deg limit in DroneModelv7_RT is sized for the HL-20's weaker derivatives.
% Trim needs 0.56 deg of elevator against that 12 deg throw.
RT_SURF_LIM = 12;

%% ---- estimator, re-referenced to this aircraft -----------------------------
% Three estimator parameters are plant properties and carry HL-20 values in
% uav_setup_v7_est.m. Everything else there is sensor hardware and stands.
EST_RHO = 1.225;            % kg/m^3 at sea level (HL-20: 0.9046 at 10,000 ft)

% The sideslip channel inverts m*a_y = qbar*Sref*CY_beta*beta. CY_beta here is
% the static-grid value, per DEGREE, matching the EST_CYBETA convention.
EST_CYBETA = -0.004627;     % /deg (HL-20: -0.01242)

% GPS vertical-velocity noise. Alpha reconstruction divides vertical velocity
% by airspeed, making this source important for the low-speed plant.
VZ_SIG = 0.10;              % m/s, GPS vertical velocity (barometric: 0.5)

% EST_P carries the pre-flight alignment and sample time. Match the estimator
% alignment to the plant before rebuilding its parameter vector.
EST_EULER0 = init_euler_orien(:);

% SENS_P carries VZ_SIG, so rebuild both parameter vectors together.
uav_set_est_rate(EST_TS);
assert(norm(EST_P(10:12)-EST_EULER0) < 1e-12, ...
    'uav_setup_v7_ftx: estimator alignment does not match plant initial attitude');

%% ---- command vector --------------------------------------------------------
% Hold the trim condition by default.
DEMAND0 = [0; FTX_ALPHA_TRIM; 0];       % [phi rad; alpha deg; beta deg]
DEMAND1 = [0; FTX_ALPHA_TRIM; 0];
T_STEP  = [20; 20; 20];

%% ---- assert the plant is actually the FT Explorer --------------------------
% Cheap, and it catches the failure mode this file exists to prevent: an HL-20
% table surviving the clear above and being silently used. Shapes differ, so a
% survivor is caught here rather than becoming a plausible wrong result.
assert(isequal(size(CX0),[17 21]) && isequal(size(Cn0),[17 21]), ...
    'uav_setup_v7_ftx: static tables are not 17x21 -- HL-20 data may have survived');
assert(numel(alpha_vec)==17 && alpha_vec(1)==-8 && alpha_vec(end)==8, ...
    'uav_setup_v7_ftx: alpha_vec is not the validated -8..+8 band');
assert(numel(beta_vec)==21 && beta_vec(1)==-10 && beta_vec(end)==10, ...
    'uav_setup_v7_ftx: beta_vec is not the validated -10..+10 band');
assert(exist('Cndr','var')==1 && exist('Cldr','var')==1, ...
    'uav_setup_v7_ftx: no rudder derivatives -- this is not the three-surface set');
assert(abs(mass-0.8765) < 1e-9, 'uav_setup_v7_ftx: mass is not the FT Explorer value');
assert(abs(Sref-0.267696) < 1e-5, 'uav_setup_v7_ftx: Sref is not the FT Explorer value');

fprintf(['uav_setup_v7_ftx: FT Explorer plant. mass %.4f kg, Sref %.6f m^2, ' ...
         'bref %.5f m, cref %.6f m\n'], mass, Sref, bref, cref);
fprintf(['uav_setup_v7_ftx: %.1f m/s at %g m (Mach %.4f), trim alpha %+.2f deg, ' ...
         'de %+.2f deg\n'], FTX_V, FTX_ALT, Mach, FTX_ALPHA_TRIM, FTX_DE_TRIM);
fprintf('uav_setup_v7_ftx: Inertia diag [%.4e %.4e %.4e] kg m^2, Pxz %.4e\n', ...
        Inertia(1,1), Inertia(2,2), Inertia(3,3), -Inertia(1,3));
fprintf('uav_setup_v7_ftx: surface limit +-%g deg, band alpha [%g %g] beta [%g %g]\n', ...
        RT_SURF_LIM, alpha_vec(1), alpha_vec(end), beta_vec(1), beta_vec(end));
fprintf(['uav_setup_v7_ftx: Tc = %g s (%g Hz control), rate loops [q %g  p %g  r %g] rad/s\n' ...
         'uav_setup_v7_ftx: flat gains Kq %.4g  Kp %.4g  Kr %.4g  (HL-20: -279..-174, 51..76, -1155..-714)\n'], ...
        Tc, 1/Tc, FTX_WQ, FTX_WP, FTX_WR, Kq(1), Kp(1), Kr(1));
