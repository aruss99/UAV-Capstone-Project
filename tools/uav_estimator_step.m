function [pqr_hat, alpha_hat, beta_hat, euler_hat, s_out] = uav_estimator_step(gyro_m, acc_m, V_m, Vz_m, P, s_in)
%UAV_ESTIMATOR_STEP  Quaternion complementary (Mahony) attitude estimator with
%   gyro bias estimation, fixed-wing accelerometer compensation, and
%   airspeed-aided angle-of-attack reconstruction.
%
%   All filter state is passed in as s_in and returned as s_out; the caller
%   holds it in a Unit Delay. This makes the update rate explicit and keeps the
%   block HDL-targetable.
%
%   Sensor models live in uav_sensors_step.m and are simulation-only.
%
%   A complementary filter avoids EKF covariance propagation. The iCE40HX1K
%   has no DSP blocks, and the 50 Hz control loop provides 240,000 cycles per
%   update from the board's 12 MHz clock.
%
%   Design constraints:
%
%   1. Quaternion propagation avoids the Euler-rate singularity near
%      theta = +/-90 deg.
%
%   2. The AirFrame Accels output is specific force. Gravity is reconstructed
%      as f - vdot_b - omega x v_b, using airspeed for the kinematic terms.
%
%   3. A Gaussian in (||f||-g)/g softly gates accelerometer correction.
%
%   4. The filtered derivative (V - V_lp)/TAU limits pitot-noise gain through a
%      physical time constant rather than the sample interval.
%
%   5. P(10:12) supplies pre-flight attitude because this simulation starts
%      in flight, where accelerometer-only levelling is not valid.
%
%   P = [Ts; KP; KI; TAU_VD; GATE_SIG; mass; Sref; CYbeta; RHO;
%        phi0; theta0; psi0]
%
%   s = [q(1:4); bias(1:3); V_lp; alpha_hat; initialised_flag]   (10 elements)

%#codegen
Ts       = P(1);
KP       = P(2);
KI       = P(3);
TAU_VD   = P(4);
GATE_SIG = P(5);
mass     = P(6);
Sref     = P(7);
CYbeta   = P(8);
RHO      = P(9);
g        = 9.80665;

q     = s_in(1:4);
bh    = s_in(5:7);
V_lp  = s_in(8);
alp_h = s_in(9);
init  = s_in(10);

% ---- first call: align from the supplied pre-flight attitude ---------------
if init < 0.5
    phi0 = P(10); th0 = P(11); ps0 = P(12);
    cr = cos(phi0/2); sr = sin(phi0/2);
    cp = cos(th0/2);  sp = sin(th0/2);
    cy = cos(ps0/2);  sy = sin(ps0/2);
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
    q     = q/sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    bh    = zeros(3,1);
    V_lp  = V_m;
    alp_h = 0;
end

% ---- airspeed rate, rate-independent dirty derivative ----------------------
V_lp   = V_lp + (Ts/TAU_VD)*(V_m - V_lp);
Vdot_f = (V_m - V_lp)/TAU_VD;

% ---- reconstruct body velocity from airspeed + current alpha estimate ------
ca = cos(alp_h);  sa = sin(alp_h);
vb    = V_m   *[ca; 0; sa];
vbdot = Vdot_f*[ca; 0; sa];

% ---- compensate specific force to isolate gravity --------------------------
wm  = gyro_m(:);
wxv = [wm(2)*vb(3) - wm(3)*vb(2);
       wm(3)*vb(1) - wm(1)*vb(3);
       wm(1)*vb(2) - wm(2)*vb(1)];
f_c = acc_m(:) - vbdot - wxv;          % = -g_body

% ---- Mahony correction ------------------------------------------------------
w  = wm - bh;
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R  = [qw^2+qx^2-qy^2-qz^2, 2*(qx*qy+qw*qz),     2*(qx*qz-qw*qy);
      2*(qx*qy-qw*qz),     qw^2-qx^2+qy^2-qz^2, 2*(qy*qz+qw*qx);
      2*(qx*qz+qw*qy),     2*(qy*qz-qw*qx),     qw^2-qx^2-qy^2+qz^2];
v_exp = -R(:,3);

fn = sqrt(f_c(1)^2 + f_c(2)^2 + f_c(3)^2);
if fn > 1e-6
    wg = exp(-0.5*((fn - g)/(GATE_SIG*g))^2);
    fh = f_c/fn;
    e  = [fh(2)*v_exp(3) - fh(3)*v_exp(2);
          fh(3)*v_exp(1) - fh(1)*v_exp(3);
          fh(1)*v_exp(2) - fh(2)*v_exp(1)];
    w  = w  + KP*wg*e;
    bh = bh - KI*wg*e*Ts;
end

% ---- quaternion propagation -------------------------------------------------
qd = 0.5*[-q(2)*w(1) - q(3)*w(2) - q(4)*w(3);
           q(1)*w(1) + q(3)*w(3) - q(4)*w(2);
           q(1)*w(2) - q(2)*w(3) + q(4)*w(1);
           q(1)*w(3) + q(2)*w(2) - q(3)*w(1)];
q  = q + Ts*qd;
q  = q/sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);

qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R  = [qw^2+qx^2-qy^2-qz^2, 2*(qx*qy+qw*qz),     2*(qx*qz-qw*qy);
      2*(qx*qy-qw*qz),     qw^2-qx^2+qy^2-qz^2, 2*(qy*qz+qw*qx);
      2*(qx*qz+qw*qy),     2*(qy*qz-qw*qx),     qw^2-qx^2-qy^2+qz^2];

r13 = R(1,3);
if r13 >  1, r13 =  1; end
if r13 < -1, r13 = -1; end
euler_hat = [atan2(R(2,3), R(3,3)); -asin(r13); atan2(R(1,2), R(1,1))];
pqr_hat   = wm - bh;

% ---- alpha from theta - gamma ----------------------------------------------
Vs = V_m;
if Vs < 1, Vs = 1; end
sg = -Vz_m/Vs;
if sg >  1, sg =  1; end
if sg < -1, sg = -1; end
alpha_hat = euler_hat(2) - asin(sg);
alp_h     = alpha_hat;

% ---- beta by aerodynamic inversion of lateral specific force ---------------
%   m*a_y = qbar*Sref*CYbeta*beta. CYbeta = -0.01242 is small, so this is
%   hypersensitive to accelerometer bias: 0.1 m/s^2 of y-bias gives ~7.4 deg.
%   With no rudder on the aircraft, this channel is both unobservable and
%   uncontrollable.
qbar_est = 0.5*RHO*V_m^2;
den      = qbar_est*Sref*CYbeta;
if abs(den) < 1e-6
    den = -1e-6;
end
beta_hat = (mass*acc_m(2))/den;

s_out = [q; bh; V_lp; alp_h; 1];
end
