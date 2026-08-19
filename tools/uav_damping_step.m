function dC = uav_damping_step(alpha_deg, pqr, V, P)
%UAV_DAMPING_STEP  Aerodynamic damping increments for the FT Explorer plant.
%
%   dC = uav_damping_step(alpha_deg, pqr, V, P) returns the six body-axis
%   coefficient increments [CX CY CZ Cl Cm Cn] produced by body rates.
%
%   WHY THIS EXISTS. The HL-20 AirFrame subsystem routes its own pqr and V
%   inputs straight into Terminator blocks -- it models NO aerodynamic damping
%   at all. That is survivable on a Mach-0.6 lifting body whose normalised rates
%   stay tiny; it is not survivable here. At 13 m/s with cref 0.1849 m,
%
%       dCm = Cmq * q*cref/(2V) = -10.02 * q * 0.007111 = -0.0713 per rad/s
%
%   against a full-authority elevator worth 0.0219 per degree. Pitch damping is
%   the same order as the control itself, so omitting it would make this
%   aircraft look far less damped than it is and would corrupt any gain tuned
%   against it.
%
%   CONVENTIONS, matching the rest of the plant:
%     alpha_deg  degrees          (the same signal the coefficient tables index)
%     pqr        rad/s, body axes (straight off the 6DOF block)
%     V          m/s, true airspeed
%     dC         [CX CY CZ Cl Cm Cn], body axes x fwd / y right / z down
%
%   Rate derivatives are per RADIAN of normalised rate, which is what the
%   VSPAero stability run reports -- unlike the CONTROL derivatives in the same
%   table file, which are per DEGREE of deflection. Confusing the two scales the
%   damping by 57.
%
%   STATELESS BY CONSTRUCTION. Like uav_sensors_step and uav_estimator_step,
%   this holds no persistent state. A `persistent` inside a function called from
%   a MATLAB Function block is not Simulink-managed state and advances on every
%   solver evaluation rather than once per sample -- the bug that scored 8 deg
%   offline and 69 deg in-model.
%
%   FIXED-SIZE BY CONSTRUCTION, TOO. Everything below indexes P directly rather
%   than slicing it. Slicing with a computed range (`P(3 : 2+n)`) makes the
%   intermediates variable-size, and a variable-size signal propagates out of
%   the block and is rejected downstream by the Aerodynamic Forces and Moments
%   block with a dimensions error that points at *its* internals rather than
%   here. Keep the indexing literal.
%
%   P is packed by uav_setup_v7_ftx.m, which asserts this layout:
%     P(1)              bref [m]
%     P(2)              cref [m]
%     P(3     .. 2+N)   alpha_vec_damping [deg]
%     P(2+1*N .. 2+2*N) Clp     then, in order, at successive N-blocks:
%                       Clr Cmq Cnp Cnr CYp CYr CZq

%#codegen

% Number of points in the damping alpha grid. Fixed at build time so every
% index below is a compile-time constant. uav_setup_v7_ftx.m asserts that
% numel(alpha_vec_damping) still matches; if the stability run is ever re-gridded
% that assert fires rather than this silently reading the wrong offsets.
N = 9;

bref = P(1);
cref = P(2);

% Interpolation bracket on the alpha grid, computed once and reused for all
% eight derivatives. Ends are held, matching what the plant's Lookup and
% Prelookup blocks do outside their breakpoint range.
k = 1;
t = 0;
if alpha_deg <= P(3)
    k = 1;
    t = 0;
elseif alpha_deg >= P(2+N)
    k = N - 1;
    t = 1;
else
    for i = 1:N-1
        if alpha_deg >= P(2+i) && alpha_deg <= P(2+i+1)
            k = i;
            t = (alpha_deg - P(2+i)) / (P(2+i+1) - P(2+i));
            break
        end
    end
end

Clp = pick(P, 1*N, k, t);
Clr = pick(P, 2*N, k, t);
Cmq = pick(P, 3*N, k, t);
Cnp = pick(P, 4*N, k, t);
Cnr = pick(P, 5*N, k, t);
CYp = pick(P, 6*N, k, t);
CYr = pick(P, 7*N, k, t);
CZq = pick(P, 8*N, k, t);

% Guard the normalisation. Below a few m/s the aircraft is not flying and the
% b/2V factor runs away; clamping is honest here because the coefficient set has
% no meaning at those speeds anyway.
Vs = max(V, 1.0);

phat = pqr(1) * bref / (2*Vs);
qhat = pqr(2) * cref / (2*Vs);
rhat = pqr(3) * bref / (2*Vs);

dC = zeros(6,1);
dC(2) = CYp*phat + CYr*rhat;
dC(3) = CZq*qhat;
dC(4) = Clp*phat + Clr*rhat;
dC(5) = Cmq*qhat;
dC(6) = Cnp*phat + Cnr*rhat;
% dC(1): the stability run reports no CXq, and axial rate damping is negligible.
end


function y = pick(P, base, k, t)
%PICK  Linearly interpolate the derivative block starting at offset 2+base.
%#codegen
a = P(2 + base + k);
b = P(2 + base + k + 1);
y = a + t * (b - a);
end
