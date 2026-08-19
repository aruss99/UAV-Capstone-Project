function verify_ftx_aero(outFile)
%VERIFY_FTX_AERO  Validate the FT Explorer aerodynamic table integration.
%
%   Drives the Aerodynamic Coefficients subsystem with known alpha, beta,
%   surface deflections and rates, then compares its six body-axis coefficients
%   with an independent evaluation of the CSV tables in airframe/aero/tables/.
%
%   Checks table orientation, axis order, surface mapping, derivative parity
%   and per-degree versus per-radian damping scale.
%
%   Usage:  verify_ftx_aero
%           verify_ftx_aero('<workbench>/out/FTX_AERO_VERIFY.txt')

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(outFile), outFile = fullfile(wb,'out','FTX_AERO_VERIFY.txt'); end
toolsDir = fullfile(wb,'tools');
mdl  = 'DroneModelv7_FTX';
mdlF = fullfile(wb,'simulink',mdl,[mdl '.slx']);

addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(fullfile(wb,'simulink',mdl))); addpath(toolsDir);
evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_ftx.m')));

T = load(fullfile(wb,'airframe','aero','tables','ft_explorer_aero_tables.mat'));

%% ---- build a harness holding only the coefficient subsystem ---------------
harness = 'ftx_aero_harness';
try, close_system(harness,0); catch, end
load_system(mdlF);
new_system(harness); load_system(harness);
add_block([mdl '/AirFrame/Aerodynamic Coefficients'], [harness '/AC']);
close_system(mdl,0);

nIn = numel(get_param([harness '/AC'],'PortHandles').Inport);
for k = 1:nIn
    add_block('simulink/Sources/Constant', sprintf('%s/u%d',harness,k), ...
        'Value', sprintf('U%d',k), 'Position',[40 40+60*k 90 68+60*k]);
    add_line(harness, sprintf('u%d/1',k), sprintf('AC/%d',k), 'autorouting','on');
end
add_block('simulink/Sinks/Out1', [harness '/C'], 'Port','1', ...
    'Position',[400 100 430 118]);
add_line(harness, 'AC/1', 'C/1', 'autorouting','on');
set_param(harness, 'SolverType','Fixed-step','Solver','ode1','FixedStep','0.01', ...
                   'StopTime','0.01','SaveOutput','on','SaveFormat','Array', ...
                   'UnconnectedInputMsg','error');

%% ---- test points ----------------------------------------------------------
% Chosen to exercise each mechanism separately: the static grid off-centre in
% BOTH axes (so a transpose cannot pass), each surface alone, and a pure rate
% case for the damping path.
cases = {
%   name                      alpha  beta   da    de    dr    p     q     r
    'trim, clean'              1.60   0.00  0.0   0.0   0.0   0.0   0.0   0.0
    'static, off-grid both'    3.40  -4.20  0.0   0.0   0.0   0.0   0.0   0.0
    'static, alpha only'      -6.00   0.00  0.0   0.0   0.0   0.0   0.0   0.0
    'static, beta only'        0.00   7.00  0.0   0.0   0.0   0.0   0.0   0.0
    'aileron +8'               2.00   0.00  8.0   0.0   0.0   0.0   0.0   0.0
    'aileron -8 (parity)'      2.00   0.00 -8.0   0.0   0.0   0.0   0.0   0.0
    'elevator -5'              2.00   0.00  0.0  -5.0   0.0   0.0   0.0   0.0
    'rudder +6'                0.00   3.00  0.0   0.0   6.0   0.0   0.0   0.0
    'pitch rate q = 0.5'       1.60   0.00  0.0   0.0   0.0   0.0   0.5   0.0
    'roll rate p = 1.0'        1.60   0.00  0.0   0.0   0.0   1.0   0.0   0.0
    'yaw rate r = 0.3'         1.60   2.00  0.0   0.0   0.0   0.0   0.0   0.3
    'combined'                 4.00  -3.00  6.0  -4.0   5.0   0.4  -0.3   0.2
};

lbl = {'CX','CY','CZ','Cl','Cm','Cn'};
nf = 0; lines = {};
lines{end+1} = 'FT Explorer aero integration - model vs tables';
lines{end+1} = '==================================================================';
lines{end+1} = sprintf('Generated %s by tools/verify_ftx_aero.m', datestr(now,'yyyy-mm-dd HH:MM'));
lines{end+1} = '';
lines{end+1} = 'Model = DroneModelv7_FTX/AirFrame/Aerodynamic Coefficients, driven directly.';
lines{end+1} = 'Table = independent evaluation of airframe/aero/tables/ in this script.';
lines{end+1} = '';

for c = 1:size(cases,1)
    nm = cases{c,1};
    al = cases{c,2}; be = cases{c,3};
    da = cases{c,4}; de = cases{c,5}; dr = cases{c,6};
    pq = [cases{c,7}; cases{c,8}; cases{c,9}];

    % The subsystem takes alpha and beta in RADIANS (its alpha_rad/beta_rad
    % blocks convert), so feed radians and let the model do its own conversion.
    assignin('base','U1', al*pi/180);
    assignin('base','U2', be*pi/180);
    assignin('base','U3', da);
    assignin('base','U4', de);
    assignin('base','U5', pq);
    assignin('base','U6', evalin('base','FTX_V'));
    assignin('base','U7', dr);

    so = sim(harness, 'ReturnWorkspaceOutputs','on');
    got = so.yout(end,:);
    got = got(:).';

    want = tableCoeffs(T, al, be, da, de, dr, pq, evalin('base','FTX_V'));

    lines{end+1} = sprintf('%-26s alpha %+6.2f  beta %+6.2f  da %+5.1f de %+5.1f dr %+5.1f', ...
        nm, al, be, da, de, dr);
    ok = true;
    for i = 1:6
        d = got(i) - want(i);
        tol = 1e-9 + 1e-6*max(abs(want(i)),1e-4);
        flag = '   ';
        if abs(d) > tol, flag = ' <<'; ok = false; end
        lines{end+1} = sprintf('     %-3s model %+14.8e   table %+14.8e   d %+9.2e%s', ...
            lbl{i}, got(i), want(i), d, flag);
    end
    if ok
        lines{end+1} = '     [PASS]';
    else
        lines{end+1} = '     [FAIL]';
        nf = nf + 1;
    end
    lines{end+1} = '';
end

lines{end+1} = '==================================================================';
if nf == 0
    lines{end+1} = 'RESULT: all cases match. The tables are wired in correctly.';
else
    lines{end+1} = sprintf('RESULT: %d of %d cases FAILED.', nf, size(cases,1));
end

txt = strjoin(lines, newline);
if ~exist(fileparts(outFile),'dir'), mkdir(fileparts(outFile)); end
fid = fopen(outFile,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
fprintf('%s\n', txt);
fprintf('\nwritten to %s\n', outFile);

try, close_system(harness,0); catch, end
if nf > 0
    error('verify_ftx_aero:mismatch','%d case(s) disagree with the tables', nf);
end
end


% ===========================================================================
function C = tableCoeffs(T, al, be, da, de, dr, pqr, V)
%TABLECOEFFS  Independent evaluation of the coefficient set.
%   Deliberately written from the coefficient set's stated conventions rather
%   than by reading the model, so agreement means something.

% --- static, bilinear on (alpha, beta), tables stored alpha-major ---
C = zeros(1,6);
st = {T.CX0, T.CY0, T.CZ0, T.Cl0, T.Cm0, T.Cn0};
for i = 1:6
    C(i) = bilin(T.alpha_vec, T.beta_vec, st{i}, al, be);
end

% --- control increments -----------------------------------------------------
% Per degree, tabulated against alpha alone. CX/CZ/Cm of the ANTISYMMETRIC
% surfaces (aileron, rudder) multiply |delta|; everything else multiplies
% signed delta. The elevator carries no CY/Cl/Cn in the plant.
ac = T.alpha_vec_control;
aq = min(max(al, ac(1)), ac(end));   % the plant clamps to the table range

ail = {T.CXda, T.CYda, T.CZda, T.Clda, T.Cmda, T.Cnda};
rud = {T.CXdr, T.CYdr, T.CZdr, T.Cldr, T.Cmdr, T.Cndr};
elv = {T.CXde, [],     T.CZde, [],     T.Cmde, []    };

absMask = [true false true false true false];   % CX CZ Cm take |delta|
for i = 1:6
    ma = lin(ac, ail{i}, aq);
    mr = lin(ac, rud{i}, aq);
    if absMask(i)
        C(i) = C(i) + ma*abs(da) + mr*abs(dr);
    else
        C(i) = C(i) + ma*da + mr*dr;
    end
    if ~isempty(elv{i})
        C(i) = C(i) + lin(ac, elv{i}, aq)*de;
    end
end

% --- damping ----------------------------------------------------------------
% Per RADIAN of normalised rate.
ad = T.alpha_vec_damping;
Vs = max(V,1.0);
phat = pqr(1)*T.bref/(2*Vs);
qhat = pqr(2)*T.cref/(2*Vs);
rhat = pqr(3)*T.bref/(2*Vs);
C(2) = C(2) + lin(ad,T.CYp,al)*phat + lin(ad,T.CYr,al)*rhat;
C(3) = C(3) + lin(ad,T.CZq,al)*qhat;
C(4) = C(4) + lin(ad,T.Clp,al)*phat + lin(ad,T.Clr,al)*rhat;
C(5) = C(5) + lin(ad,T.Cmq,al)*qhat;
C(6) = C(6) + lin(ad,T.Cnp,al)*phat + lin(ad,T.Cnr,al)*rhat;
end

function y = lin(x, v, xq)
xq = min(max(xq, x(1)), x(end));
y = interp1(x(:), v(:), xq, 'linear');
end

function z = bilin(xv, yv, Z, xq, yq)
xq = min(max(xq, xv(1)), xv(end));
yq = min(max(yq, yv(1)), yv(end));
z = interp2(yv(:).', xv(:), Z, yq, xq, 'linear');
end
