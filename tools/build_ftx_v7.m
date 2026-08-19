function outFile = build_ftx_v7(srcDir, srcModel, dstDir, dstModel)
%BUILD_FTX_V7  Put the FT Explorer plant into the re-tunable controller model.
%
%   Builds DroneModelv7_FTX from DroneModelv7_RT. The source AirFrame is the
%   Aerospace Blockset HL-20 driven by Mach-0.6 tables; this model uses the FT
%   Explorer aerodynamics, geometry, mass and inertia from airframe/aero/.
%
%   Six pieces of surgery, in dependency order:
%
%     A. Body coefficients   CX/CY/CZ/Cl/Cm/Cn all become 2-D tables on
%                            (alpha, beta). The HL-20 set has only CX/CZ/Cm as
%                            tables; CY and Cl are scalar gains on beta and Cn
%                            is a table on |beta| with the sign restored
%                            afterwards. The new set is a full signed grid, so
%                            all six become the same shape.
%     B. Control increments  Aileron and elevator retabled; a RUDDER path added,
%                            which the HL-20 plant does not have.
%     C. Damping             The HL-20 AirFrame terminates its own pqr
%                            and V inputs -- it models no aerodynamic damping at
%                            all. At 13 m/s Cmq = -10.02 contributes dCm =
%                            -0.0713 per rad/s of q, over 3x the full per-degree
%                            elevator authority, so leaving it out would make
%                            this aircraft look far less damped than it is and
%                            would corrupt any tuning done against it.
%     D. Third surface       AirFrame gains a "Rudder Deflection" inport.
%     E. Reference length    The Aerodynamic Forces and Moments block takes
%                            `dref` (8.6076 m on the HL-20) as cbar; repointed
%                            at `cref`, since the new Cm is per mean chord.
%     F. Root wiring         ACT_MAP becomes 3x3, a rudder limiter is added, and
%                            delta_r / delta_r_sat are appended as outports 16
%                            and 17. Outports 1-15 keep their RT indices,
%                            because every driver indexes them positionally.
%
%   TABLE ORIENTATION. The HL-20 tables are stored beta-major and transposed in
%   the block (Table is `CX0'`). The replacement set is stored
%   alpha-major throughout, so the transpose is removed. The shapes are 17x21
%   and non-square, so getting this wrong errors at compile rather than
%   silently transposing the aerodynamics.
%
%   Usage:  build_ftx_v7

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(srcDir),   srcDir   = fullfile(wb,'simulink','DroneModelv7_RT'); end
if nargin<2||isempty(srcModel), srcModel = 'DroneModelv7_RT';  end
if nargin<3||isempty(dstDir),   dstDir   = fullfile(wb,'simulink','DroneModelv7_FTX'); end
if nargin<4||isempty(dstModel), dstModel = 'DroneModelv7_FTX'; end

toolsDir = fullfile(wb,'tools');
addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(srcDir)); addpath(toolsDir);
if ~exist(dstDir,'dir'), mkdir(dstDir); end
outFile = fullfile(dstDir,[dstModel '.slx']);

evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_ftx.m')));

fprintf('\n=== building %s from %s ===\n', dstModel, srcModel);
try, close_system(dstModel,0); catch, end
if exist(outFile,'file'), delete(outFile); end
if ~exist(fullfile(srcDir,[srcModel '.slx']),'file')
    error('build_ftx_v7:noSource', ...
        ['%s not found. Build it first:\n' ...
         '  build_est_v7; build_rt_v7'], fullfile(srcDir,[srcModel '.slx']));
end
load_system(fullfile(srcDir,[srcModel '.slx']));
save_system(srcModel, outFile);
close_system(srcModel,0);
load_system(outFile);

root = dstModel;
af   = [root '/AirFrame'];
ac   = [af '/Aerodynamic Coefficients'];
bc   = [ac '/Body Coefficients'];
ai   = [ac '/Actuator Increments'];

%% ==== A. body coefficients ==================================================
% Breakpoints. The Prelookup pair drives CX/CZ/Cm through index+fraction ports.
set_param([bc '/Prelookup'],  'BreakpointsData','alpha_vec');
set_param([bc '/Prelookup1'], 'BreakpointsData','beta_vec');

% Drop the transpose: the new tables are stored alpha-major.
set_param([bc '/Cx'], 'Table','CX0');
set_param([bc '/Cz'], 'Table','CZ0');
set_param([bc '/Cm'], 'Table','Cm0');
fprintf('  A. CX0/CZ0/Cm0 retabled, transpose removed (new set is alpha-major)\n');

% Cn: the HL-20 path looks it up on |beta| and restores the sign afterwards,
% because its Cn0 is a 9x4 quarter-table. The new Cn0 is a full signed 17x21
% grid, so the Abs/Signum/Product apparatus is removed and beta is used directly.
cnBlk = findBlockByName(bc, 'Cn');
set_param(cnBlk, 'BreakpointsForDimension1','alpha_vec');
set_param(cnBlk, 'BreakpointsForDimension2','beta_vec');
set_param(cnBlk, 'Table','Cn0');

% Rewire Cn straight to the Mux, then delete the sign-restoration path.
delete_line_if(bc, 'Product/1', 'Mux/6');
delete_line_if(bc, [get_param(cnBlk,'Name') '/1'], 'Product/1');
delete_line_if(bc, 'Sign/1', 'Product/2');
delete_line_if(bc, 'Abs/1', [get_param(cnBlk,'Name') '/2']);
deleteLinesFrom(bc, 'Beta3');
for b = {'Product','Sign','Abs'}
    if getSimulinkBlock(bc, b{1}), delete_block([bc '/' b{1}]); end
end
add_line(bc, 'Beta3/1', [get_param(cnBlk,'Name') '/2'], 'autorouting','on');
add_line(bc, [get_param(cnBlk,'Name') '/1'], 'Mux/6', 'autorouting','on');
fprintf('  A. Cn now a full signed (alpha,beta) table; |beta|+Signum path removed\n');

% CY and Cl were scalar gains (CYbeta, Clbeta) on beta alone. The new set has
% full grids for both, so they become tables like every other coefficient.
for s = {{'Cy','CY0','Beta1'},{'Cl','Cl0','Beta2'}}
    nm = s{1}{1}; tbl = s{1}{2}; betaSrc = s{1}{3};
    pos = get_param([bc '/' nm],'Position');
    delete_line_if(bc, [betaSrc '/1'], [nm '/1']);
    delete_line_if(bc, [nm '/1'], ['Mux/' num2str(strcmp(nm,'Cl')*4 + strcmp(nm,'Cy')*2)]);
    delete_block([bc '/' nm]);
    add_block('simulink/Lookup Tables/n-D Lookup Table', [bc '/' nm], ...
        'NumberOfTableDimensions','2', 'Table',tbl, ...
        'BreakpointsForDimension1','alpha_vec', ...
        'BreakpointsForDimension2','beta_vec', ...
        'Position',[pos(1) pos(2)-12 pos(3)+25 pos(4)+12]);
    add_line(bc, 'Alpha1/1', [nm '/1'], 'autorouting','on');
    add_line(bc, [betaSrc '/1'], [nm '/2'], 'autorouting','on');
    add_line(bc, [nm '/1'], ['Mux/' num2str(strcmp(nm,'Cl')*4 + strcmp(nm,'Cy')*2)], ...
        'autorouting','on');
end
fprintf('  A. CY and Cl promoted from scalar beta gains to full 2-D tables\n');

%% ==== B. control increments =================================================
set_param([ai '/Prelookup'],   'BreakpointsData','alpha_vec_control');
set_param([ai '/AlphaLookup'], 'BreakpointsData','alpha_vec_control');

% The HL-20 path clamps alpha to >= 0 before the aileron prelookup, because its
% control tables only cover positive alpha. The replacement set is
% tabulated over the whole -8..+8 band, so the clamp is widened to the band it
% is actually valid over rather than left at zero.
sat = findBlockByName(ai, '(0 to inf) Constants Held fixed for alpha<0');
if ~isempty(sat)
    set_param(sat, 'LowerLimit','alpha_vec_control(1)', ...
                   'UpperLimit','alpha_vec_control(end)');
end
fprintf('  B. control breakpoints -> alpha_vec_control; alpha>=0 clamp widened to the band\n');

% Rudder: clone the aileron path, which already has the right parity structure
% (|delta| on CX/CZ/Cm, signed delta on CY/Cl/Cn) for an antisymmetric surface.
apos = get_param([ai '/Aileron'],'Position');
add_block([ai '/Aileron'], [ai '/Rudder'], ...
    'Position',[apos(1) apos(4)+60 apos(3) apos(4)+60+(apos(4)-apos(2))]);
for s = {{'Cx','CXdr'},{'Cy','CYdr'},{'Cz','CZdr'}, ...
         {'Cl','Cldr'},{'Cm','Cmdr'},{'Cn','Cndr'}}
    set_param([ai '/Rudder/' s{1}{1}], 'Table', s{1}{2});
end
set_param([ai '/Rudder/Aileron'], 'Name','Rudder');

% Feed it, and sum it in.
add_block('simulink/Sources/In1', [ai '/Rudder Deflection'], 'Port','5', ...
    'Position',[40 apos(4)+120 70 apos(4)+138]);
set_param([ai '/Sum'], 'Inputs','+++');
add_line(ai, 'Bus Creator/1', 'Rudder/1', 'autorouting','on');
add_line(ai, 'Rudder Deflection/1', 'Rudder/2', 'autorouting','on');
add_line(ai, 'Rudder/1', 'Sum/3', 'autorouting','on');
fprintf('  B. rudder path added (6 derivatives, |delta| parity on CX/CZ/Cm)\n');

%% ==== C. aerodynamic damping ================================================
% New subsystem. The HL-20 AirFrame terminates pqr and V; this uses them.
%
%   dCY = CYp*(p*b/2V) + CYr*(r*b/2V)
%   dCZ = CZq*(q*c/2V)
%   dCl = Clp*(p*b/2V) + Clr*(r*b/2V)
%   dCm = Cmq*(q*c/2V)
%   dCn = Cnp*(p*b/2V) + Cnr*(r*b/2V)
%
% Every derivative is a function of alpha over alpha_vec_damping. Implemented as
% a MATLAB Function block rather than 8 lookups and 8 products: it is one place
% to read, and it is not on the HDL path (this is plant, not controller).
dmp = [ac '/Damping Increments'];
add_block('simulink/User-Defined Functions/MATLAB Function', dmp, ...
    'Position',[300 470 440 560]);
setMatlabFcn(dmp, dampingCode());

% Tables reach the block as a parameter vector on a Constant, exactly as
% SENS_P and EST_P reach the estimator in build_est_v7.m. Keeps the block's
% interface small and keeps every number in one place in the setup file.
add_block('simulink/Sources/Constant', [ac '/DMP_P'], ...
    'Value','DMP_P', 'SampleTime','inf', 'Position',[190 585 250 613]);

delete_line_if(ac, 'pqr/1', 'Terminator/1');
delete_line_if(ac, 'V/1',   'Terminator1/1');
for b = {'Terminator','Terminator1'}
    if getSimulinkBlock(ac, b{1}), delete_block([ac '/' b{1}]); end
end
set_param([ac '/Sum'], 'Inputs','+++');
% alpha comes off alpha_rad -- the same signal the coefficient tables index on,
% so the damping lookup cannot disagree with them about units.
add_line(ac, 'alpha_rad/1', 'Damping Increments/1', 'autorouting','on');
add_line(ac, 'pqr/1',       'Damping Increments/2', 'autorouting','on');
add_line(ac, 'V/1',         'Damping Increments/3', 'autorouting','on');
add_line(ac, 'DMP_P/1',     'Damping Increments/4', 'autorouting','on');

% A MATLAB Function block emits a genuine 2-D [6x1] matrix, while the Mux path
% the other two terms come from emits a 1-D width-6 vector. Summing them makes
% the result 2-D, and the Aerospace Blockset Aerodynamic Forces and Moments
% block accepts only 1-D -- it rejects it with a dimensions error pointing at
% its own internal Demux, which is a long way from the cause. Reshape back to
% 1-D here.
add_block('simulink/Math Operations/Reshape', [ac '/Damping 1-D'], ...
    'OutputDimensionality','1-D array', 'Position',[470 500 505 530]);
add_line(ac, 'Damping Increments/1', 'Damping 1-D/1', 'autorouting','on');
add_line(ac, 'Damping 1-D/1', 'Sum/3', 'autorouting','on');
fprintf('  C. damping increments wired in (pqr and V now drive damping)\n');

%% ==== D. third control surface into the AirFrame ============================
add_block('simulink/Sources/In1', [ac '/Rudder Deflection'], 'Port','7', ...
    'Position',[40 420 70 438]);
add_line(ac, 'Rudder Deflection/1', 'Actuator Increments/5', 'autorouting','on');

afPos = get_param([af '/Elevator Deflection'],'Position');
add_block('simulink/Sources/In1', [af '/Rudder Deflection'], 'Port','3', ...
    'Position',[afPos(1) afPos(2)+60 afPos(3) afPos(4)+60]);
add_line(af, 'Rudder Deflection/1', 'Aerodynamic Coefficients/7', 'autorouting','on');
fprintf('  D. AirFrame now takes three surfaces\n');

%% ==== E. reference length, and the centre-of-pressure arm ===================
afm = findBlockByMaskType(af, 'Aerodynamic Forces and Moments');
set_param(afm, 'cbar','cref');

% The Aerodynamic Forces and Moments block adds (CP - CG) x F to the table
% moment. The FT Explorer tables and inertia are referenced to the same model
% CG, 57 mm aft of the wing leading edge, so CP is set equal to CG.
set_param([af '/CP'], 'Value','[-x_cg y_cg z_cg]');
fprintf('  E. cbar dref -> cref; CP pinned to CG (HL-20 uses [-12 0 0])\n');

%% ==== F. root wiring ========================================================
set_param([root '/Surfaces'], 'Outputs','3');

lpos = get_param([root '/Elevator Limit'],'Position');
add_block([root '/Elevator Limit'], [root '/Rudder Limit'], ...
    'Position',[lpos(1) lpos(4)+40 lpos(3) lpos(4)+40+(lpos(4)-lpos(2))]);
add_line(root, 'Surfaces/3', 'Rudder Limit/1', 'autorouting','on');
add_line(root, 'Rudder Limit/1', 'AirFrame/3', 'autorouting','on');

% delta_r (pre-saturation command) and delta_r_sat, appended so outports 1-15
% keep the indices every driver assumes.
add_block('simulink/Sinks/Out1', [root '/delta_r'], 'Port','16', ...
    'Position',[lpos(3)+120 lpos(4)+40 lpos(3)+150 lpos(4)+58]);
add_line(root, 'Surfaces/3', 'delta_r/1', 'autorouting','on');
add_block('simulink/Sinks/Out1', [root '/delta_r_sat'], 'Port','17', ...
    'Position',[lpos(3)+120 lpos(4)+80 lpos(3)+150 lpos(4)+98]);
add_line(root, 'Rudder Limit/1', 'delta_r_sat/1', 'autorouting','on');
fprintf('  F. ACT_MAP 3x3, rudder limiter, delta_r/delta_r_sat as outports 16/17\n');

%% ==== solver and diagnostics ================================================
% Fixed-step for anything touching the estimator -- on a variable-step solver a
% divergence collapses the step to ~4e-14 and runs for hours looking like a
% hang.
set_param(root, 'SolverType','Fixed-step','Solver','ode4','FixedStep','0.001');
set_param(root, 'UnconnectedInputMsg','error', ...
                'UnconnectedOutputMsg','warning');
set_param(root, 'StopTime','60');

save_system(root, outFile);
fprintf('\n  saved %s\n', outFile);

% Compile it here rather than leaving the first error to a driver. Use an
% update-diagram rather than the model compile command, which refuses to run on
% an accelerator-mode model.
fprintf('  compiling...\n');
try
    set_param(root,'SimulationCommand','update');
    fprintf('  [PASS] %s compiles\n', dstModel);
catch e
    fprintf(2,'  [FAIL] %s did not compile: %s\n', dstModel, e.message);
    rethrow(e);
end

% Dimensions are the cheap check that the table orientation is right: the
% static grids are 17x21 and non-square, so a stray transpose shows up here
% rather than as quietly wrong aerodynamics.
fprintf('\n  plant summary:\n');
fprintf('    static tables   %s on alpha %g..%g, beta %g..%g\n', ...
    mat2str(size(evalin('base','CX0'))), ...
    evalin('base','alpha_vec(1)'), evalin('base','alpha_vec(end)'), ...
    evalin('base','beta_vec(1)'),  evalin('base','beta_vec(end)'));
fprintf('    surfaces        aileron, elevator, rudder\n');
fprintf('    damping         Clp Clr Cmq Cnp Cnr CYp CYr CZq (new)\n');
fprintf('    mass %.4f kg, Sref %.6f m^2, cref %.6f m\n', ...
    evalin('base','mass'), evalin('base','Sref'), evalin('base','cref'));
end


% ===========================================================================
function code = dampingCode()
% Body of the Damping Increments MATLAB Function block. It delegates to
% tools/uav_damping_step.m so the physics lives in a file that can be read,
% diffed and driven offline -- the same split used for the sensor and estimator
% blocks. uav_damping_step is stateless, which is what makes that safe.
code = sprintf('%s\n', ...
'function dC = damping(alpha_deg, pqr, V, DMP_P)', ...
'%#codegen', ...
'% Aerodynamic damping increments, [CX CY CZ Cl Cm Cn] in body axes.', ...
'% The HL-20 AirFrame terminates pqr and V and carries no damping at all.', ...
'dC = uav_damping_step(alpha_deg, pqr, V, DMP_P);', ...
'end');
end

function setMatlabFcn(blkPath, code)
% Write the body of a MATLAB Function block.
sf = sfroot();
ch = sf.find('-isa','Stateflow.EMChart','Path',blkPath);
ch.Script = code;
end

function h = getSimulinkBlock(sys, name)
h = find_system(sys,'SearchDepth',1,'LookUnderMasks','all','Name',name);
h = ~isempty(h);
end

function b = findBlockByName(sys, name)
b = find_system(sys,'SearchDepth',1,'RegExp','off','Name',name);
if isempty(b)
    % Names in this model carry stray spaces and newlines.
    all_ = find_system(sys,'SearchDepth',1,'Type','Block');
    for k=1:numel(all_)
        if strcmp(strtrim(strrep(get_param(all_{k},'Name'),newline,' ')), strtrim(name))
            b = all_{k}; return
        end
    end
    b = '';
else
    b = b{1};
end
end

function b = findBlockByMaskType(sys, mt)
all_ = find_system(sys,'Type','Block');
b = '';
for k=1:numel(all_)
    try
        if strcmp(get_param(all_{k},'MaskType'), mt), b = all_{k}; return; end
    catch
    end
end
end

function delete_line_if(sys, src, dst)
try, delete_line(sys, src, dst); catch, end
end

function deleteLinesFrom(sys, blkName)
b = findBlockByName(sys, blkName);
if isempty(b), return; end
ph = get_param(b,'PortHandles');
for k=1:numel(ph.Outport)
    l = get_param(ph.Outport(k),'Line');
    if l ~= -1, try, delete_line(l); catch, end; end
end
end
