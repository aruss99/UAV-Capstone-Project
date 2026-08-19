function outFile = build_rt_v7(srcDir, srcModel, dstDir, dstModel)
%BUILD_RT_V7  Build DroneModelv7_RT: the re-tunable controller.
%
%   Built on top of DroneModelv7_EST. Adds output limits, anti-windup and
%   surface saturation, and exposes the longitudinal gains as scale factors so
%   the loop can be re-tuned against a plant that INCLUDES the sensor and
%   estimator models.
%
%   Model configuration:
%
%   1. RATE-CORRECT LAG FILTERS. In DroneModelv7_EST the three Discrete
%      Transfer Fcn blocks hold the ZOH of 10/(s+10) as NUMERIC LITERALS for
%      Tc = 0.2, while their SampleTime field says 'Tc'. Changing Tc therefore
%      mis-discretises them silently. Expressions in Tc keep the coefficients
%      consistent with the configured sample time.
%
%   2. PI OUTPUT LIMITS + ANTI-WINDUP.
%      All three blocks are stock Discrete PID Controller blocks with
%      LimitOutput off and AntiWindupMode none, so the integrators can wind up
%      without bound. Clamping avoids an additional back-calculation gain.
%
%   3. SURFACE SATURATION.
%      The control derivatives are tabulated per degree over a modest
%      deflection range, while the unlimited model commands 100+ and, with a
%      bad beta estimate, 4 455. Saturating at RT_SURF_LIM keeps the plant
%      inside the range its own aero data is valid over.
%
%      The delta_a / delta_e outports log the pre-saturation command. Appended
%      outports log the applied deflection while preserving the positional
%      indices of outports 1-13.
%
%   4. TUNABLE LONGITUDINAL GAINS. The P gains are not lookups - they are
%      S-Function subsystems returning a hardcoded constant, so they cannot be
%      retuned from the workspace. A Gain block is inserted on each P-block
%      output. The I and rate-feedback gains are already workspace tables and
%      are scaled in uav_setup_v7_rt.m instead.
%
%   Usage:  build_rt_v7

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(srcDir),   srcDir   = fullfile(wb,'simulink','DroneModelv7_EST'); end
if nargin<2||isempty(srcModel), srcModel = 'DroneModelv7_EST'; end
if nargin<3||isempty(dstDir),   dstDir   = fullfile(wb,'simulink','DroneModelv7_RT'); end
if nargin<4||isempty(dstModel), dstModel = 'DroneModelv7_RT';  end

toolsDir = fullfile(wb,'tools');
addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(srcDir)); addpath(toolsDir);
if ~exist(dstDir,'dir'), mkdir(dstDir); end
outFile = fullfile(dstDir,[dstModel '.slx']);

evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_rt.m')));

fprintf('\n=== building %s from %s ===\n', dstModel, srcModel);
try, close_system(dstModel,0); catch, end
if exist(outFile,'file'), delete(outFile); end
load_system(fullfile(srcDir,[srcModel '.slx']));
save_system(srcModel, outFile);
close_system(srcModel,0);
load_system(outFile);

root = dstModel;
ctl  = [root '/Flight Control System/Controller'];

%% ---- 1. rate-correct the lag filters ---------------------------------------
% ZOH of 10/(s+10) at Tc:  H(z) = (1-a)/(z-a),  a = exp(-10*Tc).
% As expressions, so they track Tc instead of being frozen at Tc = 0.2.
W_LAG = 10;
tfs = find_system(ctl,'SearchDepth',1,'BlockType','DiscreteTransferFcn');
for k = 1:numel(tfs)
    set_param(tfs{k}, ...
        'Numerator',   sprintf('1-exp(-%g*Tc)', W_LAG), ...
        'Denominator', sprintf('[1, -exp(-%g*Tc)]', W_LAG), ...
        'SampleTime',  'Tc');
    set_param(tfs{k},'AttributesFormatString', ...
        'ZOH of 10/(s+10) as an expression in Tc');
end
fprintf('  %d lag filters expressed as functions of Tc\n', numel(tfs));

%% ---- 2. PI output limits + anti-windup -------------------------------------
pids = {'PI Alpha','RT_LIM_ALPHA'; 'PI Beta','RT_LIM_BETA'; 'PI phi','RT_LIM_PHI'};
for k = 1:size(pids,1)
    blk = [ctl '/' pids{k,1}];
    set_param(blk, ...
        'LimitOutput',          'on', ...
        'UpperSaturationLimit', pids{k,2}, ...
        'LowerSaturationLimit', ['-' pids{k,2}], ...
        'AntiWindupMode',       'clamping');
    set_param(blk,'AttributesFormatString', ...
        sprintf('output limited to +-%s, clamping anti-windup', pids{k,2}));
    fprintf('  %-9s output limited to +-%s, anti-windup clamping\n', pids{k,1}, pids{k,2});
end

%% ---- 3. tunable P gains -----------------------------------------------------
% The P blocks are S-Function subsystems returning a constant, so the only way
% to reach them from the workspace is to scale their output.
pgain = {'P alpha','PI Alpha','RT_PA_SCALE'
         'P beta', 'PI Beta', 'RT_PB_SCALE'
         'P phi',  'PI phi',  'RT_PP_SCALE'};
for k = 1:size(pgain,1)
    src = pgain{k,1}; dst = pgain{k,2}; par = pgain{k,3};
    gname = ['RT scale ' src];
    pos = get_param([ctl '/' src],'Position');
    delete_line(ctl, [src '/1'], [dst '/2']);
    add_block('simulink/Math Operations/Gain', [ctl '/' gname], ...
        'Gain', par, 'Position', [pos(3)+25 pos(2) pos(3)+60 pos(2)+30]);
    set_param([ctl '/' gname],'AttributesFormatString', ...
        sprintf('retune handle on the fixed %s gain', src));
    add_line(ctl, [src '/1'],   [gname '/1'], 'autorouting','smart');
    add_line(ctl, [gname '/1'], [dst '/2'],   'autorouting','smart');
    fprintf('  %-9s -> %s -> %s\n', src, gname, dst);
end

%% ---- 4. surface saturation --------------------------------------------------
apos = get_param([root '/AirFrame'],'Position');
surf = {'Aileron Limit','Surfaces/1','AirFrame/1',1
        'Elevator Limit','Surfaces/2','AirFrame/2',2};
for k = 1:size(surf,1)
    nm = surf{k,1};
    delete_line(root, surf{k,2}, surf{k,3});
    add_block('simulink/Discontinuities/Saturation', [root '/' nm], ...
        'UpperLimit','RT_SURF_LIM', 'LowerLimit','-RT_SURF_LIM', ...
        'Position', [apos(1)-190 apos(2)+40*k-20 apos(1)-150 apos(2)+40*k+10]);
    set_param([root '/' nm],'AttributesFormatString', ...
        'control derivatives are per-degree over a modest range');
    add_line(root, surf{k,2},  [nm '/1'], 'autorouting','smart');
    add_line(root, [nm '/1'],  surf{k,3}, 'autorouting','smart');
    fprintf('  %s: +-RT_SURF_LIM between mapping and AirFrame\n', nm);
end

% Log what the surface actually did, APPENDED so indices 1-13 do not move.
outs = find_system(root,'SearchDepth',1,'BlockType','Outport');
n    = numel(outs);
opos = get_param(outs{end},'Position');
newOuts = {'delta_a_sat','Aileron Limit/1'; 'delta_e_sat','Elevator Limit/1'};
for k = 1:size(newOuts,1)
    add_block('simulink/Sinks/Out1', [root '/' newOuts{k,1}], ...
        'Position', opos + [0 40*k 0 40*k], 'Port', num2str(n+k));
    add_line(root, newOuts{k,2}, [newOuts{k,1} '/1'], 'autorouting','smart');
end
fprintf('  logging added: delta_a_sat (%d), delta_e_sat (%d) - existing ports unmoved\n', n+1, n+2);

%% ---- 5. keep the diagnostics strict ----------------------------------------
set_param(root,'UnconnectedInputMsg','error');
set_param(root,'Description', sprintf([ ...
    'DroneModelv7_RT - re-tunable controller, generated by\n' ...
    'tools/build_rt_v7.m from DroneModelv7_EST.\n\n' ...
    'Adds output limits, clamping anti-windup and surface saturation, and\n' ...
    'exposes the fixed P gains as scale factors so the loop can be re-tuned\n' ...
    'against a plant that includes the sensor and estimator models.\n\n' ...
    'Outports 1-13 keep the indices they have in DroneModelv7_EST; 14 and 15\n' ...
    'are the post-saturation surface deflections. 7 and 8 remain the\n' ...
    'pre-saturation COMMAND, which is the diagnostic worth keeping.']));

save_system(root);
close_system(root,0);
fprintf('\nwrote %s\n', outFile);
end
