function outFile = build_est_v7(srcDir, srcModel, dstDir, dstModel)
%BUILD_EST_V7  Insert a sensor + estimator layer between the AirFrame and the
%   Flight Control System, producing DroneModelv7_EST.
%
%   Built on top of DroneModelv7_CL, replacing the ideal state feedback with
%   modelled sensors and an estimator.
%
%   Structure inside the "Sensors + Estimator" subsystem:
%
%     truth --> [Sensor Models] --> measurements --> [Estimator] --> estimates
%                (simulation only)                   (HDL-targetable)   |
%                                                                       v
%                                        truth ----------------> [Blend] --> FCS
%
%   The split is deliberate: the sensor block represents physical hardware and
%   would never be synthesised; the estimator block is the part that would.
%
%   Usage:  build_est_v7

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(srcDir),   srcDir   = fullfile(wb,'simulink','DroneModelv7_CL'); end
if nargin<2||isempty(srcModel), srcModel = 'DroneModelv7_CL';  end
if nargin<3||isempty(dstDir),   dstDir   = fullfile(wb,'simulink','DroneModelv7_EST'); end
if nargin<4||isempty(dstModel), dstModel = 'DroneModelv7_EST'; end

toolsDir = fullfile(wb,'tools');
addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(srcDir)); addpath(toolsDir);
if ~exist(dstDir,'dir'), mkdir(dstDir); end
outFile = fullfile(dstDir,[dstModel '.slx']);

evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_est.m')));

fprintf('\n=== building %s from %s ===\n', dstModel, srcModel);
try, close_system(dstModel,0); catch, end
if exist(outFile,'file'), delete(outFile); end
load_system(fullfile(srcDir,[srcModel '.slx']));
save_system(srcModel, outFile);
close_system(srcModel,0);
load_system(outFile);

root = dstModel;
fcs  = [root '/Flight Control System'];
sub  = [root '/Sensors + Estimator'];

%% ---- A. remove the ideal truth feedback ------------------------------------
% These four lines are what close_loop_v7.m added: plant states straight into
% the controller with no sensor in between.
old = { 'AirFrame/6', 'Flight Control System/1'
        'AirFrame/9', 'Flight Control System/2'
        'AirFrame/10','Flight Control System/3'
        'AirFrame/3', 'Flight Control System/4' };
for k = 1:size(old,1)
    delete_line(root, old{k,1}, old{k,2});
end
fprintf('  removed 4 ideal-feedback lines (AirFrame -> FCS)\n');

%% ---- B. the subsystem -------------------------------------------------------
apos = get_param([root '/AirFrame'],'Position');
add_block('built-in/Subsystem', sub, ...
    'Position', [apos(1)-40 apos(4)+380 apos(1)+240 apos(4)+660]);
set_param(sub,'AttributesFormatString','sensors + estimator');
% built-in/Subsystem can come with a default In1->Out1 pair; clear it
inner = find_system(sub,'SearchDepth',1,'Type','Block');
for k = 1:numel(inner)
    if ~strcmp(inner{k},sub), delete_block(inner{k}); end
end

inNames  = {'pqr_t','acc_t','Vb_t','Ve_t','alpha_t','beta_t','euler_t'};
for k = 1:numel(inNames)
    add_block('simulink/Sources/In1', [sub '/' inNames{k}], ...
        'Port', num2str(k), 'Position', [30 40+60*k 60 58+60*k]);
end

cst = {'SENS_P','EST_P','EST_BLEND'};
for k = 1:numel(cst)
    add_block('simulink/Sources/Constant', [sub '/' cst{k}], ...
        'Value', cst{k}, 'SampleTime','inf', ...
        'Position', [30 500+60*k 90 528+60*k]);
end

mf = 'simulink/User-Defined Functions/MATLAB Function';
add_block(mf, [sub '/Sensor Models'], 'Position',[190 100 330 260]);
add_block(mf, [sub '/Estimator'],     'Position',[420 100 560 260]);
add_block(mf, [sub '/Blend'],         'Position',[650 100 790 320]);

rt = sfroot;
setScript = @(pth,txt) set(rt.find('-isa','Stateflow.EMChart','Path',pth),'Script',txt);

setScript([sub '/Sensor Models'], sprintf([ ...
 'function [gyro_m, acc_m, V_m, Vz_m, sb_out] = fcn(pqr_t, acc_t, Vb_t, Ve_t, SENS_P, sb_in)\n' ...
 '%%#codegen\n' ...
 '[gyro_m, acc_m, V_m, Vz_m, sb_out] = uav_sensors_step(pqr_t, acc_t, Vb_t, Ve_t, SENS_P, sb_in);\n' ...
 'end\n']));

setScript([sub '/Estimator'], sprintf([ ...
 'function [pqr_hat, alpha_hat, beta_hat, euler_hat, s_out] = fcn(gyro_m, acc_m, V_m, Vz_m, EST_P, s_in)\n' ...
 '%%#codegen\n' ...
 '[pqr_hat, alpha_hat, beta_hat, euler_hat, s_out] = uav_estimator_step(gyro_m, acc_m, V_m, Vz_m, EST_P, s_in);\n' ...
 'end\n']));

setScript([sub '/Blend'], sprintf([ ...
 'function [pqr_o, alpha_o, beta_o, euler_o] = fcn(pqr_h, alpha_h, beta_h, euler_h, pqr_t, alpha_t, beta_t, euler_t, BL)\n' ...
 '%%#codegen\n' ...
 '[pqr_o, alpha_o, beta_o, euler_o] = uav_blend_step(pqr_h, alpha_h, beta_h, euler_h, pqr_t, alpha_t, beta_t, euler_t, BL);\n' ...
 'end\n']));

% Discrete, at the control rate. This is not cosmetic: these blocks hold
% persistent state, and an inherited (continuous) sample time would advance
% that state on every minor solver step. Sample time lives on the Stateflow
% chart object, not on the block, so set_param does not reach it.
for b = {'Sensor Models','Estimator','Blend'}
    ch = rt.find('-isa','Stateflow.EMChart','Path',[sub '/' b{1}]);
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = 'EST_TS';
    fprintf('  %-14s -> DISCRETE @ EST_TS\n', b{1});
end

% ---- one-sample sensor/compute latency --------------------------------------
% REQUIRED, and for a physical reason worth stating. The AirFrame's Accels
% output is NOT a state: aerodynamic force depends instantaneously on control
% deflection, so routing it through the estimator into the controller creates
% a direct-feedthrough algebraic loop that Simulink refuses to solve.
%
% The IMU, estimator and controller form a sampled pipeline. An explicit unit
% delay represents that latency and breaks the algebraic loop.
%
% It is not free: one sample of delay at 5 Hz is 0.2 s of dead time, which is
% a direct phase-margin cost on the loop.
% ---- filter state, held in explicit Unit Delays ----------------------------
% The sensor and estimator functions are STATELESS: state goes in and comes
% back out, and these delays hold it between samples.
%
% MATLAB `persistent` variables inside an external function are not
% Simulink-managed block state and may advance on solver evaluations. Explicit
% delays update state once per sample and keep the estimator HDL-targetable.
add_block('simulink/Discrete/Unit Delay', [sub '/Estimator State'], ...
    'SampleTime','EST_TS', 'InitialCondition','zeros(10,1)', ...
    'Position',[420 320 470 360]);
add_block('simulink/Discrete/Unit Delay', [sub '/Sensor State'], ...
    'SampleTime','EST_TS', 'InitialCondition','zeros(4,1)', ...
    'Position',[190 320 240 360]);

delayNames = {'zoh_pqr','zoh_alpha','zoh_beta','zoh_euler'};
for k = 1:4
    add_block('simulink/Discrete/Unit Delay', [sub '/' delayNames{k}], ...
        'SampleTime','EST_TS', 'InitialCondition','0', ...
        'Position',[590 100+55*k 630 130+55*k]);
    set_param([sub '/' delayNames{k}],'AttributesFormatString', ...
        'sensor + compute latency, 1 sample');
end

outNames = {'pqr_o','alpha_o','beta_o','euler_o', ...
            'pqr_hat','alpha_hat','beta_hat','euler_hat'};
for k = 1:numel(outNames)
    add_block('simulink/Sinks/Out1', [sub '/' outNames{k}], ...
        'Port', num2str(k), 'Position', [900 40+55*k 930 58+55*k]);
end

L = { 'pqr_t/1','Sensor Models/1'; 'acc_t/1','Sensor Models/2'
      'Vb_t/1','Sensor Models/3';  'Ve_t/1','Sensor Models/4'
      'SENS_P/1','Sensor Models/5'
      'Sensor State/1','Sensor Models/6'
      'Sensor Models/5','Sensor State/1'
      'Sensor Models/1','Estimator/1'; 'Sensor Models/2','Estimator/2'
      'Sensor Models/3','Estimator/3'; 'Sensor Models/4','Estimator/4'
      'EST_P/1','Estimator/5'
      'Estimator State/1','Estimator/6'
      'Estimator/5','Estimator State/1'
      'Estimator/1','zoh_pqr/1';  'Estimator/2','zoh_alpha/1'
      'Estimator/3','zoh_beta/1'; 'Estimator/4','zoh_euler/1'
      'zoh_pqr/1','Blend/1';      'zoh_alpha/1','Blend/2'
      'zoh_beta/1','Blend/3';     'zoh_euler/1','Blend/4'
      'pqr_t/1','Blend/5';   'alpha_t/1','Blend/6'
      'beta_t/1','Blend/7';  'euler_t/1','Blend/8'
      'EST_BLEND/1','Blend/9'
      'Blend/1','pqr_o/1';   'Blend/2','alpha_o/1'
      'Blend/3','beta_o/1';  'Blend/4','euler_o/1'
      'zoh_pqr/1','pqr_hat/1';  'zoh_alpha/1','alpha_hat/1'
      'zoh_beta/1','beta_hat/1'; 'zoh_euler/1','euler_hat/1' };
for k = 1:size(L,1)
    add_line(sub, L{k,1}, L{k,2}, 'autorouting','smart');
end
fprintf('  subsystem built: 7 in, 8 out, 3 function blocks\n');

%% ---- C. wire it into the loop ----------------------------------------------
feed = { 'AirFrame/6', 'Sensors + Estimator/1', 'pqr   (truth) -> sensors'
         'AirFrame/12','Sensors + Estimator/2', 'Accels(truth, SPECIFIC FORCE) -> sensors'
         'AirFrame/5', 'Sensors + Estimator/3', 'Vb    (truth) -> airspeed'
         'AirFrame/1', 'Sensors + Estimator/4', 'Ve    (truth) -> vertical speed'
         'AirFrame/9', 'Sensors + Estimator/5', 'alpha (truth) -> blend'
         'AirFrame/10','Sensors + Estimator/6', 'beta  (truth) -> blend'
         'AirFrame/3', 'Sensors + Estimator/7', 'Euler (truth) -> blend' };
for k = 1:size(feed,1)
    add_line(root, feed{k,1}, feed{k,2}, 'autorouting','smart');
    fprintf('  %s\n', feed{k,3});
end
back = { 'Sensors + Estimator/1','Flight Control System/1'
         'Sensors + Estimator/2','Flight Control System/2'
         'Sensors + Estimator/3','Flight Control System/3'
         'Sensors + Estimator/4','Flight Control System/4' };
for k = 1:size(back,1)
    add_line(root, back{k,1}, back{k,2}, 'autorouting','smart');
end
fprintf('  estimator -> FCS (replaces ideal feedback)\n');

%% ---- D. log the estimates alongside truth ----------------------------------
outs = find_system(root,'SearchDepth',1,'BlockType','Outport');
n    = numel(outs);
opos = get_param(outs{end},'Position');
logs = { 'est_pqr','Sensors + Estimator/5'; 'est_alpha','Sensors + Estimator/6'
         'est_beta','Sensors + Estimator/7'; 'est_euler','Sensors + Estimator/8' };
for k = 1:size(logs,1)
    add_block('simulink/Sinks/Out1', [root '/' logs{k,1}], ...
        'Position', opos + [0 40*k 0 40*k], 'Port', num2str(n+k));
    add_line(root, logs{k,2}, [logs{k,1} '/1'], 'autorouting','smart');
end
fprintf('  logging added: est_pqr, est_alpha, est_beta, est_euler\n');

set_param(root,'UnconnectedInputMsg','error');
set_param(root,'Description', sprintf([ ...
    'DroneModelv7_EST - closed loop with a sensor + estimator layer.\n' ...
    'Generated by tools/build_est_v7.m.\n\n' ...
    'Replaces ideal state feedback with a BMI323-class IMU, MS4525DO-class\n' ...
    'airspeed, and a quaternion complementary (Mahony) filter with gyro bias\n' ...
    'estimation and fixed-wing accelerometer compensation.\n\n' ...
    'EST_BLEND selects estimate vs truth per channel so each channel''s\n' ...
    'contribution to closed-loop degradation can be measured separately.']));

save_system(root);
close_system(root,0);
fprintf('\nwrote %s\n', outFile);
end
