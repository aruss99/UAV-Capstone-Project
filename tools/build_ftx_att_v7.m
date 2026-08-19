function outFile = build_ftx_att_v7(srcDir, srcModel, dstDir, dstModel)
%BUILD_FTX_ATT_V7  Build the powered sensor-in-loop pitch-attitude model.
%
%   DroneModelv7_FTX_ATT differs from DroneModelv7_FTX in one control-law
%   decision: the longitudinal outer loop regulates pitch attitude theta rather
%   than reconstructed angle of attack.  The blended Euler estimate already
%   feeds the controller's roll channel; this build selects theta from the same
%   vector and feeds it through the longitudinal degrees conversion.  Alpha is
%   still estimated and logged, but it is not in the feedback path.
%
%   Constant effective power represents the selected FT Power Pack B Radial v2
%   and HQ 9x4.5 propeller. Its body-axis force is summed with aerodynamics
%   before BOTH the equations of motion and the specific-force sensor output,
%   so the IMU sees the acceleration the plant receives.
%
%   Three additional outputs expose inertial position, body velocity, and
%   propulsive thrust so the useful, bounded flight interval can be measured
%   directly.

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(srcDir),   srcDir   = fullfile(wb,'simulink','DroneModelv7_FTX'); end
if nargin<2||isempty(srcModel), srcModel = 'DroneModelv7_FTX'; end
if nargin<3||isempty(dstDir),   dstDir   = fullfile(wb,'simulink','DroneModelv7_FTX_ATT'); end
if nargin<4||isempty(dstModel), dstModel = 'DroneModelv7_FTX_ATT'; end

toolsDir = fullfile(wb,'tools');
addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(srcDir)); addpath(toolsDir);
if ~exist(dstDir,'dir'), mkdir(dstDir); end
outFile = fullfile(dstDir,[dstModel '.slx']);

evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_ftx_att.m')));

fprintf('\n=== building %s from %s ===\n', dstModel, srcModel);
try, close_system(dstModel,0); catch, end
if exist(outFile,'file'), delete(outFile); end
load_system(fullfile(srcDir,[srcModel '.slx']));
save_system(srcModel,outFile);
close_system(srcModel,0);
load_system(outFile);

root = dstModel;
fcs  = [root '/Flight Control System'];
af   = [root '/AirFrame'];

% Replace reconstructed alpha with theta from the already blended Euler
% estimate.  The FCS input path converts radians to degrees before the PI, so
% no new unit conversion is introduced here.
delete_line(root,'Sensors + Estimator/2','Flight Control System/2');
add_block('simulink/Sinks/Terminator',[root '/Unused Alpha Feedback'], ...
    'Position',[635 655 655 675]);
set_param([root '/Unused Alpha Feedback'],'AttributesFormatString', ...
    'alpha is monitored, not controlled');
add_line(root,'Sensors + Estimator/2','Unused Alpha Feedback/1','autorouting','smart');
sel = [root '/Pitch Attitude Feedback'];
add_block([fcs '/Selector'],sel, ...
    'Indices','2','InputPortWidth','3', ...
    'Position',[530 705 615 735]);
set_param(sel,'AttributesFormatString', ...
    '[CONTROL LAW] theta from blended Euler estimate');
add_line(root,'Sensors + Estimator/4','Pitch Attitude Feedback/1','autorouting','smart');
add_line(root,'Pitch Attitude Feedback/1','Flight Control System/2','autorouting','smart');

% Add the selected propulsion as constant effective power, T = P_eff / Vx.
% The low-speed thrust cap is only a numerical guard; it is inactive throughout
% the validated cruise cases. The force is inserted upstream of both the
% equations of motion and Force ---> Acc so the simulated accelerometer remains
% a physically consistent specific-force measurement.
prop = [af '/Constant Power Propulsion'];
add_block('simulink/User-Defined Functions/MATLAB Function',prop, ...
    'Position',[395 650 545 730]);
setMatlabFcn(prop,propulsionCode());
add_block('simulink/Sources/Constant',[af '/PROP_P'], ...
    'Value','FTX_PROP_P','SampleTime','inf','Position',[280 705 350 733]);
add_block('simulink/Math Operations/Reshape',[af '/Propulsion 1-D'], ...
    'OutputDimensionality','1-D array','Position',[575 655 610 685]);
add_block('simulink/Math Operations/Sum',[af '/Aero + Propulsion'], ...
    'Inputs','++','Position',[665 285 695 335]);

sixdof = find_system(af,'SearchDepth',1,'LookUnderMasks','all', ...
    'MaskType','6DOF EoM (Body Axis)');
afm = find_system(af,'SearchDepth',1,'LookUnderMasks','all', ...
    'MaskType','Aerodynamic Forces and Moments');
assert(numel(sixdof)==1 && numel(afm)==1, ...
    'build_ftx_att_v7: expected one 6DOF and one aerodynamic force block');
sixPH = get_param(sixdof{1},'PortHandles');
afmPH = get_param(afm{1},'PortHandles');
propPH = get_param(prop,'PortHandles');
parPH = get_param([af '/PROP_P'],'PortHandles');
reshapePH = get_param([af '/Propulsion 1-D'],'PortHandles');
forceSumPH = get_param([af '/Aero + Propulsion'],'PortHandles');
sum1PH = get_param([af '/Sum1'],'PortHandles');
accPH = get_param([af '/Force ---> Acc'],'PortHandles');

% Replace the branched aerodynamic-force line with the summed non-gravity
% force. This keeps both the dynamics and IMU specific force on the same path.
oldForceLine = get_param(afmPH.Outport(1),'Line');
if oldForceLine ~= -1, delete_line(oldForceLine); end
add_line(af,sixPH.Outport(5),propPH.Inport(1),'autorouting','on');
add_line(af,parPH.Outport(1),propPH.Inport(2),'autorouting','on');
add_line(af,propPH.Outport(1),reshapePH.Inport(1),'autorouting','on');
add_line(af,afmPH.Outport(1),forceSumPH.Inport(1),'autorouting','on');
add_line(af,reshapePH.Outport(1),forceSumPH.Inport(2),'autorouting','on');
add_line(af,forceSumPH.Outport(1),sum1PH.Inport(1),'autorouting','on');
add_line(af,forceSumPH.Outport(1),accPH.Inport(1),'autorouting','on');

add_block('simulink/Sinks/Out1',[af '/Propulsive Thrust'], ...
    'Port','13','Position',[740 695 770 713]);
thrustOutPH = get_param([af '/Propulsive Thrust'],'PortHandles');
add_line(af,propPH.Outport(2),thrustOutPH.Inport(1),'autorouting','on');

% Make the changed command semantics visible at the top level.  Only the block
% name changes; the second element of the command vector remains in degrees.
if ~isempty(find_system(root,'SearchDepth',1,'Name','alpha cmd (deg)'))
    set_param([root '/alpha cmd (deg)'],'Name','theta cmd (deg)');
end

% Preserve alpha as a monitored safety variable and expose the translational
% states needed to evaluate the bounded flight interval.
add_block('simulink/Sinks/Out1',[root '/position Xe'], ...
    'Port','18','Position',[1010 845 1040 863]);
add_line(root,'AirFrame/2','position Xe/1','autorouting','smart');
add_block('simulink/Sinks/Out1',[root '/velocity Vb'], ...
    'Port','19','Position',[1010 885 1040 903]);
add_line(root,'AirFrame/5','velocity Vb/1','autorouting','smart');
add_block('simulink/Sinks/Out1',[root '/propulsive thrust'], ...
    'Port','20','Position',[1010 925 1040 943]);
add_line(root,'AirFrame/13','propulsive thrust/1','autorouting','smart');

set_param(root,'Description',sprintf([ ...
    'DroneModelv7_FTX_ATT - FT Explorer sensor-in-loop pitch-attitude hold.\n' ...
    'Generated by tools/build_ftx_att_v7.m.\n\n' ...
    'The longitudinal outer loop regulates theta from the blended Euler\n' ...
    'estimate. Alpha remains estimated and logged as an aerodynamic-envelope\n' ...
    'variable but is not used for feedback. Propulsion is the selected FT\n' ...
    'Power Pack B Radial v2 represented by constant effective power.']));
set_param(root,'UnconnectedInputMsg','error','UnconnectedOutputMsg','warning');
save_system(root,outFile);

fprintf('  theta estimate -> longitudinal PI input; alpha removed from feedback\n');
fprintf('  constant-power propulsion -> dynamics and specific-force sensor path\n');
fprintf('  appended outports 18 position Xe, 19 velocity Vb, 20 thrust\n');
fprintf('  compiling...\n');
set_param(root,'SimulationCommand','update');
fprintf('  [PASS] %s compiles\n',dstModel);
close_system(root,0);
end

function code = propulsionCode()
code = sprintf('%s\n', ...
'function [Fprop, thrust] = constant_power_propulsion(Vb, P)', ...
'%#codegen', ...
'% P = [effective power W; minimum axial speed m/s; thrust cap N].', ...
'u = Vb(1);', ...
'if u < P(2)', ...
'    u = P(2);', ...
'end', ...
'thrust = P(1)/u;', ...
'if thrust > P(3)', ...
'    thrust = P(3);', ...
'elseif thrust < 0', ...
'    thrust = 0;', ...
'end', ...
'Fprop = [thrust; 0; 0];', ...
'end');
end

function setMatlabFcn(blkPath, code)
sf = sfroot();
ch = sf.find('-isa','Stateflow.EMChart','Path',blkPath);
ch.Script = code;
end
