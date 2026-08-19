function outFile = close_loop_v7(srcDir, srcModel, dstDir, dstModel)
%CLOSE_LOOP_V7  Build a closed-loop DroneModelv7 by connecting the Flight
%   Control System to the AirFrame through an explicit actuator mapping.
%
%   simulink/DroneModelv7 is not modified; this builds a separate model.
%
%   In DroneModelv7 the Flight Control System is a disconnected island: all
%   four of its inputs dangle, it has no output ports, and both AirFrame
%   control inputs dangle. UnconnectedInputMsg = 'none' lets it run anyway.
%
%   Usage:  close_loop_v7

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(srcDir),   srcDir   = fullfile(wb,'simulink','DroneModelv7'); end
if nargin < 2 || isempty(srcModel), srcModel = 'DroneModelv7';    end
if nargin < 3 || isempty(dstDir),   dstDir   = fullfile(wb,'simulink','DroneModelv7_CL'); end
if nargin < 4 || isempty(dstModel), dstModel = 'DroneModelv7_CL'; end

toolsDir = fullfile(wb,'tools');
addpath(genpath(srcDir)); addpath(toolsDir);
if ~exist(dstDir,'dir'), mkdir(dstDir); end
outFile = fullfile(dstDir,[dstModel '.slx']);

% The model needs its base-workspace variables just to load cleanly.
evalin('base', sprintf('run(''%s'')', fullfile(toolsDir,'uav_setup_v7_cl.m')));

fprintf('\n=== building %s from %s ===\n', dstModel, srcModel);
close_system(dstModel, 0);            %#ok<*NASGU>
if exist(outFile,'file'), delete(outFile); end
load_system(srcModel);
save_system(srcModel, outFile);
close_system(srcModel, 0);
load_system(outFile);

root = dstModel;
fcs  = [root '/Flight Control System'];
ctl  = [fcs  '/Controller'];
af   = [root '/AirFrame'];

%% ---------------------------------------------------------------------------
%  A. Inside the Flight Control System
%% ---------------------------------------------------------------------------
% A1 The FCS is internally complete apart from one gap. Its Selector already
%     pulls phi (index 1) out of phi;theta;psi and feeds Controller/Phi, and
%     the two Angle Conversion blocks already feed Alpha and Beta. Only
%     Controller inport 4 has no source. Assert that rather than assume it.
ctlPorts = get_param(ctl,'PortHandles');
openIn = find(arrayfun(@(p) get_param(p,'Line') == -1, ctlPorts.Inport));
assert(isequal(openIn(:)', 4), ...
    'expected Controller inport 4 (Demands) to be the only open input, got %s', mat2str(openIn));
fprintf('  [CHECK]    Controller inputs open: only port 4 (Demands), as expected\n');

% A2 Controller inport 4 "Demands" has no source. Bring it out to the
%     subsystem boundary so the command vector is supplied from the top level
%     and stays visible as an input, not a buried constant.
pos = get_param([fcs '/PQR '],'Position');
add_block('simulink/Sources/In1', [fcs '/Demands'], ...
    'Position', pos + [0 200 0 200], 'Port','5');
add_line(fcs, 'Demands/1', 'Controller/4', 'autorouting','smart');
set_param([fcs '/Demands'], 'AttributesFormatString', 'test command vector');
fprintf('  [ADDED]    new Demands inport -> Controller/Demands\n');

% A3 The Controller emits u_swm but the subsystem has no output port at all.
%     Add one.
cpos = get_param(ctl,'Position');
add_block('simulink/Sinks/Out1', [fcs '/u_swm'], ...
    'Position', [cpos(3)+120 cpos(2)+40 cpos(3)+150 cpos(2)+58], 'Port','1');
add_line(fcs, 'Controller/1', 'u_swm/1', 'autorouting','smart');
fprintf('  [ADDED]    Controller/u_swm -> new subsystem outport\n');

%% ---------------------------------------------------------------------------
%  B. Root level - sensor feedback into the controller
%% ---------------------------------------------------------------------------
% Port names on both sides determine these four uniquely. The AirFrame emits
% alpha/beta/Euler in radians; the FCS converts alpha and beta to degrees
% internally with its own Angle Conversion blocks, and takes phi in radians.
% That unit split is the controller's own.
feedback = { 'AirFrame/6', 'Flight Control System/1', 'pqr    -> PQR'
             'AirFrame/9', 'Flight Control System/2', 'alpha  -> Alpha  (rad, converted to deg inside)'
             'AirFrame/10','Flight Control System/3', 'beta   -> Beta   (rad, converted to deg inside)'
             'AirFrame/3', 'Flight Control System/4', 'Euler  -> phi;theta;psi (rad)' };
for k = 1:size(feedback,1)
    add_line(root, feedback{k,1}, feedback{k,2}, 'autorouting','smart');
    fprintf('  [WIRED]    %s\n', feedback{k,3});
end

% The command source: three Step blocks rather than a constant, so the same
% model can hold a trim command or inject a step on any one axis -
% a step is the only way to tell a working loop from an inert one on a plant
% that has no thrust input and therefore descends no matter what the
% controller does. Setting DEMAND0 == DEMAND1 makes any channel a constant.
apos = get_param(af,'Position');
cmdNames = {'phi cmd (rad)','alpha cmd (deg)','beta cmd (deg)'};
for k = 1:3
    b = [root '/' cmdNames{k}];
    add_block('simulink/Sources/Step', b, ...
        'Time',   sprintf('T_STEP(%d)',k), ...
        'Before', sprintf('DEMAND0(%d)',k), ...
        'After',  sprintf('DEMAND1(%d)',k), ...
        'SampleTime','0', ...
        'Position',[apos(1)-320 apos(4)+120+55*k apos(1)-260 apos(4)+150+55*k]);
    set_param(b,'AttributesFormatString','test command');
end
add_block('simulink/Signal Routing/Mux', [root '/Demands'], 'Inputs','3', ...
    'Position',[apos(1)-200 apos(4)+170 apos(1)-195 apos(4)+320]);
for k = 1:3
    add_line(root, [cmdNames{k} '/1'], sprintf('Demands/%d',k), 'autorouting','smart');
end
add_line(root, 'Demands/1', 'Flight Control System/5', 'autorouting','smart');
fprintf('  [ADDED]    3 Step commands -> Demands mux -> FCS/Demands\n');

%% ---------------------------------------------------------------------------
%  C. Root level - the actuator mapping itself (open question #10)
%% ---------------------------------------------------------------------------
% One matrix gain maps controller channels to plant surfaces:
%     [delta_a; delta_e] = ACT_MAP * u_swm
% Rows 1-2 are forced by the channel trace and the AirFrame's inport names.
% Column 3 is zero because there is no rudder to map the yaw channel onto.
% ACT_MAP_SIGN supplies the negative-feedback polarity.
fpos = get_param(fcs,'Position');
mapPos = [fpos(1)-360 fpos(2)+40 fpos(1)-280 fpos(2)+100];
add_block('simulink/Math Operations/Gain', [root '/Actuator Mapping'], ...
    'Gain','ACT_MAP', 'Multiplication','Matrix(K*u)', 'Position', mapPos);
set_param([root '/Actuator Mapping'], 'AttributesFormatString', ...
    '[da;de] = ACT_MAP*u_swm');
add_block('simulink/Signal Routing/Demux', [root '/Surfaces'], ...
    'Outputs','2', 'Position',[mapPos(1)+130 mapPos(2) mapPos(1)+135 mapPos(4)]);
add_line(root, 'Flight Control System/1', 'Actuator Mapping/1', 'autorouting','smart');
add_line(root, 'Actuator Mapping/1', 'Surfaces/1', 'autorouting','smart');
add_line(root, 'Surfaces/1', 'AirFrame/1', 'autorouting','smart');
add_line(root, 'Surfaces/2', 'AirFrame/2', 'autorouting','smart');
fprintf('  [WIRED]    u_swm -> Actuator Mapping -> AirFrame aileron/elevator\n');

% The third channel has nowhere to go. Rather than delete it, expose it so the
% simulation records what the yaw channel asks for while no surface exists to
% deliver it.
add_block('simulink/Signal Routing/Selector', [root '/Unmapped Yaw Channel'], ...
    'IndexOptions','Index vector (dialog)', 'Indices','[3]', ...
    'InputPortWidth','3', 'OutputSizes','1', ...
    'Position',[mapPos(1) mapPos(2)+140 mapPos(3) mapPos(2)+175]);
set_param([root '/Unmapped Yaw Channel'], 'AttributesFormatString', ...
    'no rudder: no AirFrame inport, no *dr derivative');
add_line(root, 'Flight Control System/1', 'Unmapped Yaw Channel/1', 'autorouting','smart');

%% ---------------------------------------------------------------------------
%  D. Log the new signals
%% ---------------------------------------------------------------------------
outs = find_system(root,'SearchDepth',1,'BlockType','Outport');
n = numel(outs);
opos = get_param(outs{end},'Position');
newOuts = { 'delta_a',          'Surfaces/1'
            'delta_e',          'Surfaces/2'
            'u_yaw_unmapped',   'Unmapped Yaw Channel/1' };
for k = 1:size(newOuts,1)
    nm = newOuts{k,1};
    add_block('simulink/Sinks/Out1', [root '/' nm], ...
        'Position', opos + [0 40*k 0 40*k], 'Port', num2str(n+k));
    add_line(root, newOuts{k,2}, [nm '/1'], 'autorouting','smart');
end
fprintf('  logging added: delta_a, delta_e, u_yaw_unmapped\n');

%% ---------------------------------------------------------------------------
%  E. Stop the model from ever silently grounding an input again
%% ---------------------------------------------------------------------------
% This single setting is what lets an open loop run without complaint.
set_param(root, 'UnconnectedInputMsg',  'error');
set_param(root, 'UnconnectedOutputMsg', 'warning');
set_param(root, 'UnconnectedLineMsg',   'warning');
fprintf('  [HARDENING] UnconnectedInputMsg: none -> error\n');

set_param(root,'Description', sprintf([ ...
    'DroneModelv7_CL - closed-loop build, generated by\n' ...
    'tools/close_loop_v7.m.\n\n' ...
    'Derived from DroneModelv7, which is open-loop: its Flight Control\n' ...
    'System is not connected to the AirFrame.\n\n' ...
    'The controller/surface mapping in the "Actuator Mapping" block follows\n' ...
    'the controller channel ordering. The\n' ...
    'loop polarity (ACT_MAP_SIGN) is measured. Demands is a test input.']));

save_system(root);
close_system(root, 0);
fprintf('\nwrote %s\n', outFile);
end
