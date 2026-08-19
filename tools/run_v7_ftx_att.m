function R = run_v7_ftx_att(outFile, figFile)
%RUN_V7_FTX_ATT  Validate pitch-attitude hold with sensor feedback in the loop.
%
% Success here means a bounded, useful flight interval consistent with the
% demonstrated aircraft envelope: straight attitude hold and only small bank
% commands.  It does not assert indefinite navigation-grade estimation.

wb = fileparts(fileparts(mfilename('fullpath')));
if nargin<1||isempty(outFile), outFile = fullfile(wb,'out','FTX_ATT_RESULT.txt'); end
if nargin<2||isempty(figFile), figFile = fullfile(wb,'out','ftx_att_sensor_loop.png'); end
toolsDir = fullfile(wb,'tools');
mdl = 'DroneModelv7_FTX_ATT';
addpath(genpath(fullfile(wb,'simulink','DroneModelv7')));
addpath(genpath(fullfile(wb,'simulink',mdl))); addpath(toolsDir);
evalin('base',sprintf('run(''%s'')',fullfile(toolsDir,'uav_setup_v7_ftx_att.m')));
load_system(fullfile(wb,'simulink',mdl,[mdl '.slx']));
set_param(mdl,'SaveOutput','on','SaveFormat','Dataset','SaveTime','on');

th0 = evalin('base','FTX_THETA_HOLD');
% name, bank step deg, pitch step deg, feedback blend, stop time.  The model
% starts at t=10 s, so stop 40 is a 30 s flight interval and stop 70 is 60 s.
cases = {
    'straight hold, truth',       0, 0, [0;0;0;0], 40
    'straight hold, estimated',   0, 0, [1;1;1;1], 40
    'pitch +2 deg, estimated',    0, 2, [1;1;1;1], 40
    'bank +2 deg, estimated',     2, 0, [1;1;1;1], 40
    'bank +5 deg, estimated',     5, 0, [1;1;1;1], 40
    'long straight, estimated',   0, 0, [1;1;1;1], 70
};

R = struct([]);
for k=1:size(cases,1)
    assignin('base','DEMAND0',[0;th0;0]);
    assignin('base','DEMAND1',[cases{k,2}*pi/180;th0+cases{k,3};0]);
    % The model starts at t=10 s.  Put the command at t=15 s so the first five
    % seconds are a real pre-step baseline.
    assignin('base','T_STEP',[15;15;15]);
    assignin('base','EST_BLEND',cases{k,4});
    set_param(mdl,'StopTime',num2str(cases{k,5}));
    try
        so=sim(mdl,'ReturnWorkspaceOutputs','on');
        S=unpack(so); S.ok=true; S.name=cases{k,1};
        S.phi_cmd=cases{k,2}; S.theta_cmd=th0+cases{k,3};
    catch e
        S=struct('ok',false,'name',cases{k,1},'err',e.message, ...
            'phi_cmd',cases{k,2},'theta_cmd',th0+cases{k,3});
    end
    if isempty(R), R=S; else, R(end+1)=S; end %#ok<AGROW>
end

L={};
    function p(fmt,varargin)
        s=sprintf(fmt,varargin{:}); L{end+1}=s; fprintf('%s\n',s);
    end
p('FT Explorer pitch-attitude hold - sensor in the loop');
p('==========================================================================');
p('Controller: theta attitude outer loop; alpha monitored, not fed back');
p('Propulsion: FT Power Pack B Radial v2 + HQ 9x4.5 CCW, %.2f W effective', ...
  evalin('base','FTX_PROP_POWER'));
p('Control %.0f Hz; estimator %.0f Hz; surface limit +-%g deg', ...
  1/evalin('base','Tc'),1/evalin('base','EST_TS'),evalin('base','RT_SURF_LIM'));
p('');
p('%-28s %5s %7s %7s %7s %7s %7s %6s %6s %6s %6s %7s %7s', ...
  'case','dur','phi','theta','th err','alpha','beta','|da|','|de|','|dr|','dV','dh','status');
p('%s',repmat('-',1,128));
for k=1:numel(R)
    S=R(k);
    if ~S.ok, p('%-28s FAILED: %s',S.name,S.err); continue; end
    valid=S.finite && S.alpha_max<=8.0001 && S.alpha_min>=-8.0001 && S.beta_abs<=10.0001;
    p('%-28s %5.0f %7.2f %7.2f %+7.2f %7.2f %7.2f %6.2f %6.2f %6.2f %+6.2f %+7.2f %7s', ...
      S.name,S.duration,S.phi_end,S.theta_end,S.theta_end-S.theta_cmd, ...
      S.alpha_abs,S.beta_abs,S.da_max,S.de_max,S.dr_max,S.speed_change, ...
      S.alt_change,ternary(valid,'BOUNDED','OUT'));
end
p('');
p('Estimator drift and useful interval:');
p('%-28s %10s %10s %10s %10s %10s', ...
  'case','bias final','bias max','to 2 deg','to 5 deg','band time');
for k=1:numel(R)
    S=R(k); if ~S.ok, continue; end
    p('%-28s %+10.2f %10.2f %10s %10s %10.1f',S.name,S.theta_est_err_end, ...
      S.theta_est_err_max,timeText(S.drift2),timeText(S.drift5),S.bound_duration);
end
p('');
p('BOUNDED means finite for the reported duration and inside the aerodynamic table band:');
p('alpha -8..+8 deg and beta -10..+10 deg. It does not mean drift-free');
p('indefinite operation or flight-envelope robustness.');
p('==========================================================================');

txt=strjoin(L,newline);
if ~exist(fileparts(outFile),'dir'),mkdir(fileparts(outFile));end
fid=fopen(outFile,'w');fprintf(fid,'%s\n',txt);fclose(fid);
save(fullfile(wb,'out','FTX_ATT_RESULT.mat'),'R');

% Plot the full-estimate +2 degree bank case.
ix=find(strcmp({R.name},'bank +2 deg, estimated'),1);
if ~isempty(ix)&&R(ix).ok
    S=R(ix); f=figure('Visible','off','Color','w','Position',[100 100 1400 900]);
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
    nexttile; plot(S.t,S.phi,'LineWidth',1.4); hold on; yline(S.phi_cmd,'--');
    ylabel('bank (deg)'); grid on; title('Small-bank command');
    nexttile; plot(S.t,S.theta,'LineWidth',1.4); hold on; plot(S.t_est,S.theta_est,'--','LineWidth',1.1); yline(S.theta_cmd,':');
    ylabel('pitch (deg)'); legend('truth','estimate','command','Location','best'); grid on; title('Pitch-attitude hold');
    nexttile; plot(S.t,S.alpha,'LineWidth',1.2); hold on; yline(8,'r:'); yline(-8,'r:');
    ylabel('alpha (deg)'); grid on; title('Aerodynamic envelope');
    nexttile; plot(S.t,S.beta,'LineWidth',1.2); hold on; yline(10,'r:'); yline(-10,'r:');
    ylabel('beta (deg)'); grid on; title('Sideslip');
    nexttile; plot(S.t_ctrl,[S.da S.de S.dr],'LineWidth',1.1); ylabel('deflection (deg)'); xlabel('time (s)');
    legend('aileron','elevator','rudder','Location','best'); grid on; title('Surface commands');
    nexttile; yyaxis left; plot(S.t,S.altitude,'LineWidth',1.2); ylabel('altitude (m)');
    yyaxis right; plot(S.t,S.speed,'LineWidth',1.2); ylabel('speed (m/s)'); xlabel('time (s)'); grid on; title('Constant-power flight interval');
    sgtitle('FT Explorer: powered sensor-in-loop pitch-attitude hold');
    exportgraphics(f,figFile,'Resolution',180); close(f);
end
close_system(mdl,0);
end

function S=unpack(so)
d=so.yout;
[S.t,eul]=signal(d,2); [~,pqr]=signal(d,1);
[~,S.alpha]=signal(d,3); [~,S.beta]=signal(d,4);
[S.t_ctrl,S.da_cmd]=signal(d,7); [~,S.de_cmd]=signal(d,8); [~,S.dr_cmd]=signal(d,16);
[~,S.da]=signal(d,14); [~,S.de]=signal(d,15); [~,S.dr]=signal(d,17);
[S.t_est,ee]=signal(d,13); [~,xe]=signal(d,18); [~,vb]=signal(d,19); [~,S.thrust]=signal(d,20);
S.phi=eul(:,1)*180/pi; S.theta=eul(:,2)*180/pi;
S.theta_est=ee(:,2)*180/pi; S.alpha=S.alpha*180/pi; S.beta=S.beta*180/pi;
S.altitude=-xe(:,3); S.speed=sqrt(sum(vb.^2,2));
S.speed_change=S.speed(end)-S.speed(1);
S.alt_change=S.altitude(end)-S.altitude(1);
S.thrust_mean=mean(S.thrust);
w=S.t>=S.t(end)-2;
S.phi_end=mean(S.phi(w)); S.theta_end=mean(S.theta(w));
theta_at_est=interp1(S.t,S.theta,S.t_est,'linear');
est_err=S.theta_est-theta_at_est;
% The estimator state comes out of a Unit Delay whose initial output is zero,
% so sample 1 always reports the whole initial attitude as "error" - even in
% the truth-feedback case, where the estimator is not in the loop at all.
% That is a startup transient, not a bias. Measure drift from sample 2 on.
if numel(est_err)>1, i0=2; else, i0=1; end
err_d=est_err(i0:end); t_d=S.t_est(i0:end);
S.theta_est_err_max=max(abs(err_d));
we=t_d>=t_d(end)-2;
S.theta_est_err_end=mean(err_d(we));
S.drift2=firstCrossing(t_d,abs(err_d),2);
S.drift5=firstCrossing(t_d,abs(err_d),5);
S.alpha_max=max(S.alpha); S.alpha_min=min(S.alpha); S.alpha_abs=max(abs(S.alpha));
S.beta_abs=max(abs(S.beta)); S.da_max=max(abs(S.da)); S.de_max=max(abs(S.de)); S.dr_max=max(abs(S.dr));
S.alt_loss=S.altitude(1)-S.altitude(end);
S.finite=all(isfinite([S.phi;S.theta;S.alpha;S.beta;S.da;S.de;S.dr]));
S.duration=S.t(end)-S.t(1);
bad=~isfinite(S.alpha)|~isfinite(S.beta)|S.alpha>8|S.alpha<-8|abs(S.beta)>10;
ib=find(bad,1);
if isempty(ib),S.bound_duration=S.duration;else,S.bound_duration=S.t(ib)-S.t(1);end
S.q=pqr(:,2);
end

function [t,x]=signal(d,k)
v=d.getElement(k).Values; t=v.Time(:); x=squeeze(v.Data);
% MATLAB Function and vector outports often log as 3x1xN; ordinary root
% vectors log as Nx3. Normalize both to samples-by-channel.
if isvector(x)
    x=x(:);
elseif size(x,1)~=numel(t) && size(x,2)==numel(t)
    x=x.';
end
end

function t=firstCrossing(ts,x,limit)
i=find(x>=limit,1);
if isempty(i),t=NaN;else,t=ts(i)-ts(1);end
end

function s=timeText(t)
if isnan(t),s='--';else,s=sprintf('%.1f s',t);end
end

function o=ternary(c,a,b),if c,o=a;else,o=b;end,end
