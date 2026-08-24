%% ========================================================================
%  PMSM FIELD-ORIENTED CONTROL (FOC)  -  built from first principles
%  Clarke/Park transforms, cascaded PI current + speed loops, dq motor model
%  ------------------------------------------------------------------------
%  Author : Rohit
%  Tool   : MATLAB R2026a  (BASE MATLAB ONLY - no toolboxes required)
%
%  Field-Oriented Control makes an AC PMSM behave like a DC motor by
%  regulating current in the rotor (dq) reference frame:
%     * id  -> 0        (no flux-weakening; all current makes torque)
%     * iq  -> torque   (set by the outer speed loop)
%  This script implements the FULL cascade and simulates a speed step plus
%  a load-torque disturbance, from the motor's differential equations up.
%
%  Control structure
%     speed_ref -> [speed PI] -> iq_ref ->|
%                                 id_ref=0 ->[current PI x2]-> vd,vq
%                                 -> inverse Park -> (inverter) -> motor
%     motor abc currents -> Clarke -> Park -> id,iq  (feedback)
%
%  Deliverables produced
%    images\foc_speed_tracking.png   - rotor speed follows the reference
%    images\foc_dq_currents.png      - id held at 0, iq = torque current
%    images\foc_phase_currents.png   - resulting balanced 3-phase currents
%    images\foc_torque.png           - electromagnetic vs load torque
%    images\foc_dashboard.png        - all four on one panel
%    results\foc_results.txt          - motor params + tracking metrics
% =========================================================================

clear; clc; close all;
rng(2026);
set(0,'DefaultFigureVisible','off');
here=fileparts(mfilename('fullpath'));
imgdir=fullfile(here,'images'); resdir=fullfile(here,'results');
if ~exist(imgdir,'dir'),mkdir(imgdir);end
if ~exist(resdir,'dir'),mkdir(resdir);end

%% ---------------- PMSM parameters (small servo motor) ------------------
Rs   = 0.5;        % stator resistance (ohm)
Ld   = 4e-3;       % d-axis inductance (H)
Lq   = 4e-3;       % q-axis inductance (H)  (surface PMSM: Ld=Lq)
psim = 0.10;       % permanent-magnet flux linkage (Wb)
P    = 4;          % pole pairs
J    = 1.0e-3;     % rotor inertia (kg.m^2)
Bvis = 5e-4;       % viscous friction (N.m.s)
Vdc  = 300;        % DC-bus voltage (V) -> phase voltage limit
Vmax = Vdc/sqrt(3);

%% ---------------- simulation timing ------------------------------------
Ts   = 2e-5;               % 50 kHz control/sim step
Tend = 0.30;               % s
t    = 0:Ts:Tend;  N=numel(t);

% references / disturbance
wref_rpm = 1500*(t>=0.01);                 % speed step to 1500 rpm at 10 ms
wref     = wref_rpm*2*pi/60;               % mechanical rad/s
TL       = 2.0*(t>=0.18);                  % 2 N.m load applied at 180 ms

%% ---------------- PI gains (pole-placement style) ----------------------
% Current loop bandwidth ~ 2*pi*300 rad/s ; plant = 1/(Ls + Rs)
wc_i = 2*pi*300;
Kp_i = Ld*wc_i;   Ki_i = Rs*wc_i;
% Speed loop bandwidth ~ 2*pi*15 rad/s
wc_s = 2*pi*15;
Kt   = 1.5*P*psim;                         % torque constant Te = Kt*iq
Kp_s = J*wc_s/Kt;  Ki_s = Bvis*wc_s/Kt + J*wc_s^2/(Kt*10);

iq_max = 12;                               % current limit (A)

%% ---------------- state + logs -----------------------------------------
id=0; iq=0; wm=0; th=0;                     % motor states (dq currents, speed, mech angle)
ei_d=0; ei_q=0; ei_w=0;                     % PI integrators
ID=zeros(1,N); IQ=zeros(1,N); WM=zeros(1,N); TE=zeros(1,N);
IDREF=zeros(1,N); IQREF=zeros(1,N); IA=zeros(1,N); IB=zeros(1,N); IC=zeros(1,N);

for k=1:N
    the = P*th;                            % electrical angle

    % ---- outer speed PI -> iq_ref ----
    ew   = wref(k) - wm;
    ei_w = ei_w + ew*Ts;
    iqref= Kp_s*ew + Ki_s*ei_w;
    iqref= max(min(iqref,iq_max),-iq_max); % saturate (with clamp anti-windup)
    if abs(iqref)>=iq_max, ei_w = ei_w - ew*Ts; end
    idref= 0;                              % surface PMSM: id* = 0

    % ---- inner current PI (dq) with cross-coupling decoupling ----
    ed=idref-id; eq=iqref-iq;
    ei_d=ei_d+ed*Ts; ei_q=ei_q+eq*Ts;
    we = P*wm;
    vd = Kp_i*ed + Ki_i*ei_d - we*Lq*iq;               % decoupling feedforward
    vq = Kp_i*eq + Ki_i*ei_q + we*(Ld*id + psim);
    % voltage limit (circular)
    vm=hypot(vd,vq); if vm>Vmax, vd=vd*Vmax/vm; vq=vq*Vmax/vm; end

    % ---- PMSM electrical dynamics (dq), forward Euler ----
    did=(vd - Rs*id + we*Lq*iq)/Ld;
    diq=(vq - Rs*iq - we*(Ld*id + psim))/Lq;
    id=id+did*Ts; iq=iq+diq*Ts;

    % ---- torque + mechanical dynamics ----
    Te=1.5*P*(psim*iq + (Ld-Lq)*id*iq);
    dwm=(Te - TL(k) - Bvis*wm)/J;
    wm=wm+dwm*Ts; th=th+wm*Ts;

    % ---- reconstruct 3-phase currents (inverse Park/Clarke) for plotting ----
    ia= id*cos(the)-iq*sin(the);
    ib= id*cos(the-2*pi/3)-iq*sin(the-2*pi/3);
    ic= id*cos(the+2*pi/3)-iq*sin(the+2*pi/3);

    ID(k)=id; IQ(k)=iq; WM(k)=wm; TE(k)=Te;
    IDREF(k)=idref; IQREF(k)=iqref; IA(k)=ia; IB(k)=ib; IC(k)=ic;
end
WM_rpm=WM*60/(2*pi);

%% ---------------- Figure : speed tracking ------------------------------
f1=figure('Color','w','Position',[100 100 820 420]); theme(f1,'light');
plot(t*1e3, wref_rpm,'k--','LineWidth',1.6); hold on;
plot(t*1e3, WM_rpm,'b','LineWidth',1.5);
xline(180,'r:','load step','LineWidth',1.2,'LabelVerticalAlignment','bottom');
grid on; box on; xlabel('time (ms)'); ylabel('speed (rpm)');
title('FOC speed tracking - step to 1500 rpm, 2 N.m load at 180 ms');
legend('reference','rotor speed','Location','southeast');
exportgraphics(f1,fullfile(imgdir,'foc_speed_tracking.png'),'Resolution',150);

%% ---------------- Figure : dq currents ---------------------------------
f2=figure('Color','w','Position',[100 100 820 420]); theme(f2,'light');
plot(t*1e3, IQ,'b','LineWidth',1.3); hold on;
plot(t*1e3, ID,'r','LineWidth',1.3);
plot(t*1e3, IQREF,'b--','LineWidth',0.8);
grid on; box on; xlabel('time (ms)'); ylabel('current (A)');
title('Rotor-frame currents:  i_d held at 0,  i_q = torque current');
legend('i_q','i_d','i_q ref','Location','northeast');
exportgraphics(f2,fullfile(imgdir,'foc_dq_currents.png'),'Resolution',150);

%% ---------------- Figure : 3-phase currents (zoom) ---------------------
f3=figure('Color','w','Position',[100 100 820 420]); theme(f3,'light');
idx=(t>=0.20)&(t<=0.24);
plot(t(idx)*1e3, IA(idx),'r', t(idx)*1e3, IB(idx),'g', t(idx)*1e3, IC(idx),'b','LineWidth',1.2);
grid on; box on; xlabel('time (ms)'); ylabel('current (A)');
title('Balanced 3-phase stator currents produced by FOC (zoom)');
legend('i_a','i_b','i_c','Location','northeast');
exportgraphics(f3,fullfile(imgdir,'foc_phase_currents.png'),'Resolution',150);

%% ---------------- Figure : torque --------------------------------------
f4=figure('Color','w','Position',[100 100 820 420]); theme(f4,'light');
plot(t*1e3, TE,'b','LineWidth',1.3); hold on;
plot(t*1e3, TL,'k--','LineWidth',1.4);
grid on; box on; xlabel('time (ms)'); ylabel('torque (N.m)');
title('Electromagnetic torque tracks load demand');
legend('electromagnetic torque T_e','load torque T_L','Location','northeast');
exportgraphics(f4,fullfile(imgdir,'foc_torque.png'),'Resolution',150);

%% ---------------- Figure : dashboard -----------------------------------
f5=figure('Color','w','Position',[100 100 1000 720]); theme(f5,'light');
subplot(2,2,1); plot(t*1e3,wref_rpm,'k--',t*1e3,WM_rpm,'b','LineWidth',1.3);
grid on;box on;title('Speed (rpm)');xlabel('ms');legend('ref','act','Location','se');
subplot(2,2,2); plot(t*1e3,IQ,'b',t*1e3,ID,'r','LineWidth',1.2);
grid on;box on;title('dq currents (A)');xlabel('ms');legend('i_q','i_d');
subplot(2,2,3); idx=(t>=0.20)&(t<=0.24);
plot(t(idx)*1e3,IA(idx),'r',t(idx)*1e3,IB(idx),'g',t(idx)*1e3,IC(idx),'b','LineWidth',1);
grid on;box on;title('Phase currents (A)');xlabel('ms');
subplot(2,2,4); plot(t*1e3,TE,'b',t*1e3,TL,'k--','LineWidth',1.2);
grid on;box on;title('Torque (N.m)');xlabel('ms');legend('T_e','T_L');
sgtitle('PMSM Field-Oriented Control - simulation dashboard');
exportgraphics(f5,fullfile(imgdir,'foc_dashboard.png'),'Resolution',150);

%% ---------------- metrics + log ----------------------------------------
% settling: time after step (10ms) to reach within 2% of 1500 rpm
band=0.02*1500; aft=find(t>=0.01);
reached=aft(find(abs(WM_rpm(aft)-1500)<=band,1));
tset=(t(reached)-0.01)*1e3;
% speed dip at load step
dipidx=(t>=0.18)&(t<=0.22); dip=1500-min(WM_rpm(dipidx));
fid=fopen(fullfile(resdir,'foc_results.txt'),'w');
fprintf(fid,'PMSM Field-Oriented Control (base MATLAB)\n\n');
fprintf(fid,'Motor: Rs=%.2f ohm  Ld=Lq=%.1f mH  psi_m=%.3f Wb  P=%d  J=%.1e\n',Rs,Ld*1e3,psim,P,J);
fprintf(fid,'Control: current-loop BW=300 Hz, speed-loop BW=15 Hz\n\n');
fprintf(fid,'Speed step 0 -> 1500 rpm:\n');
fprintf(fid,'  2%%%% settling time        = %.1f ms\n',tset);
fprintf(fid,'  steady-state i_d          = %.3f A  (target 0)\n',mean(ID(t>0.15&t<0.17)));
fprintf(fid,'  i_q at 2 N.m load         = %.3f A  (expected Te/Kt=%.3f)\n',mean(IQ(t>0.25)),2/Kt);
fprintf(fid,'  speed dip at load step    = %.1f rpm\n',dip);
fclose(fid);
disp('DONE - PMSM FOC figures in .\images');
