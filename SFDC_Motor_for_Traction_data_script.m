%% DC Motor Control Tramway - Project
clear all
close all


%% Data
V_n=600;     % Line voltage
w_n=314;     % Rated speed [rad/s]
v_r=60;      % Rated speed [km/h]
eta=0.9;     % Efficiency
tau_a=10e-3; % Armature time constant
tau_e=1;     % Excitation time constant
Ve=120;      % Excitation rated voltage
Ie=1;        % Excitation rated current
m_p=80;      % Mass of std passenger
n_p=200;     % Number of passengers
m_t=10000;   % Tramway mass
t_a=25;      % time of acceleration
%% Parameters identification
M=m_t+m_p*n_p;         % Total mass
v_max=v_r*1000/3600;    % Rated speed [m/s]
a=v_max/t_a;           % Average acceleration
F_trac=M*a;            % Traction force
P_trac=F_trac*v_max;   % Traction power
P_tot=P_trac+P_trac/3; % Rated power
P_el=P_tot/eta;        % Electrical power absorbed
T_n=P_tot/w_n;         % Rated torque
I_n=P_el/V_n;          % Rated current
K=T_n/(I_n*Ie);        % Torque/e.m.f. coefficient
Ra=(P_el-P_tot)/I_n^2; % Armature resistance
La=Ra*tau_a;           % Armature inductance
En=eta*V_n;            % Rated e.m.f.
Re=Ve/Ie;              % Excitation resistance
Le=Re*tau_e;           % Excitation inductance
J=M*v_max^2/w_n^2;     % Equivalent inertia
beta=P_trac/3/w_n^2;   % Damping/friction factor

%% Controllers

s=tf('s');

% Armature
G_ia=1/(Ra+s*La);
tauG_ia=La/Ra;
TaG_ia=5*tauG_ia;

Tad=TaG_ia/2; % final time at least half of settling time
wc_ia=5/Tad;
kp_ia=wc_ia*La;
ki_ia=wc_ia*Ra;
R_ia=kp_ia+ki_ia/s; % Armature current controller

% Excitation
G_ie=1/(Re+s*Le);
tauG_ie=Le/Re;
TaG_ie=5*tauG_ie;

wc_ie=wc_ia/10;
kp_ie=wc_ie*Le;
ki_ie=wc_ie*Re;
R_ie=kp_ie+ki_ie/s; % Excitation current controller

% Mechanical
G_m=1/(beta+s*J);
tauG_m=J/beta;
TaG_m=5*tauG_m;

wc_w=wc_ia/200;
kp_w=wc_w*J;
ki_w=wc_w*beta;
R_w=kp_w+ki_w/s; % Speed controller

%% Conversion factors

% [rad/s]->[m/s]
F1=v_max/w_n;

% [m/s]->[rad/s]
F2=w_n/v_max;

% [km/h]->[rad/s]
F3=w_n/v_r;