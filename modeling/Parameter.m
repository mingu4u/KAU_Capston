clc;

global trim;
global motor;
global inertia;
global Ts;
global initial_condition

%% 시뮬레이션 Setting
Ts                     = 0.001;  % [sec]
initial_phi            = 0;     % [deg]
initial_phi_dot        = 0;      % [deg/sec]
initial_zf             = 1;    % [m]
initial_zf_dot         = 0.0;    % [m/s]
Sampling_time_angle    = 0.005;  % [s]
Sampling_time_distance = 0.01;  % [s]

%% 관성 파라미터
m  = 1.45;    % [kg]
Ix = 0.015;   % [kg*m2]
l  = 0.28;    % [m]
c  = 0.00019; % [kg*m2/s]
cg = 0.004;   % [m]

%% 모터 파라미터 (13inch)
kt =  2.4344e-05;
a  = 0.90555;
b  = -940.17;
ActuatorDelay = 0.2;

%% 구조체 생성

inertia.mass = m;
inertia.Ix   = Ix;
inertia.l    = l;
inertia.c    = c;
inertia.cg   = cg;

motor.kt = kt;
motor.a  = a;
motor.b  = b;

initial_condition.phi     = initial_phi;
initial_condition.phi_dot = initial_phi_dot;
initial_condition.zf      = initial_zf;
initial_condition.zf_dot  = initial_zf_dot;

%% trim pwm 계산

syms x
eqn = 2*kt*(a*x+b)^2 == m*9.81;
trim.pwm = (double((solve(eqn,x))))-100;

for i = 1:length(trim.pwm)
    if (trim.pwm(i)<1000)
        trim.pwm(i) = [];
        break;
    end
end
% trim.pwm = 1580;
trim.omega = a*trim.pwm + b;

clear x eqn i kt a b l m Ix c cg initial_phi initial_phi_dot initial_zf initial_zf_dot