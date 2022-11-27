close all; clc;

%          Ts  = 3; % [sec]
%         Time = [0, Ts]';
% Height_Input = [1.45*9.81, 1.45*9.81]';
%   Roll_Input = [0, 0]';
%           UT = [Time, Height_Input, Roll_Input];

%         simout = sim('linearization',Ts,[],UT);

     lin_model = linmod('linearization');

    clear Ts Time Height_Input Roll_Input UT U

    save('LINEAR_MODEL','lin_model');

    s = tf('s');
    tol = 0.001;

    A = lin_model.a;
    B = lin_model.b;
    C = lin_model.c;
    D = lin_model.d;

%% Roll Vel signal -> phi_dot
close all;
[num1, den1] = ss2tf(A,B,C,D,2);
Gstar1 = minreal(tf(num1(2,:),den1),tol)
[num_G1, den_G1] = tfdata(Gstar1);

% RollRateP = 1.5
% RollRateI = 0.04
% RollRateD = 0.04

a = 100/1.4;
b = 1/1.4;
K = 1;

Controller1 = (a*s^2/(0.05*s+1) + s + b)/s;

openloop_TF1 = Controller1 * Gstar1;
[num_tol1, den_tol1] = tfdata(openloop_TF1);
num_tol1 = cell2mat(num_tol1);
den_tol1 = cell2mat(den_tol1);
gain_root1 = roots(K*num_tol1+den_tol1);

figure(1)
rlocus(openloop_TF1)
hold on;
plot(real(gain_root1),imag(gain_root1),'k*');
hold off;
axis equal
title('RollRate PID')

Kpvel2 = K;
Kivel2 = K * b;
Kdvel2 = K * a;

PID = [Kpvel2, Kivel2, Kdvel2]

%% Height Signal -> Height

close all; clc;
[num3, den3] = ss2tf(A,B,C,D,1);
Gstar3 = minreal(tf(num3(3,:),den3),tol)
[num_G3, den_G3] = tfdata(Gstar3);

a = 1/60;
b = 0.5/60;
K = 100;

Controller3 = (a*s^2/(0.01*0+1) + s + b)/s;

openloop_TF3 = Controller3 * Gstar3
[num_tol3, den_tol3] = tfdata(openloop_TF3);
num_tol3 = cell2mat(num_tol3);
den_tol3 = cell2mat(den_tol3);
gain_root3 = roots(K*num_tol3+den_tol3);

figure(3)
rlocus(openloop_TF3)
hold on;
plot(real(gain_root3),imag(gain_root3),'k*');
hold off;
axis equal
title('RollRate PID')

Kpvel2 = K;
Kivel2 = K * b;
Kdvel2 = K * a;

PID = [Kpvel2, Kivel2, Kdvel2]


