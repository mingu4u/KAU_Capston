close all; clc;

lin_model = linmod('linearization');

s = tf('s');
tol = 0.001;

A = lin_model.a;
B = lin_model.b;
C = lin_model.c;
D = lin_model.d;

%% 각도 컨트롤러

close all;
[num1, den1] = ss2tf(A,B,C,D,2);
Gstar1 = minreal(tf(num1(2,:),den1),tol)
[num_G1, den_G1] = tfdata(Gstar1);

%% 고도 컨트롤러

close all;
[num3, den3] = ss2tf(A,B,C,D,1);
Gstar3 = minreal(tf(num3(3,:),den3),tol)
[num_G3, den_G3] = tfdata(Gstar3);
