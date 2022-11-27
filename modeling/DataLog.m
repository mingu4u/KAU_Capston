clc; close all;

%% 저장하고자 하는 시뮬레이션 결과를 csv 파일로 저장하는 코드

time_state = out.States.time;
phi        = out.States.signals(1).values;
phi_dot    = out.States.signals(2).values;
zf         = out.States.signals(3).values;
zf_dot     = out.States.signals(4).values;

time_height = out.Height_PID.time;
Height_P    = out.Height_PID.signals(1).values;
Height_D    = out.Height_PID.signals(2).values;
Height_I    = out.Height_PID.signals(3).values;
Height_T    = out.Height_PID.signals(4).values;

time_roll  = out.Roll_PID.time;
Roll_P     = out.Roll_PID.signals(1).values;
Roll_D     = out.Roll_PID.signals(2).values;
Roll_I     = out.Roll_PID.signals(3).values;
Roll_T     = out.Roll_PID.signals(4).values;

time_pwm   = out.Total_pwm.time;
Left_pwm   = out.Total_pwm.signals(1).values;
Right_pwm  = out.Total_pwm.signals(2).values;

states     = [time_state, phi, phi_dot, zf, zf_dot];
height_pid = [time_height, Height_P, Height_I, Height_D, Height_T];
roll_pid   = [time_roll, Roll_P, Roll_I, Roll_D, Roll_T];
pwm        = [time_pwm, Left_pwm, Right_pwm];

filename = 'data_case1.csv';
writematrix(states,filename);

%% 비교대상인 두 경우에 대한 시뮬레이션 Data.csv 불러오는 코드

close all;

data_case1 = load ('data_case1.csv');
% data_case2 = load ('data_case2.csv');
% data_case3 = load ('data_case3.csv');
% data_case4 = load ('data_case4.csv');

plot(data_case1(:,1), data_case1(:,2));
legend('Gain = 1','Gain = 5','Gain = 10','Gain = 15');
xlabel('time [sec]'); ylabel('고도 [m]'); 
title('추력 보상 게인에 따른 응답 비교');

%% 입력 비교
close all; clc;
addpath(genpath('C:\Users\wngud\OneDrive - 한국항공대학교\2022년\학부연구생\Project_Matlab\2DOF_Bicopter\Data'));
FileOrder = '_20221122_4';
File_ext  = '.csv';
Filename  = strcat('real_data', FileOrder, File_ext);
RealData  = load(Filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% 데이터 변수화 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt_angle       = RealData(:,1);
dt_distance    = RealData(:,2);
NowAngle       = RealData(:,3);
NowGyro        = RealData(:,4);
P_Angle        = RealData(:,5);
P_Gyro         = RealData(:,6);
I_Gyro         = RealData(:,7);
D_Gyro         = RealData(:,8);
D_LPF_Gyro     = RealData(:,9);
NowDistance    = RealData(:,10);
P_Distance     = RealData(:,11);
I_Distance     = RealData(:,12);
D_Distance     = RealData(:,13);
D_LPF_Distance = RealData(:,14);
pwm_left       = RealData(:,15);
% pwm_right      = RealData(:,16);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 시간 데이터 %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

timearray = zeros(length(RealData),1);
for i = 2:length(RealData)
    timearray(i,1) = timearray(i-1) + dt_angle(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% 명령 추종성능 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig1 = figure(1);
subplot(2,1,1) % 현재 각도, 명령 각도
hold on; grid on;
plot(timearray,NowAngle); yline(0);
hold off;
title('Angle Control Performance');
legend('Now Angle','CMD Angle');
xlabel('time [sec]'); ylabel('Angle [deg]');
subplot(2,1,2) % 현재 각속도, 명령 각속도
hold on; grid on;
plot(timearray,NowGyro); plot(timearray,P_Angle);
hold off;
title('Gyro Control Performance')
legend('Now Gyro','CMD Gyro');
xlabel('time [sec]'); ylabel('Angle Rate [deg/s]');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Gyro 제어입력 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig2 = figure(2);
subplot(2,2,1)
plot(timearray,P_Gyro);
xlabel('time [sec]'); title('P Gyro'); grid on;
subplot(2,2,2)
plot(timearray,I_Gyro); 
xlabel('time [sec]'); title('I Gyro'); grid on;
subplot(2,2,3)
hold on; grid on;
plot(timearray,D_Gyro); 
plot(timearray,D_LPF_Gyro);
xlabel('time [sec]'); title('D Gyro'); grid on; legend('LPF X','LPF O');
hold off;
subplot(2,2,4);
plot(timearray,P_Gyro+I_Gyro+D_Gyro);
xlabel('time [sec]'); title('Total Input'); grid on;

fig3 = figure(3);
subplot(1,2,1)
Fourier(NowAngle);
title('Angle FFT'); xlabel('Frequency');
subplot(1,2,2)
Fourier(NowGyro);
title('Gyro FFT'); xlabel('Frequency');

saveas(fig1,strcat('fig1',FileOrder,'.jpg'))

function Fourier(data)
    Y  = fft(data);
    L  = length(data);
    Fs = 1/0.003;
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    
    f = Fs*(0:floor(L/2))/L;
    plot(f,P1)
end

