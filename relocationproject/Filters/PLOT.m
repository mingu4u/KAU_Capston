% clc;
% clear; close all;
%% Xsens 
data_xsens = load('xsens.csv');
time_xsens = 10^-4*(data_xsens(:,2)-data_xsens(1,2));
Xsens_angleX = data_xsens(:,3);
Xsens_angleY = data_xsens(:,4);
Xsens_angleZ = data_xsens(:,5);


%% complementary filter
data_mpu = load('CF_data.csv');
dt_mpu = data_mpu(:,1);
Gyro_angvelX = data_mpu(:,2);
Gyro_angvelY = data_mpu(:,6);
Gyro_angvelZ = data_mpu(:,7);
AccX = data_mpu(:,3);
AccY = data_mpu(:,4);
AccZ = data_mpu(:,5);

for k = 1:length(dt_mpu)
    Acc_angleX(k,1) = atan2d(AccY(k),sqrt((AccX(k))^2+(AccZ(k))^2));
    Acc_angleY(k,1) = atan2d(AccX(k),sqrt((AccX(k))^2+(AccZ(k))^2));
    Acc_angleZ(k,1) = 0;
end

comp_angleX(1) = Acc_angleX(1,1);
comp_angleY(1) = Acc_angleY(1,1);
comp_angleZ(1) = 0;

time_mpu(1) = dt_mpu(1);

for j =1:length(dt_mpu)-1
   time_mpu(j+1) =  dt_mpu(j+1)+time_mpu(j);
end

%check init degree
for i = 1:100 
    sumGyvelX(i) = Gyro_angvelX(i);
    sumGyvelY(i) = Gyro_angvelY(i);
    sumGyvelZ(i) = Gyro_angvelZ(i);
end

averGyvelX = sum(sumGyvelX)/i;
averGyvelY = sum(sumGyvelY)/i;
averGyvelZ = sum(sumGyvelZ)/i;

p = Gyro_angvelX - averGyvelX;
q = Gyro_angvelY - averGyvelY;
r = Gyro_angvelZ - averGyvelZ;

ALPHA = 0.96;

for i = 1 : length(dt_mpu)-1
    
    matrix = [1 sind(comp_angleX(i))*tand(comp_angleY(i)) cosd(comp_angleX(i))*tand(comp_angleY(i));
              0 cosd(comp_angleY(i)) -sind(comp_angleX(i));
              0 sind(comp_angleX(i))*secd(comp_angleY(i)) cosd(comp_angleX(i))*secd(comp_angleY(i))];
    
    Euler_rate = matrix * [p(i) q(i) r(i)]';
    
      phi_rate(i) = Euler_rate(1);
    pitch_rate(i) = Euler_rate(2);
      yaw_rate(i) = Euler_rate(3);
      
    tmp_angleX(i+1) = comp_angleX(i) + phi_rate(i) * dt_mpu(i);
    tmp_angleY(i+1) = comp_angleY(i) + pitch_rate(i) * dt_mpu(i);
    tmp_angleZ(i+1) = comp_angleZ(i) + yaw_rate(i) * dt_mpu(i);

    comp_angleX(i+1) = ALPHA * tmp_angleX(i+1) + (1.0-ALPHA) * Acc_angleX(i);
    comp_angleY(i+1) = ALPHA * tmp_angleY(i+1) + (1.0-ALPHA) * Acc_angleY(i);
    comp_angleZ(i+1) = ALPHA * tmp_angleZ(i+1) + (1.0-ALPHA) * Acc_angleZ(i);
end

%% Kalmanfilter
%     Q = [0.2 0; 0 0.2];
%     R = 0.05;

    X = [0;sqrt(2.1)];
    P = 10*eye(2);
    X1 = [0;sqrt(2.1)];
    P1 = 10*eye(2);
        R = 0.03;
 Q = [0.09 0; 0 0.18];
    H = [1 0];

    Xsaved = X;
    Psaved = P;
    X1saved = X1;
    P1saved = P1;
    
    input = Gyro_angvelX;
    input2 = Gyro_angvelY;
    Sdata(:,1) = Acc_angleX;
    Sdata2(:,1) = Acc_angleY;

for i = 1 : length(dt_mpu)-1

[X,P]=linear_kalman_filter_function(X,P,Sdata(i),dt_mpu(i),Q,R,input(i));
[X1,P]=linear_kalman_filter_function(X1,P,Sdata2(i),dt_mpu(i),Q,R,input2(i));

Xsaved(:,:,i+1) = X;
Psaved(:,:,i+1) = P;
X1saved(:,:,i+1) = X1;
P1saved(:,:,i+1) = P1;

end

KF_AngleX  = Xsaved(1,1,:);
KF_AngleX = reshape(KF_AngleX,1,[]);
KF_AngleY  = X1saved(1,1,:);
KF_AngleY = reshape(KF_AngleY,1,[]);


%% EKF
EulerSaved = zeros(length(dt_mpu),3);

for i = 1:length(dt_mpu)
    [phi(i) theta(i) psi(i)] = EulerEKF([Acc_angleX(i) Acc_angleY(i)]',[p(i) q(i) r(i)]', dt_mpu(i),time_mpu(i),i);
EulerSaved(i,:) = [phi(i) theta(i) psi(i)];
end

PhiSaved = EulerSaved(:,1);
ThetaSaved = EulerSaved(:,2);
PsiSaved = EulerSaved(:,3);

%% match start time
for i = 1:20
%     time_xsens(1) = [];
%     Xsens_angleX(1) = [];
time_mpu(1) = [];
dt_mpu(1) = [];

KF_AngleX(1) = [];
comp_angleX(1) = [];
PhiSaved(1) = [];
Acc_angleX(1) = [];

KF_AngleY(1) = [];
comp_angleY(1) = [];
ThetaSaved(1) = [];
Acc_angleY(1) = [];
% Acc_angleX(1) = [];
% Acc_angleY(1) = [];
% p(1) = [];
% q(1) = [];
% r(1) = [];
end

%% PLOT
time_xsens = time_xsens - time_xsens(1);
time_mpu = time_mpu - time_mpu(1);
dt_mpu = dt_mpu - dt_mpu(1);

figure(1)
plot(time_xsens,Xsens_angleX,'r'); hold on;
plot(time_mpu,KF_AngleX,'b'); hold on;
plot(time_mpu,comp_angleX,'g'); grid on; hold on;
plot(time_mpu,PhiSaved','k')
xlabel('time (sec)');
ylabel('Angle (Deg)');
title('X축 회전 필터 각도 비교');

plot(time_mpu,PhiSaved,'c')
plot(time_mpu,PhiSaved,'g')
legend('XSens','Kalman Filter Angle','complementary','EKF','location','best');
% 
% figure(2)
% plot(time_xsens,Xsens_angleY,'r'); grid on;  hold on;
% plot(time_mpu,-KF_AngleY,'b'); hold on;
% plot(time_mpu,-comp_angleY,'g'); grid on; hold on;
% plot(time_mpu,-ThetaSaved','k')
% xlabel('time (sec)');
% ylabel('Angle (Deg)');
% title('Y축 회전 필터 각도 비교');
% legend('XSens 참값','Kalman Filter Angle','상보필터','EKF','location','best');
