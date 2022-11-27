clc;
clear;
%% 상보필터 테스트
data = load('angle_filter_test.csv');
comp_angleX = data(:,1);
Acc_angleX = data(:,2);
Gyro_angleX = data(:,3);
Gyro_angvelX = data(:,4);
time = data(:,5);

figure(1)
plot(time,Acc_angleX,'r');
hold on;
plot(time,Gyro_angleX,'b');
grid on;
hold on;
plot(time,comp_angleX,'g');
yline(90,'--','90Deg');
yline(-90,'--','-90Deg');
legend('Accangle','GyroAngle','Complementary Filter Angle','location','best');

xlabel('time (sec)')
ylabel('Angle (Deg)')
title('상보필터 테스트 (-90Deg ~ 90Deg)')


%% 칼만필터 테스트
    Q = [0.001 0; 0 0.003];
    H = [1 0];
    R = 0.03;
    X = [0;sqrt(2.1)];
    P = 10*eye(2);

    Xsaved = X;
    Psaved = P;
    P = 5*eye(2);
    input = Gyro_angvelX;
    Sdata = Acc_angleX;

for i =1:length(time)-1
    dt(i) = time(i+1)-time(i);
    
end
dt(length(time)) = dt(length(time)-1);
for i = 1 : length(time)-1

[X,P]=linear_kalman_filter_function(X,P,Sdata(i),dt(i),Q,R,input(i));


Xsaved(:,:,i+1) = X;
Psaved(:,:,i+1) = P;

end

KF_AngleX  = Xsaved(1,1,:)
KF_AngleX = reshape(KF_AngleX,1,[]);
figure(2)

hold on
grid on
plot(time,Sdata,'r')
plot(time,KF_AngleX,'g')
yline(90,'--','90Deg');
yline(-90,'--','-90Deg');
legend('Accangle','Kalman Filter Angle','location','best');

xlabel('time (sec)')
ylabel('Angle (Deg)')
title('칼만필터 테스트 (-90Deg ~ 90Deg)')

figure(3)
plot(time,comp_angleX,'r');
hold on;
grid on;
plot(time,KF_AngleX,'b');
yline(90,'--','90Deg');
yline(-90,'--','-90Deg');
legend('Complementary Filter Angle','Kalman Filter Angle','location','best');

xlabel('time (sec)')
ylabel('Angle (Deg)')
title('상보,칼만필터 비교 (-90Deg ~ 90Deg)')
