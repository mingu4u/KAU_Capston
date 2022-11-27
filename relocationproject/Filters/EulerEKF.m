function [phi theta psi] = EulerEKF(z, rates, dt,time_mpu,i)

persistent H Q R
persistent x P
persistent count

if (isempty(count))
    H = [1 0 0;
         0 1 0];
     Q = [0.0055 0 0;
         0 0.0055 0;
         0 0 0.55];
%      Q = [0.0001 0 0;
%          0 0.0001 0;
%          0 0 0.1];
     R = [6 0;
         0 6];
     
     x = [0 0 0]';
     P = 10*eye(3);
     

end
     count =  1;
A = Ajacob(x,rates,dt);

xp = fx(x,rates,dt,i);
Pp = A*P*A' + Q;

K = Pp*H'*inv(H*Pp*H'+R);
x = xp + K*(z-H*xp);
P = Pp - K*H*Pp;

phi = x(1);
theta = x(2);
psi = x(3);

%-----------------------------------------------
function xp = fx(xhat,rates,dt,i)
%
%
phi = xhat(1);
theta = xhat(2);

p = rates(1);
q = rates(2);
r = rates(3);

xdot = zeros(3,1);
    matrix = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta);
              0 cosd(theta) -sind(phi);
              0 sind(phi)*secd(theta) cosd(phi)*secd(theta)];
          
xdot = matrix * [p q r]';
xp = xhat + xdot*dt;

%------------------------------------------------------
function A = Ajacob(xhat,rates,dt)
%
%
A = zeros(3,3);

phi = xhat(1);
theta = xhat(2);

p = rates(1);
q = rates(2);
r = rates(3);
A(1,1) = q*cosd(phi)*tand(theta) - r*sind(phi)*tand(theta);
A(1,2) = q*sind(phi)*secd(theta)^2 + r*cosd(phi)*secd(theta)^2;
A(1,3) = 0;

A(2,1) = -q*sind(phi)-r*cosd(phi);
A(2,2) = 0;
A(2,3) = 0;

A(3,1) = q*cosd(phi)*secd(theta) - r*sind(phi)*secd(theta);
A(3,2) = q*sind(phi)*secd(theta)*tand(theta)+r*cosd(phi)*secd(theta)*tand(theta);
A(3,3) = 0;

A = eye(3) + A.*dt;
     