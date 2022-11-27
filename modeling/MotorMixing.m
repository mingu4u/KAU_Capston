function [pwm] = MotorMixing(in)

thrust = in(1);
  roll = in(2);
 
  pwm1 = thrust - roll; %  Left Motor
  pwm2 = thrust + roll; % Right Motor
    
  pwm  = [pwm1, pwm2];