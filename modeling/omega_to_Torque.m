function [Control_Torque_Matrix] = omega_to_Torque(in)

persistent kt l

if isempty(kt)
    global motor;
    global inertia;

    kt = motor.kt;
    l  = inertia.l;
    clear motor inertia
end

w1 = (in(1))^2; %  Left Motor
w2 = (in(2))^2; % Right Motor

Matrix = [  kt,    kt;
          -kt*l, kt*l];

           
Control_Torque_Matrix = Matrix*[w1; w2];
