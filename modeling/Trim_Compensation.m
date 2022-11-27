function compensated_trim_pwm = Trim_Compensation(phi)

persistent m kt a b trim_pwm

if isempty(m)
    global inertia
    global motor
    global trim

    m        = inertia.mass;
    kt       = motor.kt;
    a        = motor.a;
    b        = motor.b;
    trim_pwm = trim.pwm;

    clear inertia motor trim
end

compensated_trim_pwm = (sqrt(m*9.81/(2*kt*cosd(phi))) - b)/a - trim_pwm;
% compensated_trim_pwm = compensated_trim_pwm;
