v_pwm_0 =  4.2; % voltaje cuando el pwm es 0%
v_pwm_100 = 1.8; % voltaje cuando pwm es 100%

vpwm_100 = 5; % voltaje cuando pwm es 100%

R1 = 750; % R feedback entre fv y gnd

syms  R3 R2; %R3 pwm fedback, R2 voltaje feedback

eqn1 = v_pwm_0*R1*R3/(R1*R3+R2*(R1+R3)) == 1.235;
eqn2 = v_pwm_100*R1*R3/(R1*R3+R2*(R1+R3)) + ...
    vpwm_100*R1*R2/(R1*R2+R3*(R1+R2)) == 1.235;

[R2,R3] = solve(eqn1,eqn2,0<R2);

R2 = double(R2);
R3 = double(R3);

fprintf("R2 = %f.2\n",R2)
fprintf("R3 = %f.2\n",R3)

%% --------- parallel equivalent resistors ----------

R2_a = 3e3;

R3_a = 4.7e3;

syms R2_b R3_b

R2_b = double( solve(R2_a*R2_b/(R2_a+R2_b) == R2 ));
R3_b = double( solve(R3_a*R3_b/(R3_a+R3_b) == R3 ));

fprintf("R2_b = %f.2\n",R2_b)
fprintf("R3_b = %f.2\n",R3_b)





