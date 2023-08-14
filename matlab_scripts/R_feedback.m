v_pwm_0 =  5.6; % voltaje cuando el pwm es 0%
v_pwm_100 = 4.4; % voltaje cuando pwm es 100% a la salida

vpwm_100 = 5; % voltaje cuando pwm es 100%

R1 = 750; % R feedback entre fv y gnd

syms  R3 R2; %R3 pwm fedback, R2 voltaje feedback

eqn1 = v_pwm_0*R1*R3/(R1*R3+R2*(R1+R3)) == 1.235;
eqn2 = v_pwm_100*R1*R3/(R1*R3+R2*(R1+R3)) + ...
    vpwm_100*R1*R2/(R1*R2+R3*(R1+R2)) == 1.235;

[R2,R3] = solve(eqn1,eqn2,0<R2);

R2 = double(R2);
R3 = double(R3);

fprintf("R2 = %.2f\n",R2)
fprintf("R3 = %.2f\n",R3)

%% --------- parallel equivalent resistors ----------

R2_a = 3.3e3;

R3_a = 15e3;

syms R2_b R3_b

R2_b = double( solve(R2_a*R2_b/(R2_a+R2_b) == R2 ));
R3_b = double( solve(R3_a*R3_b/(R3_a+R3_b) == R3 ));

fprintf("R2_b = %.2f\n",R2_b)
fprintf("R3_b = %.2f\n",R3_b)

%% ----------- 
R2_b_select = 10e3;
R3_b_select = 30e3;

R1_eq = R1;
R2_eq = R2_a*R2_b_select/(R2_a+R2_b_select);
R3_eq = R3_a*R3_b_select/(R3_a+R3_b_select);

vpwm_test = 5   ;

syms vout

eqn = vout*R1_eq*R3_eq/(R1_eq*R3_eq+R2_eq*(R1_eq+R3_eq)) + ...
    vpwm_test*R1_eq*R2_eq/(R1_eq*R2_eq+R3_eq*(R1_eq+R2_eq)) == 1.235;
Vout = solve(eqn,0<vout);

fprintf("Vout = %.2f\n ",double(Vout))
