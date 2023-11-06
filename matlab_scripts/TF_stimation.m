%% Interpolar los datos para tener un espaciado uniforme
load('sistema.mat')

t_fin = 20e-3;
N = 5e3;
T = t_fin/N;
time_inter = 0:t_fin/N:t_fin;

Vin_inter = zeros(N+1,1);
Vout_inter = zeros(N+1,1);
for i = 0:N
    Vin_inter(i+1) = interp1(time,Vin,i*T);
    Vout_inter(i+1) = interp1(time,Vout,i*T);
end
%% use system identification

Vin_no_avg = Vin_inter - mean(Vin_inter);
Vout_no_avg = Vout_inter - mean(Vout_inter);

input_data = iddata(Vout_no_avg,Vin_no_avg,T);
figure(3)
plot(input_data);

% estimate transfer function
opt = tfestOptions('Display','on');

no_poles = 3;
no_zeros = 0;

tf_sys = tfest(input_data,no_poles,no_zeros,opt)
figure(1)
step(tf_sys);

%% Estimado inicial del PID

crossover = 2000;
[C,info] = pidtune(tf_sys,'pid',crossover)

figure(2)
step(feedback(C*tf_sys,1))
%% ------- Discretizando -----------
Ts = 1/1000
C_tf = tf(C)
pid_discrete = c2d(C_tf,Ts,'Tustin');
pid_dsp = filt(pid_discrete.Numerator,pid_discrete.Denominator,Ts)
sys_discrete = c2d(tf_sys,Ts,'Tustin');

step(feedback(pid_discrete*sys_discrete,1));
