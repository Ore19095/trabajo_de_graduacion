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
plot(input_data);

% estimate transfer function
opt = tfestOptions('Display','on');

no_poles = 3;
no_zeros = 0;

tf_sys = tfest(input_data,no_poles,no_zeros,opt)
step(tf_sys);

%% Estimado inicial del PID

crossover = 80;
[C,info] = pidtune(tf_sys,'pid',crossover)


step(feedback(C*tf_sys,1))
