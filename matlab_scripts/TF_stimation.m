t_fin = 20e-3;
N = 20e3;
T = 1/N

Vin_inter = zeros(N+1,1);
Vout_inter = zeros(N+1,1);
for i = 0:N

    Vin_inter(i+1) = interp1(time,Vin,i*t_fin/N);
    Vout_inter(i+1) = interp1(time,Vout,i*t_fin/N);
end

%% test controller 

controller = tf(C)

Sys = feedback(tf4*controller,1)

step(Sys)
