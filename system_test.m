J = 0.5215;
Kb = 1.28;
Kt = 1.28;
B = 0.002953;
Ra = 11.2;
La = 0.1215;

A = [-Ra/La -Kb/La;Kt/J -B/J];
B = [1/La;0];
C = [0 1];
D = 0;

Cab = [B A*B];
rank_Cab=rank(Cab)

Oac = [C;C*A];
rank_Oac = rank(Oac)

% Internal Stability Test
time = 0:0.01:10;
[t_max] = size(time);
x = [0.1;0.1];
z_out = zeros([2,t_max(2)]);

% Internal Stability Equation
for t = 1:t_max(2)
z = expm(A*(time(t)))*x;
z_out(1,t) = z(1);
z_out(2,t) = z(2);
end

% Plot
figure(1)
plot(time,z_out);
legend("I","w_r","Location","northeast");
title('PMDC Internal Stability');