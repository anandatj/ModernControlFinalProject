%System Parameters
J = 0.5215;
Kb = 1.28;
Kt = 1.28;
B_m = 0.002953;
Ra = 11.2;
La = 0.1215;


A = [-Ra/La -Kb/La;Kt/J -B_m/J];
B = [1/La;0];
C = [0 1];
D = [0];


% Controller Parameters
R = 0.1*eye(3);
Q = eye(3);

h=0.00001; %sampling time
[Ad,Bd,Cd,Dd]=c2dm(A,B,C,D,h,'zoh');


