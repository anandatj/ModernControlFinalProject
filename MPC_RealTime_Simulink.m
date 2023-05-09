%% This code is used in the DCMotorMPC.slx file . Control signal is updated online and is working!

function u_mpc = fcn(yt,ref)

% System discrete model at Ts = 1e-4
Ad = [0.999078612745286	-0.000105301249844195;...
2.45332729742468e-05	0.999999942082396];
Bd = [8.22666037703133e-05;1.00975483059156e-09];
Cd = [0 1];
Dd = [0];

% Input and Output Calculation
n=size(Ad,1);
p=size(Bd,2);

% Controller and Gradient Descent Parameter
R = 0.00002*eye(10);
Q = 50*eye(10);
Np = 10; % Prediction Horizon
alpha=10; % Step Size

% Reference Signal
rp = ref*ones(Np,1);

% Matrix Augmentation for Observer
Aa=[Ad zeros(n,p);Cd*Ad eye(p)];
Ba=[Bd;Cd*Bd];
Ca=[zeros(n,p)' eye(p)];


% Observer Matrix for Np = 10
W=[Ca*Aa;Ca*(Aa^2);Ca*(Aa^3);Ca*(Aa^4);Ca*(Aa^5);Ca*(Aa^6);Ca*(Aa^7);...
    Ca*(Aa^8);Ca*(Aa^9);Ca*(Aa^10);];

Z=[Ca*Ba 0 0 0 0 0 0 0 0 0;...
    Ca*Aa*Ba Ca*Ba 0 0 0 0 0 0 0 0;...
    Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0 0 0 0 0 0;...
    Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0 0 0 0 0;...
    Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0 0 0 0;...
    Ca*(Aa^5)*Ba Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0 0 0;...
    Ca*(Aa^6)*Ba Ca*(Aa^5)*Ba Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0 0;...
    Ca*(Aa^7)*Ba Ca*(Aa^6)*Ba Ca*(Aa^5)*Ba Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0 0;...
    Ca*(Aa^8)*Ba Ca*(Aa^7)*Ba Ca*(Aa^6)*Ba Ca*(Aa^5)*Ba Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba 0;...
    Ca*(Aa^9)*Ba Ca*(Aa^8)*Ba Ca*(Aa^7)*Ba Ca*(Aa^6)*Ba Ca*(Aa^5)*Ba Ca*(Aa^4)*Ba Ca*(Aa^3)*Ba Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba];

% Initial States and input [del_x_1;del_x_2;z_k] and del_u
xa=zeros(n+p,p);
xa(3) = yt;
Del_U=zeros(Np,1);

% Gradient Descent Method
Max_Iter=1e4;  
    %Gradient Descent
    for i=1:Max_Iter    
         grad_J=-(rp-W*xa-Z*Del_U)'*Q*Z + Del_U'*R;
         Del_U = Del_U - (alpha)*(grad_J');
    end

%Output Computation with u_ref=del_u(1)
u_mpc = Del_U(1);
u_mpc = u_mpc/400;
