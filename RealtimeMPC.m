

%System Parameters
ref = 100;
ut = 10;
yt = 100;
% Controller Parameters
R = 0.1*eye(10);
Q = 1*eye(10);
Np = 10; %Pred Horizon
alpha=0.005; %Step Size

Ad = [0.911927848596776	-0.0100639265996126;...
0.00234471156635270	0.999981797020531];
Bd = [0.00786246525855858;9.79727138107321e-06];
Cd = [0 1];
Dd = 0;

n=size(Ad,1);%number of states
p=size(Bd,2);%number of inputs

rp = ref*ones(Np,1);

Aa=[Ad zeros(n,p);Cd*Ad eye(1,p)];
Ba=[Bd;Cd*Bd];
Ca=[zeros(n,p)' eye(p)];


%Define the matrices W and Z
W=[Ca*Aa;Ca*(Aa^2);Ca*(Aa^3);Ca*(Aa^4);Ca*(Aa^5);Ca*(Aa^6);Ca*(Aa^7);...
    Ca*(Aa^8);Ca*(Aa^9);Ca*(Aa^10)];
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

%Initial conditions: 

u=zeros(2,1);
u(1) = ut;
xa=ones(n+p,p);
xa(3) = yt;

Del_U=zeros(Np,1);

L=5e3;%number of iterations of the gradient descent   
    %Gradient Descent
    for i=1:L 
         grad_J=-(rp-W*xa-Z*Del_U)'*Q*Z + Del_U'*R;
         Del_U = Del_U - (alpha)*(grad_J');
    end

