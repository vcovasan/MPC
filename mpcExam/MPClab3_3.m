

%state space model matrices
A= [1 1
    0 2];
B= [0
    0.5];
C = [1 0];

% dimensions
n = size(A,1);
m = size(B,2);
N = 1; %prediction horizon
[F,G] = predict_mats(A,B,N);
Q =C'*C;    R=1;    %P=Q;
P=zeros(2,2);
[H,L,M] = cost_mats(F,G,Q,R,P);
x0 = [3;0]; %initial state
f = x0' * L';
[U_opt,cost]=quadprog(H,f);

% Time invariant state feedback control law
Kn = -inv(H)*L;
%Kn = Kn(1:m,:);
u0 = Kn*x0;

%% 3.3
%get MPC control law
K = -inv(H)*L;
K = K(1:m,:); % first step, use the first m rows as k for a+bk

%K = -dlqr(A,B,Q,R);

i = 0:1:N;

%set number of simulation steps
nk = N; %same as the horizon but dosn't have to be

%initialize
x= x0;
Acl = (A+B*K); % this is the essence of this exerciese 
xs = zeros(n,nk+1);
us = zeros(m,nk+1);

%loop


for k=1:nk+1
    %store
    xs(:,k) = x;
    us(:,k) = K*x;
    %move
    x= Acl*x;
end

%plot on top of open loop graphs
figure(1); hold all;
stairs(i,us,'DisplayName','u(k)'); 

figure(2); hold all;
plot(i,xs(1,:),'DisplayName','x1(k)');
plot(i,xs(2,:),'DisplayName','x2(k)');

%% 3.5 

%infinite horizon LQR controller K_inf

%as N approaches infinity, Kn approaches K_inf
%hence LQR is the same to an infinite horizon unconstrained MPC

%% 3.6
%One should tune the system using R, to switch the penalty between input
%magnitude and state error


