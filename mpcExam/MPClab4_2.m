%% 4.2: plotting the value function for P = 0 
clear,clc
close all
hold off;

%state space model matrices
A= [1 1
    0 2];
B= [0
    0.5];
C = [1 0];

% dimensions
n = size(A,1);
m = size(B,2);
N = 10; %prediction horizon
[F,G] = predict_mats(A,B,N);
Q =C'*C;    R=1;    %P=Q;
P=zeros(2,2);
[H,L,M] = cost_mats(F,G,Q,R,P);


%Simulate closed loop response
%get MPC control law
K = -inv(H)*L;
K = K(1:m,:);

%set number of simulation steps
nk = 10; %same as the horizon but dosn't have to be

%initialize
x0 = [3;0]; %initial state
x= x0;
Acl = (A+B*K);
eig(Acl);
xs = zeros(n,nk+1);
us = zeros(m,nk+1);
VN = zeros(1,nk+1);
%loop
for k=1:nk+1
    %store
    xs(:,k) = x; %calculate state
    us(:,k) = K*x; %calculate input
    Cost = 0.5*us(:,k)'* H* us(:,k) + x'*L'*us(:,k) + x'*M*x; % calculate cost function
    VN(:,k) = Cost(1,1); % only take the cost of the first sequence 
    %move
    x= Acl*x;
end
%plot 
i = 0:1:nk;
figure(1); hold all;
stairs(i,us,'DisplayName','u(k)'); 
title (['CL inputs for N= ',num2str(N)]);

figure(2); hold all;
plot(i,xs(1,:),'DisplayName','x1(k)');
plot(i,xs(2,:),'DisplayName','x2(k)');
title(['CL states for N= ',num2str(N)]);

figure(3);
plot(i,VN,'DisplayName','Cost function value');
title(['Cost function value for N= ',num2str(N)]);

%% the cost keeps increasing, which further suggests that the controller is unstable