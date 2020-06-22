%state space model matrices
A= [1 1
    0 2];
B= [0
    0.5];
C = [1 0];

% dimensions
n = size(A,1);
m = size(B,2);
N = 15; %prediction horizon
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

U_opt(1) == u0; % yes, up to the 8th decimal, there they start to differ, which might be because of computational errors in inverting large matrices or smth, FININTE PRECISION OF THE COMPUTER


%% 3.2
%x = F*x0 + G*u0;
x = (F- G*inv(H)*L)*x0; % OPEN LOOP STATE PREDICTIONS 
x= [x0;x];
u0 =[u0;0];
x1 = x(1:2:end);
x2 = x(2:2:end);
figure(1)
i = 0:1:N;
stairs(i,u0,'DisplayName','u(0+1|k)');
title (['Predicted and CL inputs for N= ',num2str(N)]);
legend('-DynamicLegend');

figure(2)
plot(i,x1,'DisplayName','x1(0+1|k)');
legend('-DynamicLegend');
hold all;
plot(i,x2,'DisplayName','x2(0+1|k)');
title(['Predicted and CL states for N= ',num2str(N)]);