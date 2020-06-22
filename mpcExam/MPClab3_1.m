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
Kn = Kn(1:m,:);
u0 = Kn*x0;

U_opt(1) == u0; % yes, up to the 8th decimal, there they start to differ, which might be because of computational errors in inverting large matrices or smth, FININTE PRECISION OF THE COMPUTER


