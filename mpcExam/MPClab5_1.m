%% 5.1
%state space model matrices
A= [1 1
    0 2];
B= [0
    0.5];
C = [1 0];

% dimensions
n = size(A,1);
m = size(B,2);
N = 3; %prediction horizon
[F,G] = predict_mats(A,B,N);
Q =C'*C;    R=1;    

%Constraints Definition
Umax=1;
Umin=-Umax; % YES, MINUS -UMAX 
Xmax=[10;5];
Xmin=-Xmax;

%input constraint matrices
%Pu*u(k+j|k) <= qu
Pu = [eye(m);-eye(m)];
qu = [Umax;-Umin]; % THIS IS VERY IMPORTANT
%state constraint matrices
%Px*x(k+j|k) <= qx
Px = [eye(2);-eye(2)];
qx = [Xmax;-Xmin]; %% THIS IS VERY MPORTANT 
%termianl state constraint matrices
%Pxf*x(k+N|k) <= qxf
Pxf = [eye(n);-eye(n)];
qxf = repmat([0;0],n,1);
% [Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf); %with terminal constraint
[Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,[],[]); %%without terminal constraints