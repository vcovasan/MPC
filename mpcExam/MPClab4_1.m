%% 4.1: SHOWING THAT THE SYSTEM IS UNSTABLE FOR N = 3 
clear,clc
close all
hold off;

%state space model matrices
A= [1 1
    0 2];
B= [0
    0.5];
C = [1 0];

N = 3;

% dimensions
n = size(A,1);
m = size(B,2);
N = 3; %prediction horizon
[F,G] = predict_mats(A,B,N);
Q =C'*C;    R=1;    %P=Q;
P=zeros(2,2);
[H,L,M] = cost_mats(F,G,Q,R,P);

K = -inv(H)*L;
K = K(1:m,:);
%u0 = K*x0;

Acl = (A+B*K); % this is the essence of this exerciese 

% eigenvalues are outside the unit circle so unstable for N = 3 