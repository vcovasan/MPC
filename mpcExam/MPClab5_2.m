%% 5.2
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
Umax=1.75;
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

%K=[-2 -6]; %Mode2 stabilising feedback matrix
K = -place(A,B,[0 1*10^(-5)])
Acl = (A+B*K);
%P = [22 29; 29 53];
P = dlyap(Acl',Q+K'*R*K);


%set number of simulation steps
nk = 50; 

%initialize
x0 = [3;0]; %initial state
x= x0;

[H,L,M] = cost_mats(F,G,Q,R,P);

xs = zeros(n,nk+1);
us = zeros(m,nk+1);
VN = zeros(1,nk+1);

%loop
for k=1:nk+1
    %solve the constrained QP
    [Uopt,fval,flag] = quadprog(H,L*x,Pc,qc+Sc*x);
    %check feasability
    if flag<1
        disp(['Optimization infeasible at k = ' num2str(k)])
        break
    end
    %store
    xs(:,k) = x; %retrieve state
    us(:,k) = Uopt(1); %calculate input
    VN(:,k) = xs(:,k)'*Q*xs(:,k)+us(:,k)'*R*us(:,k);
    %move
    x= A* xs(:,k) + B* us(:,k);
end
%plot 
i = 0:1:nk;
figure(); 
subplot(2,1,1);
stairs(i,us,'DisplayName','u(k)'); 
title (['CL inputs for N= ',num2str(N)]);


subplot(2,1,2); hold all;
plot(i,xs(1,:),'DisplayName','x1(k)');
plot(i,xs(2,:),'DisplayName','x2(k)');
title(['CL states for N= ',num2str(N)]);

figure(3); % hold all;
plot(i,VN,'DisplayName','Cost function value');
title(['Cost function value for N= ',num2str(N)]);
