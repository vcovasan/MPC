%% 5.2

N = 10; %prediction horizon

A = [3 10;
     0 3]; 
B = [ 2 1 ;0.1  2];
C = [1 0; 0 1];
D = zeros(2,2);

sys = ss(A,B,C,D,0.1);

statePerturb = 0;
inputPerturb = 0;


% dimensions
n = size(A,1);
m = size(B,2);

[F,G] = predict_mats(A,B,N);
Q =[100 0; 0 1000];    R=[1 0; 0 10];


% dimensions
n = size(A,1);
m = size(B,2);


%K= -plac(A,B,Q,R);

K = -place(A,B,[0 0]);


%P = [22 29; 29 53];
%P = zeros(2,2)
P = dlyap((A+B*K)',Q+K'*R*K);

x0 = [12;7]; %initial state

%Constraints Definition
Umax=[45; 5];
Umin=-Umax; % YES, MINUS -UMAX 
Xmax=[15;10];
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


% Pxf = [eye(n);-eye(n)];
% qxf = repmat([0;0],n,1);

% 
M = kron(eye(n),[Px; Pu*K]);
Pxf = M*[(A+B*K)^(0);(A+B*K)^(1)]; % do it with KRON 
qxf = [qx ; qu; qx ; qu];


[Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf); %with terminal constraint
%[Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,[],[]); %%without terminal constraints


% %input constraint matrices
% %Pu*u(k+j|k) <= qu
% Pu = [eye(m);-eye(m)];
% qu = [Umax;-Umin]; % THIS IS VERY IMPORTANT
% %state constraint matrices
% %Px*x(k+j|k) <= qx
% Px = [eye(2);-eye(2)];
% qx = [Xmax;-Xmin]; %% THIS IS VERY MPORTANT 
% %termianl state constraint matrices
% %Pxf*x(k+N|k) <= qxf
% Pxf = [eye(n);-eye(n)];
% qxf = repmat([0;0],n,1);
% % [Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf); %with terminal constraint
% [Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,[],[]); %%without terminal constraints

%K=[-2 -6]; %Mode2 stabilising feedback matrix



%set number of simulation steps
nk = 40; 

%initialize

xs= x0;

[H,L,M] = cost_mats(F,G,Q,R,P);

xs = zeros(n,nk+1);
us = zeros(m,nk+1);
VN = zeros(1,nk+1);
u = zeros(size(B,2),length(0:nk+1));
U = zeros(size(B,2)*N,length(0:nk+1));
X = zeros(size(A,1)*N,length(0:nk+1));

xs(:,1) = x0;
i = 1;
%loop

for k=1:nk+1
    %solve the constrained QP
    %[Uopt,fval,flag] = quadprog(H,L*xs(:,i));
    [Uopt,fval,flag] = quadprog(H,L*xs(:,i),Pc,qc+Sc*xs(:,i));
    %check feasability
    if flag<1
        disp(['Optimization infeasible at k = ' num2str(k)])
        break
    end
    %store
  U(:,i) = Uopt;
  %cost_fn(i) = fval; 
  
  X(:,i) = F*xs(:,i) + G*U(:,i);
  
  us(:,i) = U(1:2,i);
  
  xs(:,i+1) = A*(xs(:,i)+statePerturb*xs(:,i)) + B*(us(:,i)+inputPerturb*(us(:,i)));
    VN(:,k) = xs(:,k)'*Q*xs(:,k)+us(:,k)'*R*us(:,k);
    %move
    %x= A* xs(:,k) + B* us(:,k);
    
      if k ~= nk+1
    i = i+1;
      end
    
end
%plot 
% 
% i1 = 0:1:nk+1;
% i = 0:1:nk;
% figure(1); 
% stairs(us(1,:),'linewidth',2);hold on;
% stairs(us(2,:),'linewidth',2);
% title ('Closed-Loop Inputs vs Time');
% legend('\delta_{e}','\theta')
% xlabel('Samples(s^{-1})');
% ylabel('Pitch Angle(Degrees) and Pitch Rate(ms^{-1})')
% 
% figure(2)
% plot(xs(1,:),'linewidth',2); hold on;
% plot(xs(2,:),'linewidth',2);
% title('Closed-Loop States vs Time');
% legend('\alpha','q')
% xlabel('Samples(s^{-1})');
% ylabel('Elevator Angle(Degrees) / Thrust Vectoring Angle(Degrees)')
% 
% figure(3); % hold all;
% plot(VN,'DisplayName','Cost function value','linewidth',2);
% title('Minimal Cost vs Time');
% xlabel('Samples(s^{-1})');
% ylabel('Value function V_{N}^{o}')
% 
% figure(4)
% plot(xs(1,:),xs(2,:),'linewidth',2);
% title('Constraintd Phase Plot: Angle of Attack vs Thrust Vectoring Angle');
% xlabel('Angle of Attack(Degrees)');
% ylabel('Pitch Rate(m/s)')
% legend('Pitch Rate')