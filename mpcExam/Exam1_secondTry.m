%% 5.2

N = 10; %prediction horizon

% x1 is angle of attach
% x2 is pitch rate 

% 

A = [3 10;
     0 3]; 
B = [ 2 1 ;0.1  2];
C = [1 0; 0 1];
D = zeros(2,2);

sys = ss(A,B,C,D,0.1);

statePerturb = 0.175;
inputPerturb = -0.1;


% dimensions
n = size(A,1);
m = size(B,2);

[F,G] = predict_mats(A,B,N);
Q =[10 0; 0 1];    R=[1 0; 0 100];

K= -dlqr(A,B,Q,R);
%K = -place(A,B,[0 1*10^-5])
Acl = (A+B*K);
%P = [22 29; 29 53];
%P = zeros(2,2)
P = dlyap((A+B*K)',Q+K'*R*K);

x0 = [100; 100]; %initial state

nk = 150; 

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
    [Uopt,fval,flag] = quadprog(H,L*xs(:,i));
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
    
      if k ~= Tfinal
    i = i+1;
      end
    
end
%plot 
i1 = 0:1:nk;
i = 0:1:nk;
figure(1); 
stairs(i,us(1,:),'linewidth',2);hold on;
stairs(i,us(2,:),'linewidth',2);
title ('Closed-Loop Inputs vs Time');
legend('\delta_{e}','\theta')
xlabel('Samples(s^{-1})');
ylabel('Pitch Angle(Degrees) and Pitch Rate(ms^{-1})')

figure(2)
plot(i1,xs(1,:),'linewidth',2); hold on;
plot(i1,xs(2,:),'linewidth',2);
title('Closed-Loop States vs Time');
legend('\alpha','q')
xlabel('Samples(s^{-1})');
ylabel('Elevator Angle(Degrees) / Thrust Vectoring Angle(Degrees)')

figure(3); % hold all;
plot(i,VN,'DisplayName','Cost function value','linewidth',2);
title('Minimal Cost vs Time');
xlabel('Samples(s^{-1})');
ylabel('Value function V_{N}^{o}')

figure(4)
plot(xs(1,:),xs(2,:),'linewidth',2);
title('Phase Plot');
xlabel('Samples(s^{-1})');
ylabel('Elevator Angle(Degrees) / Thrust Vectoring Angle(Degrees)')