N = 3; % unstable for 3 or greater 

% this is a ONE input, two output system

A = [5 10; 0 3];
B = [ 2 1 ;0.1  2];
C = [1 0; 0 1];
D = zeros(2,2);

sys=ss(A,B,C,D);

rank(ctrb(A,B)); % full rank, therefore reachable

[F,G]=predict_mats(A,B,N);

n = size(A,1);
m = size(B,2);

Q = eye(2); % 
R = [1 0; 0 1]; %

K = -dlqr(A,B,Q,R);

P = zeros(2,2);
%P = dlyap((A+B*K)',Q+K'*R*K);

[H,L,M] = cost_mats(F,G,Q,R,P);
Tfinal = 100;

percent = 0; 
x0 = [100 -100];

%K = -place(A,B,[1 0.9])




%P = dlyap(Acl',Q+K'*R*K);

%H; % huge numbers in the hessian even though the values in A are only 1s and 2s 

%set number of simulation steps
%initialize

x = zeros(size(A,1),length(0:Tfinal)+1);
U = zeros(size(B,2)*N,length(0:Tfinal));
X = zeros(size(A,1)*N,length(0:Tfinal));
u = zeros(size(B,2),length(0:Tfinal));
cost_fn = zeros(1,length(0:Tfinal));
VN = zeros(1,length(0:Tfinal));


%x(:,1) = x0;
i = 1;

%loop
for k=1:Tfinal+1
  [Uopt,fval,flag] = quadprog(H,L*x(:,i));
  
  if flag < 1
      disp(['OPtimization infeasible at k = ',num2str(k)])
      break
  end
  
  U(:,i) = Uopt;
  cost_fn(i) = fval; 
  
  X(:,i) = F*x(:,i) + G*U(:,i); % state predictions
  
  u(:,i) = U(1:2,i);
  
  x(:,i+1) = A*x(:,i) + B*u(:,i); % actual states 
  %VN(:,k) = x(:,k)'*Q*xs(:,k)+u(:,k)'*R*u(:,k); % actual cost 
    %move


  
  if k ~= Tfinal
    i = i+1;
  end

end


i1 = 0:1:Tfinal+2;
i2 = 0:1:Tfinal;
x1 = x(1,:);
x2 = x(2,:);


x1= [x0(1) x1];
x2= [x0(2) x2];

u1 = u(1,:);
u2 = u(2,:);

figure(1)
plot(i1,x1,i1,x2);title('Time series');grid on;
legend('x1','x2');

figure(2)
stairs(i2,u1);title('Time series');grid on;
hold on;
stairs(i2,u2);title('Time series');grid on;
legend('u1','u2');
