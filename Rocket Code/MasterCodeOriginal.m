clc
clear

T = 0.5; % seconds
N = 40; % Prediction Horizon
Tfinal = 125; % the final time step; time steps: k = 0, 1, 2, ..., Tfinal

A = [eye(3) T*eye(3);
     zeros(3) eye(3)];
 
B = [(1/2)*T^2*eye(3);
     T*eye(3)]; 

Q = [eye(3) zeros(3);
     zeros(3) eye(3)*10];
 
R = eye(3)*0.1;

Kinf = -dlqr(A,B,Q,R);
%Kinf = -place(A,B,[0 0 0 10^-5 10^-5 10^-5])
P= dlyap((A+B*Kinf)',Q+Kinf'*R*Kinf);


[F,G] = predict_mats(A,B,N);
[H,L,M] = cost_mats(F,G,Q,R,P);

k1 = tand(10);
k2 = (1/tand(30));
g = 9.80665; % m/s^2

Pu = [0 0 1;
      0 0 -1;
      1 0 -tand(10);
      -1 0 -tand(10);
      0 1 -tand(10);
      0 -1 -tand(10)];
  
qu = [12-g g g*tand(10) g*tand(10) g*tand(10) g*tand(10)]';

Px = [1 0 -(1/tand(30)) 0 0 0;
      -1 0 -(1/tand(30)) 0 0 0;
      0 1 -(1/tand(30)) 0 0 0;
      0 -1 -(1/tand(30)) 0 0 0;
      0 0 0 0 0 -1;
      0 0 0 1 0 0;
      0 0 0 -1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 -1 0];
  
qx = [0 0 0 0 15 20 20 20 20]';

Pxf = Px;
qxf = qx;

[Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf);

r0 = [-600 600 500]'; % initial r in m
v0 = [5 -5 5]'; % initial v in m/s
x0 = [r0;
      v0];

x = zeros(size(A,1),length(0:Tfinal)+1);
U = zeros(size(B,2)*N,length(0:Tfinal));
X = zeros(size(A,1)*N,length(0:Tfinal));
u = zeros(size(B,2),length(0:Tfinal));
cost_fn = zeros(1,length(0:Tfinal));

x(:,1) = x0;
i = 1;
for k = 0:Tfinal
  
  [Uopt,fval,flag] = quadprog(H,L*x(:,i),Pc,qc + Sc*x(:,i));
  
  if flag < 1
      disp(['OPtimization infeasible at k = ',num2str(k)])
      break
  end
  
  U(:,i) = Uopt;
  cost_fn(i) = fval; 
  
  X(:,i) = F*x(:,i) + G*U(:,i);
  
  u(:,i) = U(1:3,i);
  
  x(:,i+1) = A*(x(:,i)) + B*u(:,i);
  
  if k ~= Tfinal
    i = i+1;
  end
end


rx = x(1,:);
ry = x(2,:);
rz = x(3,:);

vx = x(4,:);
vy = x(5,:);
vz = x(6,:);
% 
% 
% trajectory = plot3(rx,ry,rz, 'r') 
% grid on
% grid minor
% xlim([-50 50])
% ylim([-50 50])
% zlim([0 200])
% xlabel("Latititude(m)")
% ylabel("Longitude(m)")
% zlabel("Altitude(m)")
% title("Rocket Trajectory in 3D Space")
% % 
% figure()
% plot(rx,rz)
% axis equal
% grid on 
% grid minor
% xlabel('r_x in meters')
% ylabel('r_z in meters')
% 
% 
% figure()
% plot(vx,vz)
% axis equal
% grid on
% grid minor
% 
% figure()
% plot(0:k+1,vz(1:k+2))
% grid on 
% grid minor
% ylabel('v_z in m/s')
% xlabel('discrete time steps (k)')
% 
% figure()
% plot(0:k+1,vx(1:k+2))
% grid on 
% grid minor
% ylabel('v_x in m/s')
% xlabel('discrete time steps (k)')
% 
% figure()
% plot(0:k,u(1,1:k+1))
% grid on 
% grid minor
% ylabel('f_x/m in N/kg')
% xlabel('discrete time steps (k)')
% 
% figure()
% plot(0:k,u(2,1:k+1))
% xlabel("Time")
% ylabel("Control Magnittude")
% 
% figure()
% plot(0:k,u(3,1:k+1)+g)
% grid on 
% grid minor
% ylabel('f_z/m in N/kg')
% xlabel('discrete time steps (k)')
% 
% figure()
% plot(0:k+1,atan2d(rz(1:k+2),abs(rx(1:k+2))))
% grid on 
% grid minor
% xlabel('discrete time steps (k)')
% ylabel('tan^{-1}(r_z/|r_x|) in degrees')