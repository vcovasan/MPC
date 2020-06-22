clear,clc

A= [1 1.5
    0 0.5];
B= [0.5
    1];
C = [1 0];
N = 2; % smaller horizon

[F,G] = predict_mats(A,B,N);

syms u1 u2;

% implement the two linear equations making xf = Ax0+Bu
% where u is the unknown X 

% xf1 = a11*x11 + a12*x21 + b11*u1
eq1 = 2*u1 + u2/2 + 3/4 ==-1;

% xf2 = a11*x11 + a22*x21 + b12*u1
eq2 = u1/2 + u2 - 1/4 == 4;
[X,Y] = equationsToMatrix([eq1, eq2], [u1,u2]);
X = linsolve(X,Y);

x0 = [3;-1]; %initial state
u0 = X; %control sequence
n=size(A,2);
x1 = []; %predicted state
x2 = []; %predicted state

x = F*x0 + G*u0;
x= [x0;x];
for i=1:size(x)
    if mod(i,2)==0
    x1 = [x1; x(i)]; %prediction of first state
    else
    x2 = [x2; x(i)]; %prediction of second state
    end
end

%plotting time series
i = 0:1:size(x1)-1; %time vector

figure(1)
plot(i,x1,i,x2);title('Time series');grid on;
legend('x1','x2');

% figure(2)

% plot(x1,x2);title('Phase plot');grid on;
% xlabel('x1');
% ylabel('x2');