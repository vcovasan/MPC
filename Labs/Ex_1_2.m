clear,clc

A= [1 1.5
    0 0.5];
B= [0.5
    1];
C = [1 0];
N = 6;
[F,G] = predict_mats(A,B,N);

x0 = [1;0]; %initial state
u0 = [-0.25; 0; 0; 0; 0; 0]; %control sequence
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

figure(2)
plot(x1,x2);title('Phase plot');grid on;
xlabel('x1');
ylabel('x2');
