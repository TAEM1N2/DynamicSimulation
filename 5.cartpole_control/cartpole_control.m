%% ================================
% Practice 5
% Cart-Pole System Dynamic Simulation 
% 20191091 
% Taemin Kim 
%% =================================
clc;
clear all;
close all;


global p p1 l        %변수를 전역으로 설정함

%% 시뮬레이션 준비

% 0 ~ 10초 dt=0.001초로 설정 
dt = 0.001;         % 시간 증분
t = 0:dt:10;        % 시뮬레이션 시간 
n = length(t);      % iteration 수 

x = zeros(1,n);     % cart position 초기화  
d_X= zeros(1,n);    % cart velocity 초기화 
th = zeros(1,n);    % pole angle 초기화 
d_th = zeros(1,n);  % pole angular velocity 초기화 

m1 = 10;            % mass of cart [kg]
m2 = 1;             % mass of pole [kg]
l = 1;              % length of pole [m]
g = 9.81;           % gravity acceleration [m/s^2]



%Initial Condition 
x(1) = 0;
d_x(1) = 0;
th(1) = 10 * pi/180;
d_th(1) = 0;

% Finding controller 
r = 3;

A=[0 1 0 0
   0 0 -m2*g/m1 0
   0 0 0 1
   0 0 (m1+m2)*g/(m1*l) 0];

B = [0 ; 1/m1 ; 0 ; -1/(m1*l)];


P = [-4;-4;-4;-4];

ABCD = [0 1 0 0 0
        0 0 -m2*g/m1 0 1/m1
        0 0 0 1 0 
        0 0 (m1+m2)*g/(m1*l) 0 -1/(m1*l)
        1 0 0 0 0];
N = inv(ABCD)*[0;0;0;0;1];

N_x = N([1,2,3,4],:);
N_u = N(5,:);

k = acker(A,B,P);
N_bar = N_u + k*N_x;

%% Iteration
for i=1:n-1

% Step 4. Control
% k =[-24.4648  -50.9684 -482.3748 -150.9684];
u = -k*[x(i);d_x(i);th(i);d_th(i)] + N_bar*r;

% Step 1. 운동방정식으로부터 다음시간의 가속도 구하기
% D2 = inv(M)*(-C-G+T)
M = [m1+m2 m2*l*cos(th(i)); m2*l*cos(th(i)) m2*l*l];
C = [-m2*l*sin(th(i))*d_th(i)*d_th(i); 0];
G = [0; -m2*g*l*sin(th(i))];
T = [u;0];


D2 = inv(M)*(-C-G+T);

d2_x = D2(1);
d2_th = D2(2);

% Step 2. 가속도를 수치적분하여 속도 구하기 
d_x(i+1) = d_x(i) + d2_x * dt;
d_th(i+1) = d_th(i) + d2_th * dt;

% Step 3. 속도를 수치적분하여 위치 구하기
x(i+1) = x(i) + d_x(i+1)*dt;
th(i+1) = th(i) + d_th(i+1)*dt;
   
end

% 그래프 그리기 
figure(1);
subplot(211);
plot(t,x);
title('Position of the Cart');
xlabel('time(s)');
ylabel('position of cart (m)');
grid on;
hold on;

subplot(212);
plot(t,th*180/pi);
title('Angle of the Pole');
xlabel('time(s)');
ylabel('angle of pole (deg)');
grid on;
hold on;

%Animation 
figure(2);
axis([-10. 10. -1. 1.]);
Ax=[0,0];Ay=[0,0];
p=animatedline(Ax,Ay,'Color','b','LineWidth',1,'MaximumNumPoints',2);
p1=animatedline(Ax,Ay,'Color','[0.4 0.5 0.2]','LineWidth',2,'MaximumNumPoints',5);
grid on

video = VideoWriter('r_stepinput_3.mp4','MPEG-4');
open(video)

for i=1:10:n-1
    draw_animation2(x(i), th(i));
    F=getframe(gcf);
    writeVideo(video,F);
end
% close(video)


