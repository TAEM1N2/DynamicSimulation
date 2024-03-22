%% ================================
% Practice 2
% Pendulum Dynamic Simulation 
% 20191091 
% Taemin Kim 
%% =================================
clc;
clear all;
close all;


global c p

%% 시뮬레이션 준비

% 0 ~ 10초 dt=0.001초로 설정 
dt = 0.001;         % 시간 증분
t = 0:dt:10;        % 시뮬레이션 시간 
n = length(t);      % iteration 수 

x = zeros(1,n);
d_x = zeros(1,n);
th = zeros(1,n);    % 상태 angle(각도) 초기화
d_th = zeros(1,n); % 상태 d_x(각속도) 초기화

M = 10;             % 질량 of CART, [kg]
m = 1;              % 질량 of mass, [kg]
l = 1;              % 길이 [m]
g = 9.81;           % 중력가속도 [m/s^2]

% Initial condition 
x(1) = 0;
d_x(1) = 0; 
th(1) = 30 * pi/180;        % initial angle 
d_th(1) = 0;       % initial d_x
%% Iteration 
u = 0;   % 외력




for i=1:n-1
% Step 1. 운동방정식으로부터 다음시간의 가속도 구하기
    D2 = -(m*g*th(i))/M + u/M;
    D2_th = (g/l)*(1+(m/M))*th(i) - u/(M*l);
% Step 2. 가속도를 수치적분하여 속도 구하기 
    d_x(i+1) = d_x(i) + D2 * dt;
    d_th(i+1) = d_th(i) + D2_th * dt;
% Step 3. 속도를 수치적분하여 위치 구하기
    x(i+1) = x(i) + d_x(i+1)*dt;
    th(i+1) = th(i) + d_th(i+1)*dt;
end

% % 그래프 그리기
% figure(1);
% plot(t, x);
% title('Angle of Pendulum');
% xlabel('time(s)')
% ylabel('angle(deg)')
% grid on;
% 
% % 그래프 그리기
% figure(2);
% plot(t, th*180/pi);
% title('Position of Cart');
% xlabel('time(s)')
% ylabel('position')
% grid on;


%% Draw Animation
figure(20)
axis([-5. 5. -5. 5.])
ax=[0,0]; ay=[0,0];

c=animatedline(ax,ay,'Color','[0.4 0.8 0.2]','LineWidth',2,'MaximumNumPoints',5);
p=animatedline(ax,ay,'Color','b','LineWidth',1,'MaximumNumPoints',2);

grid on

video = VideoWriter('CartPole.mp4','MPEG-4');
open(video)

for i=1:10:n-1
    draw_animation(x(i), 0);
    F=getframe(gcf);
    writeVideo(video,F);
end
close(video)

