%% ================================
% Practice 2
% Pendulum Dynamic Simulation 
% 20191091 
% Taemin Kim 
%% =================================
clc;
clear all;
close all;


global p p1

%% 시뮬레이션 준비

% 0 ~ 10초 dt=0.001초로 설정 
dt = 0.001;         % 시간 증분
t = 0:dt:10;        % 시뮬레이션 시간 
n = length(t);      % iteration 수 

ang = zeros(1,n);   % 상태 angle(각도) 초기화
d_ang = zeros(1,n); % 상태 d_ang(각속도) 초기화    

m = 1;              % 질량, [kg]
l = 1;              % 길이 [m]
g = 9.81;           % 중력가속도 [m/s^2]

% Initial condition 
ang(1) = 30 * pi/180;        % initial angle 
d_ang(1) = 0;       % initial d_ang
%% Iteration 
for i=1:n-1

tau = 0;        %진자 가해지는 토크 

% Step 1. 운동방정식으로부터 다음시간의 가속도 구하기
    D2 = tau/m*l^2 - g*sin(ang(i))/l;

% Step 2. 가속도를 수치적분하여 속도 구하기 
    d_ang(i+1) = d_ang(i) + D2 * dt;

% Step 3. 속도를 수치적분하여 위치 구하기
    ang(i+1) = ang(i) + d_ang(i+1)*dt;

end

% 그래프 그리기
figure(7);
plot(t, ang*180/pi);
title('Angle of Pendulum');
xlabel('time(s)')
ylabel('angle(deg)')
grid on;


%% Draw Animation
figure(20)
axis([-1. 1. -2. 0.])
Ax=[0,0];Ay=[0,0];
p=animatedline(Ax,Ay,'Color','b','LineWidth',1,'MaximumNumPoints',2);
p1=animatedline(Ax,Ay,'Color','[0.4 0.5 0.2]','LineWidth',2,'MaximumNumPoints',5);
grid on

video = VideoWriter('Pendulam.mp4','MPEG-4');
open(video)

for i=1:10:n-1
    draw_animation(l*sin(ang(i)), -l*cos(ang(i)));
    F=getframe(gcf);
    writeVideo(video,F);
end
close(video)

