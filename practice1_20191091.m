%% ================================
% Practice 1 
% Mass-Spring Dynamic Simulation 
% 20191091 
% Taemin Kim 
%% =================================
clc;

global p p1

%M = m; 
%C = 0;
%G = k*x;
%tau = 0;
%D2 = inv(M)*(-C-G+tau);
%D2 = 1/M *(-C-G+tau);       % 가속도 계산

%% 시뮬레이션 준비

% 0 ~ 10초 dt=0.001초로 설정 
dt = 0.001;         % 시간 증분
t = 0:dt:10;        % 시뮬레이션 시간 
n = length(t);      % iteration 수 

x = zeros(1,n);     % 상태 x(위치) 초기화
d_x = zeros(1,n);   % 상태 d_x(속도) 초기화 

m = 1;              % 질량, [kg]
k = 100;            % 스프링계수, [N/m]

% Initial condition 
x(1) = 0.1;         % initial position
d_x(1) = 0;         % initial velocity     
%% Iteration 
for i=1:n-1


% Step 1. 운동방정식으로부터 다음시간의 가속도 구하기
    D2 = ( -k*x(i) + 0 )/m;

% Step 2. 가속도를 수치적분하여 속도 구하기 
    d_x(i+1) = d_x(i) + D2*dt;

% Step 3. 속도를 수치적분하여 위치 구하기
    x(i+1) = x(i) + d_x(i+1)*dt;

end

% 그래프 그리기
figure(7);
plot(t, x);
title('Position of Mass-Spring System');
xlabel('time(s)')
ylabel('Position(m)')
grid on;

% 그래프 그리기 2
figure(2);
plot(t,x,t,d_x);
title('Simulation of Mass-Spring System');
xlabel('time(s)')
grid on;
legend('pos','vel');


% 그래프 그리기 3
figure(3);
subplot(221)
plot(t,x)
title('Position of Mass-Spring System');
xlabel('time(s)')
ylabel('position(m)')
grid on;
hold on;

subplot(224)
plot(t,d_x)
title('Velocity of Mass-Spring System');
xlabel('time(s)')
ylabel('velocity(m/s)')
grid on;
hold on;


%% Draw Animation
figure(20)
axis([-0.2 0.2 -0.2 0.2])
Ax=[0,0];Ay=[0,0];
p=animatedline(Ax,Ay,'Color','b','LineWidth',1,'MaximumNumPoints',2);
p1=animatedline(Ax,Ay,'Color','[0.4 0.5 0.2]','LineWidth',2,'MaximumNumPoints',5);
grid on

video = VideoWriter('Practice1_20191091.mp4','MPEG-4');
open(video)

for i=1:10:n-1
    draw_animation(x(i),0);
    F=getframe(gcf);
    writeVideo(video,F);
end
close(video)

