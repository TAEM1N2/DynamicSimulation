% for Mass-Spring Simulation
%
 
function draw_animation2(x1,th)
global p p1
global l

% Addpoints
addpoints(p,[x1 x1+l*sin(th)],[0 0+l*cos(th)]);
addpoints(p1,[x1-0.05 x1+0.05 x1+0.05 x1-0.05 x1-0.05],...
             [ 0-0.01  0-0.01  0+0.01  0+0.01  0-0.01]);

%hold on
drawnow
%hold on
pause(0.01);
end



