% for Mass-Spring Simulation
%
 
function draw_animation(x1,z1)
global p p1

% Addpoints
addpoints(p,[0 x1],[0 z1]);
addpoints(p1,[x1-0.02 x1+0.02 x1+0.02 x1-0.02 x1-0.02],...
             [z1-0.05 z1-0.05 z1+0.05 z1+0.05 z1-0.05]);
addpoints(pw, [])

%hold on
drawnow
%hold on
pause(0.01);
end



