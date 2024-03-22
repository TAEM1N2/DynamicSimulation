% for Mass-Spring Simulation
%
 
function draw_animation(x1, y1, x2, y2)
global p p1 p2

% Addpoints

addpoints(p,[x1-0.04 x1+0.04 x1+0.04 x1-0.04 x1-0.04], [y1-0.05 y1-0.05 y1+0.05 y1+0.05 y1-0.05]);
addpoints(p1,[x1 x2],[y1 y2]);
addpoints(p2,[x2-0.02 x2+0.02 x2+0.02 x2-0.02 x2-0.02], [y2-0.02 y2-0.02 y2+0.02 y2+0.02 y2-0.02]);

%hold on
drawnow
%hold on
pause(0.01);
end



