%% Trajectory
function r = reference_trajectory(t)
A = 8;
w = 0.01*4*pi;
r = [t; 10 + A*sin(w*t)]';





