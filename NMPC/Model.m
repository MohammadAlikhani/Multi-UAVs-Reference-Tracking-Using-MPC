%%
import casadi.*

%% Casadi variables declaration
% Declare model variables
x1 = SX.sym('x1',1);
x2 = SX.sym('x2',1);
x3 = SX.sym('x3',1);
x4 = SX.sym('x4',1);
x5 = SX.sym('x5',1);
x6 = SX.sym('x6',1);
x = [x1; x2; x3; x4; x5; x6];
u1 = SX.sym('u1',1);
u2 = SX.sym('u2',1);
u = [u1; u2];
w1 = SX.sym('w1',1);
w2 = SX.sym('w2',1);
w = [w1; w2];

n_states = length(x);
n_controls = length(u);

dt = 0.1;

%% Planar Two Thrust Rotorcraft Models
% Parameters
m = 5;
beta1 = 0.8;
beta2 = 0.8;
g = 9.81;
L = 0.5;
I = 1/2*m*L^2;

% Wind disturbance for COM
we1 = -2.5;
we2 = 2.5;
we = [we1; we2];
v1 = (x(2)-we(1))*cos(x(5))+x(4)*sin(x(5));
v2 = -(x(2)-we(2))*sin(x(5))+x(4)*cos(x(5));
fe = -[cos(x(5))*beta1*v1*norm(v1) - sin(x(5))*beta2*v2*norm(v2);
       sin(x(5))*beta1*v1*norm(v1) + cos(x(5))*beta2*v2*norm(v2)];

% Control Oriented Model (COM)
dx_com = [x(1) + dt*x(2);
          x(2) + dt*(1/m*(-sin(x(5))*(u(1)+u(2)) + fe(1)));
          x(3) + dt*x(4);
          x(4) + dt*(1/m*(cos(x(5))*(u(1)+u(2)) + fe(2)) - g);
          x(5) + dt*x(6);
          x(6) + dt*(L/(I)*(u(1)-u(2)))];
f_COM = Function('f_COM',{x,u},{dx_com}); % nonlinear mapping function f(x,u)->(x_next)

% Simulation Oriented Model (SOM)
v1 = (x(2)-w(1))*cos(x(5))+x(4)*sin(x(5));
v2 = -(x(2)-w(2))*sin(x(5))+x(4)*cos(x(5));
f = -[cos(x(5))*beta1*v1*norm(v1) - sin(x(5))*beta2*v2*norm(v2);
      sin(x(5))*beta1*v1*norm(v1) + cos(x(5))*beta2*v2*norm(v2)];

dx_som = [x(1) + dt*x(2);
          x(2) + dt*(1/m*(-sin(x(5))*(u(1)+u(2)) + f(1)));
          x(3) + dt*x(4);
          x(4) + dt*(1/m*(cos(x(5))*(u(1)+u(2)) + f(2)) - g);
          x(5) + dt*x(6);
          x(6) + dt*(L/(I)*(u(1)-u(2)))];
f_SOM = Function('f_SOM',{x,u,w},{dx_som}); % nonlinear mapping function f(x,u,w)->(x_next)