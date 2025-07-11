clear all; close all; clc

%%
import casadi.*
%%
% Run the model file to load f_COM, f_SOM, n_states, n_controls
run('Model.m');

% Define prediction horizon and parameters
N = 20; % Number of control intervals
P = SX.sym('P',n_states + 2); % Include initial conditions and reference point

% Weight matrices for each drone
Q1 = [1 0; 0 1]; % Drone 1 tracking weights
R1 = [5 0; 0 5]; % Drone 1 control weights
Q2 = [2 0; 0 2]; % Drone 2 tracking weights
R2 = [3 0; 0 3]; % Drone 2 control weights
Q3 = [0.5 0; 0 0.5]; % Drone 3 tracking weights
R3 = [7 0; 0 7]; % Drone 3 control weights

% Additional weights for Drone 2 and Drone 3 objective functions
Qv = [0.5 0; 0 0.5]; % Velocity penalty matrix
q_theta = 2; % Pitch penalty scalar
theta_ref = 0.1; % Desired pitch (level flight)
Q_N_factor = 5; % Terminal weight scaling factor

% Bounds
u_min = [0; 0];
u_max = [70; 70];
e_min = [-1; -1];
e_max = [1; 1];
x_min = [-inf;-2;1;-2;-pi/2;-pi/2];
x_max = [inf;2;25;2;pi/2;pi/2];

% Simulation parameters
T = 80; % Final time [s]
dt = 0.1;
nT = floor(T/dt); % Number of samples
L = 0.5; % Rotorcraft length parameter
y_offset = 2; % Y-offset for Drone 2 (+2) and Drone 3 (-2)

% Initial conditions for three drones
x_ini_1 = [0; 0; 10; 0; 0; 0]; % Drone 1
x_ini_2 = [5; 0; 8; 0; 0; 0]; % Drone 2
x_ini_3 = [-5; 0; 12; 0; 0; 0]; % Drone 3

% Storage for states and controls
store_x_1 = [x_ini_1];
store_x_2 = [x_ini_2];
store_x_3 = [x_ini_3];
store_u_1 = [];
store_u_2 = [];
store_u_3 = [];
comp_time = zeros(nT, 1); % Preallocate comp_time
t = 0;

% Preallocate reference arrays
ref_1 = zeros(nT, 2); % Reference for Drone 1
ref_2 = zeros(nT, 2); % Reference for Drone 2 (y + offset)
ref_3 = zeros(nT, 2); % Reference for Drone 3 (y - offset)
%%
% Initialize animated lines for three drones
traj_line_1 = animatedline('Color','r','LineWidth',1.5);
hold on;
traj_line_2 = animatedline('Color','m','LineWidth',1.5);
hold on;
traj_line_3 = animatedline('Color','c','LineWidth',1.5);
hold on;
ref_line_1 = animatedline('Color','b','LineWidth',1.5);
hold on;
predicted_line_1 = animatedline('Color','g','LineWidth',1,'LineStyle','--');
hold on;
predicted_line_2 = animatedline('Color','g','LineWidth',1,'LineStyle',':');
hold on;
predicted_line_3 = animatedline('Color','g','LineWidth',1,'LineStyle','-.');
hold on;
robot_line_1 = animatedline('Color','k','LineWidth',2);
hold on;
robot_line_2 = animatedline('Color','k','LineWidth',2);
hold on;
robot_line_3 = animatedline('Color','k','LineWidth',2);
grid on;
legend('Drone 1','Drone 2','Drone 3','Reference','interpreter','latex');
xlim([-10 110])
ylim([-5 30])

wd1 = 0;
wd2 = 0;
wd = [wd1; wd2];

xin = repmat(0,N*n_controls,1);
args = struct;

for i = 1:nT
    % Base reference trajectory (for Drone 1)
    reference = reference_trajectory(t); % Assumes reference_trajectory is defined
    ref_1(i,:) = reference';
    ref_2(i,:) = [reference(1), reference(2) + y_offset];
    ref_3(i,:) = [reference(1), reference(2) - y_offset];
    
    % Initialize total computation time for this iteration
    iter_comp_time = 0;
    
    % Simulate each drone
    for d = 1:3
        % Select initial state and reference for current drone
        if d == 1
            x_ini = x_ini_1;
            ref_d = reference;
            Q = Q1; % Use Drone 1 weights
            R = R1;
        elseif d == 2
            x_ini = x_ini_2;
            ref_d = [reference(1); reference(2) + y_offset];
            Q = Q2; % Use Drone 2 weights
            R = R2;
        else
            x_ini = x_ini_3;
            ref_d = [reference(1); reference(2) - y_offset];
            Q = Q3; % Use Drone 3 weights
            R = R3;
        end
        
        % Build NLP for this drone
        w = []; % Optimization variables
        lbw = []; % Lower bounds
        ubw = []; % Upper bounds
        obj = 0; % Objective function
        g = []; % Constraints
        lbg = []; % Constraint lower bounds
        ubg = []; % Constraint upper bounds
        Xk = P(1:n_states); % Initial state from parameters
        
        % Prediction horizon loop
        for k = 1:N
            % New NLP variable for control
            u1_opt = SX.sym(['U_' num2str(k)]);
            u2_opt = SX.sym(['U_' num2str(k+1)]);
            Uk = [u1_opt; u2_opt];
            w = [w; Uk];
            lbw = [lbw; u_min];
            ubw = [ubw; u_max];
            
            % Integrate dynamics
            Xk_next = f_COM(Xk,Uk);
            g = [g; Xk_next];
            lbg = [lbg; x_min];
            ubg = [ubg; x_max];
            
            % Compute objective terms
            Unorm = (Uk-u_min)./(u_max-u_min);
            X_obj = [Xk_next(1);Xk_next(3)]-P(n_states+1:end);
            Enorm = (X_obj-e_min)./(e_max-e_min);
            
            % Base objective (position and control)
            obj = obj + X_obj'*Q*X_obj + Unorm'*R*Unorm;
            
            % Additional terms for Drone 2 and Drone 3
            if d == 2 || d == 3
                % Velocity penalty
                V = [Xk_next(2); Xk_next(4)];
                obj = obj + V'*Qv*V;
                
                % Pitch penalty
                pitch = Xk_next(5);
                obj = obj + q_theta*(pitch - theta_ref)^2;
                
                % Terminal cost (added only at k == N)
                if k == N
                    Q_N = Q_N_factor * Q;
                    obj = obj + X_obj'*Q_N*X_obj;
                end
            end
            
            Xk = Xk_next; % Update state
        end
        
        % Encapsulate NLP problem
        nlp_prob = struct('f', obj, 'x', w, 'g', g, 'p', P);
        
        % Solver options
        opts = struct;
        opts.ipopt.warm_start_init_point = 'yes';
        opts.ipopt.print_level = 0;
        solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
        
        % Solve NLP
        args.x0 = xin;
        args.p = [x_ini; ref_d(1); ref_d(2)];
        tin = tic;
        sol = solver('x0', args.x0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', args.p);
        tfin = toc(tin);
        iter_comp_time = iter_comp_time + tfin;
        xin = full(sol.x);
        u = reshape(full(sol.x),n_controls,N)';
        
        % Update state using SOM
        next_x = f_SOM(x_ini,u(1,:)',wd);
        
        % Store results and select plotting lines
        if d == 1
            x_ini_1 = next_x;
            store_x_1 = [store_x_1 full(next_x)];
            store_u_1 = [store_u_1 u(1,:)'];
            traj_line = traj_line_1;
            predicted_line = predicted_line_1;
            robot_line = robot_line_1;
            addpoints(ref_line_1,reference(1),reference(2))
        elseif d == 2
            x_ini_2 = next_x;
            store_x_2 = [store_x_2 full(next_x)];
            store_u_2 = [store_u_2 u(1,:)'];
            traj_line = traj_line_2;
            predicted_line = predicted_line_2;
            robot_line = robot_line_2;
        else
            x_ini_3 = next_x;
            store_x_3 = [store_x_3 full(next_x)];
            store_u_3 = [store_u_3 u(1,:)'];
            traj_line = traj_line_3;
            predicted_line = predicted_line_3;
            robot_line = robot_line_3;
        end
        
        % Plotting
        addpoints(traj_line,full(x_ini(1)),full(x_ini(3)))
        clearpoints(predicted_line);
        predicted_states = full(sol.g);
        addpoints(predicted_line,predicted_states(1),predicted_states(3))
        for j = 1:N-1
            addpoints(predicted_line,predicted_states(1+j*n_states),predicted_states(3+j*n_states))
        end
        clearpoints(robot_line);
        addpoints(robot_line, full(x_ini(1))-(L+2)*cos(full(x_ini(5))),full(x_ini(3))-(L+2)*sin(full(x_ini(5))))
        addpoints(robot_line, full(x_ini(1)),full(x_ini(3)))
        addpoints(robot_line, full(x_ini(1))+(L+2)*cos(full(x_ini(5))),full(x_ini(3))+(L+2)*sin(full(x_ini(5))))
    end
    
    % Store total computation time
    comp_time(i) = iter_comp_time;
    
    drawnow limitrate
    
    % Update wind disturbance
    if t <= 20
        wd1 = -2.5;
        wd2 = 2.5;
    elseif t <= 60
        wd1 = 0;
        wd2 = 0;
    else
        wd1 = -2.5;
        wd2 = 2.5;
    end
    wd = [wd1; wd2];
    
    t = t + dt;
end

% Check sizes before error calculation
if size(ref_2,1) ~= size(store_x_2(1,1:end-1)',1)
    fprintf('Size mismatch for Drone 2: ref_2 has %d rows, store_x_2(1,1:end-1)'' has %d rows\n', ...
        size(ref_2,1), size(store_x_2(1,1:end-1)',1));
end
if size(ref_3,1) ~= size(store_x_3(1,1:end-1)',1)
    fprintf('Size mismatch for Drone 3: ref_3 has %d rows, store_x_3(1,1:end-1)'' has %d rows\n', ...
        size(ref_3,1), size(store_x_3(1,1:end-1)',1));
end

% Compute error for each drone
error_1 = sqrt((ref_1(:,1)-store_x_1(1,1:end-1)').^2 + (ref_1(:,2)-store_x_1(3,1:end-1)').^2);
error_2 = sqrt((ref_2(:,1)-store_x_2(1,1:end-1)').^2 + (ref_2(:,2)-store_x_2(3,1:end-1)').^2);
error_3 = sqrt((ref_3(:,1)-store_x_3(1,1:end-1)').^2 + (ref_3(:,2)-store_x_3(3,1:end-1)').^2);

% Plot results
figure
time_vector = 0:dt:(nT-1)*dt; % nT elements to match comp_time
plot(store_x_1(1,1:end-1)',store_x_1(3,1:end-1)', 'r', 'LineWidth',1.5), hold on
plot(store_x_2(1,1:end-1)',store_x_2(3,1:end-1)', 'm', 'LineWidth',1.5)
plot(store_x_3(1,1:end-1)',store_x_3(3,1:end-1)', 'c', 'LineWidth',1.5)
plot(ref_1(:,1),ref_1(:,2), 'b', 'LineWidth',1.5)
xlim([-10 110])
ylim([-5 30])
grid on
xlabel('x [m]')
ylabel('y [m]')
legend('Drone 1','Drone 2','Drone 3','Reference','interpreter','latex')
title('Trajectories of Three Drones')

% Plot solving time
figure
if nT >= 10
    plot(time_vector(10:end),comp_time(10:end), 'LineWidth',1.5), hold on
else
    plot(time_vector(1:end),comp_time(1:end), 'LineWidth',1.5), hold on
end
meancomp = mean(comp_time);
yline(meancomp,'--k','LineWidth',2);
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Solving time [s]')
legend('Solving time','Mean','interpreter','latex')
title('Solving time')

% Plot controls and pitch for Drone 1
figure
subplot(2,1,1)
plot(time_vector(1:end),store_u_1(1,:)', 'r', 'LineWidth',1.5), hold on
plot(time_vector(1:end),store_u_1(2,:)', 'r--', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Thrust [N]')
legend('Thrust 1','Thrust 2','interpreter','latex')
title('Drone 1 Controls')
subplot(2,1,2)
plot(time_vector,store_x_1(5,1:end-1)', 'r', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Pitch [rad]')
legend('Pitch','interpreter','latex')
title('Drone 1 Pitch')

% Plot controls and pitch for Drone 2
figure
subplot(2,1,1)
plot(time_vector(1:end),store_u_2(1,:)', 'm', 'LineWidth',1.5), hold on
plot(time_vector(1:end),store_u_2(2,:)', 'm--', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Thrust [N]')
legend('Thrust 1','Thrust 2','interpreter','latex')
title('Drone 2 Controls')
subplot(2,1,2)
plot(time_vector,store_x_2(5,1:end-1)', 'm', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Pitch [rad]')
legend('Pitch','interpreter','latex')
title('Drone 2 Pitch')

% Plot controls and pitch for Drone 3
figure
subplot(2,1,1)
plot(time_vector(1:end),store_u_3(1,:)', 'c', 'LineWidth',1.5), hold on
plot(time_vector(1:end),store_u_3(2,:)', 'c--', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Thrust [N]')
legend('Thrust 1','Thrust 2','interpreter','latex')
title('Drone 3 Controls')
subplot(2,1,2)
plot(time_vector,store_x_3(5,1:end-1)', 'c', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Pitch [rad]')
legend('Pitch','interpreter','latex')
title('Drone 3 Pitch')

% Plot velocities for Drone 1
figure
subplot(3,1,1)
plot(time_vector,store_x_1(2,1:end-1)', 'r', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Lateral Velocity','interpreter','latex')
title('Drone 1 Lateral Velocity')
subplot(3,1,2)
plot(time_vector,store_x_1(4,1:end-1)', 'r', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Vertical Velocity','interpreter','latex')
title('Drone 1 Vertical Velocity')
subplot(3,1,3)
plot(time_vector,store_x_1(6,1:end-1)', 'r', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Rate [rad/s]')
legend('Angular Velocity','interpreter','latex')
title('Drone 1 Angular Velocity')

% Plot velocities for Drone 2
figure
subplot(3,1,1)
plot(time_vector,store_x_2(2,1:end-1)', 'm', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Lateral Velocity','interpreter','latex')
title('Drone 2 Lateral Velocity')
subplot(3,1,2)
plot(time_vector,store_x_2(4,1:end-1)', 'm', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Vertical Velocity','interpreter','latex')
title('Drone 2 Vertical Velocity')
subplot(3,1,3)
plot(time_vector,store_x_2(6,1:end-1)', 'm', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Rate [rad/s]')
legend('Angular Velocity','interpreter','latex')
title('Drone 2 Angular Velocity')

% Plot velocities for Drone 3
figure
subplot(3,1,1)
plot(time_vector,store_x_3(2,1:end-1)', 'c', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Lateral Velocity','interpreter','latex')
title('Drone 3 Lateral Velocity')
subplot(3,1,2)
plot(time_vector,store_x_3(4,1:end-1)', 'c', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Vertical Velocity','interpreter','latex')
title('Drone 3 Vertical Velocity')
subplot(3,1,3)
plot(time_vector,store_x_3(6,1:end-1)', 'c', 'LineWidth',1.5)
xlim([0 T])
grid on
xlabel('Time [s]')
ylabel('Rate [rad/s]')
legend('Angular Velocity','interpreter','latex')
title('Drone 3 Angular Velocity')

% Compute and display MSE for each drone
MSE_x_1 = immse(store_x_1(1,1:end-1)',ref_1(:,1));
MSE_y_1 = immse(store_x_1(3,1:end-1)',ref_1(:,2));
MSE_x_2 = immse(store_x_2(1,1:end-1)',ref_2(:,1));
MSE_y_2 = immse(store_x_2(3,1:end-1)',ref_2(:,2));
MSE_x_3 = immse(store_x_3(1,1:end-1)',ref_3(:,1));
MSE_y_3 = immse(store_x_3(3,1:end-1)',ref_3(:,2));
disp(['Drone 1 MSE x: ', num2str(MSE_x_1)])
disp(['Drone 1 MSE y: ', num2str(MSE_y_1)])
disp(['Drone 2 MSE x: ', num2str(MSE_x_2)])
disp(['Drone 2 MSE y: ', num2str(MSE_y_2)])
disp(['Drone 3 MSE x: ', num2str(MSE_x_3)])
disp(['Drone 3 MSE y: ', num2str(MSE_y_3)])