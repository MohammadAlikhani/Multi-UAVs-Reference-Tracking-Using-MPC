function uout = MPCController(h, r, t)
persistent Controller
persistent N

if t == 0
    % Declare parameters
    Ts = 0.1; % Sampling time (s)
    %% Parameters
    m = 5;              % Mass (kg)
    g = 9.81;           % Gravitational acceleration (m/s^2)
    l = 0.30;           % Distance from center to motors (m)
    I = 0.5 * m * l^2;  % Moment of inertia (kg*m^2)

    % Linearized system matrices
    Ac = [0 1 0 0 0 0;
          0 0 0 0 g/m 0;
          0 0 0 1 0 0;
          0 0 0 0 0 0;
          0 0 0 0 0 1;
          0 0 0 0 0 0];
    
    Bc = [0 0;
          0 0;
          0 0;
          1/m 1/m;
          0 0;
          l/I -l/I];
    
    
    
    C = [1 0 0 0 0 0;   % y₁ = x₁
         0 0 1 0 0 0];  % y₂ = x₃

    
    % Discretize model
    sysc = ss(Ac, Bc, [],[]);
    sysd = c2d(sysc, Ts);
    
    Ad = sysd.A;
    Bd = sysd.B;
    
   
    N = 25; % Prediction horizon
    Q = 5 * eye(2); % Weighting matrix for outputs (x, y, theta)
    R = eye(2);     % Weighting matrix for inputs (u1, u2)
    
    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')
    nu = 2; % Number of inputs
    nx = 6; % Number of states
    ny = 2; % Number of outputs
    
    % Setup the optimization problem
    u = sdpvar(repmat(nu, 1, N), repmat(1, 1, N));
    x = sdpvar(repmat(nx, 1, N+1), repmat(1, 1, N+1));
    rvar = sdpvar(ny, 1); % Reference for [x, y]
    
    % Define MPC controller
    constraints = [];
    objective = 0;
    for k = 1:N
        objective = objective + (rvar - C*x{k})'*Q*(rvar - C*x{k}) + u{k}'*R*u{k};
        constraints = [constraints, x{k+1} == Ad*x{k} + Bd*u{k}];
        % Input constraints (thrusts limited by physical motor capabilities)
        constraints = [constraints, % -m*g <= u{k} <= m*g, -1000 <= x{k} <= 1000]
                       -2*m*g <= u{k}(1) + u{k}(2) <= 2*m*g,
                       -5 <= (l/I)*(u{k}(2) - u{k}(1)) <= 5,
                       -100 <= x{k}(1) <= 100,
                       -10 <= x{k}(2) <= 10,
                       -15 <= x{k}(3) <= 15,
                       -10 <= x{k}(4) <= 10,
                       -0.4 <= x{k}(5) <= 0.4,
                       -2 <= x{k}(6) <= 2];

    end
    
    % Solver settings
    ops = sdpsettings('verbose', 2, 'solver', 'mosek');
    Controller = optimizer(constraints, objective, ops, {x{1}, rvar}, [x, u]);
    
    % Initial evaluation
    [res, problem] = Controller(h, r);
    traj_x = res(1:N+1);
    traj_u = res(N+2:2*N+1);
    uout = traj_u{1};
else    
    % Subsequent iterations
    [res, problem] = Controller(h, r);
    traj_x = res(1:N+1);
    traj_u = res(N+2:2*N+1);
    uout = traj_u{1};
end
end