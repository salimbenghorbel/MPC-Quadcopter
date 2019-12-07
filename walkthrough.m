%% Part 1
quad = Quad();

g = 9.81;
K = sum(quad.K(1,:))/4;
ustable = quad.mass*g/(4*K);
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1); % Initial state
u = ustable*ones(4,1); % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE 
quad.plot(sim, u); % Plot the result

%% Part 2
quad = Quad();
[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us) 
sys = quad.linearize(xs, us); % Linearize the nonlinear model

sys_transformed = sys * inv(quad.T);

%% 2.3
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Discretize 
Ts = 1/5;
discrete_system = c2d(sys, 1/5);

%% Constraints
alpha_max = 0.035; % rad
beta_max = 0.035; % rad
u_max = 1.5;
M_alpha_max = 0.3;
M_beta_max = 0.3;
M_gamma_max = 0.2;
F_max = 0.2;

%% 3.1
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs,us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% 
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
% Get control inputs with
x = [0;0.035;0;0];
x_position_reference = 0.035;
ux = mpc_x.get_u(x,x_position_reference);
%%
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw); 
quad.plot(sim);