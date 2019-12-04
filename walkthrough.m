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