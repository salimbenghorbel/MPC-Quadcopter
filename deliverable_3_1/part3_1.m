%%
clc
clear all
close all
%% Decompose system
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs,us);
sysd = c2d(sys,Ts);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Control action
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Get control inputs with
x = [0;0;0;0];
vx = mpc_x.get_u(x);
y = [0;0;0;0];
vy = mpc_y.get_u(y);
z = [0;2];
vz = mpc_z.get_u(z);
yaw = [0;0];
vyaw = mpc_yaw.get_u(yaw);

%% get decomposed indices
ind = quad.ind;
I_x = [ind.omega(2) ind.theta(2) ind.vel(1) ind.pos(1)];
I_y = [ind.omega(1) ind.theta(1) ind.vel(2) ind.pos(2)];
I_z = [ind.vel(3) ind.pos(3)];
I_yaw = [ind.omega(3) ind.theta(3)];

%%
nbSteps = 1000;
x0 = zeros(12,1); % Initial state
x0(12) = 2;
v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
states = zeros(12,nbSteps);
states(:,1) = x0;
for i=1:nbSteps-1
    %v(:,i) = [mpc_z.get_u(states(I_z,i));mpc_y.get_u(states(I_y,i));mpc_x.get_u(states(I_x,i));mpc_yaw.get_u(states(I_yaw,i))];
    v(:,i) = [mpc_z.get_u(states(I_z,i));0;0;0];
    u(:,i) = quad.T\v(:,i)+us;
%     states(:,i+1) = states(:,i) + Ts*quad.f(states(:,i),u(:,i));
%     states(:,i+1) = sysd.A *states(:,i) + sysd.B*u(:,i);
    states(:,i+1) = quad.step(states(:,i), u(:,i), Ts);
end
time = (0:nbSteps-1)*Ts;
settling_time = time(find(states(12,:)>0.05*x0(12),1,'last'))
% figure
% plot(time,states(10:12,:));
%% plot
sim = struct();
sim.x = time';
sim.y = states;
quad.plot(sim,us);
%% simulate
% Tf = 200;
% sim = ode45(@(t, x) quad.f(x, quad.T\[mpc_z.get_u(x(I_z)) ;0;0;0]+us), [0, Tf], x0); % Solve the system ODE 
% quad.plot(sim,us); % Plot the result

