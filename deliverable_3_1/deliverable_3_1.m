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
ctrl = quad.merge_controllers_no_ref(mpc_x, mpc_y, mpc_z, mpc_yaw);
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

%% Start at 45 yaw
Tf = 10;
nbSteps = ceil(Tf/Ts);
x0 = zeros(12,1); % Initial state
x0(I_yaw) = [0,45*pi/180];
xend = zeros(size(x0));
v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
states = zeros(12,nbSteps);
states(:,1) = x0;
% create simulation with control action 
for i=1:nbSteps-1
    u(:,i)  = ctrl(states(:,i));
    states(:,i+1) = quad.step(states(:,i), u(:,i), Ts);
end
time = (0:nbSteps-1)*Ts;
settling_time_yaw = time(find(vecnorm(states,2,1)>0.05*vecnorm(x0),1,'last'))

%% plot simulation 45 yaw
sim = struct();
sim.x = time';
sim.y = states;
figure
quad.plot(sim,[u,u(:,end)]);

%% Start at x,y,z = 2
Tf = 10;
nbSteps = ceil(Tf/Ts);
x0 = zeros(12,1); % Initial state
x0(10:12) = 2;
xend = zeros(size(x0));
v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
states = zeros(12,nbSteps);
states(:,1) = x0;
% create simulation with control action 
for i=1:nbSteps-1
    u(:,i)  = ctrl(states(:,i));
    states(:,i+1) = quad.step(states(:,i), u(:,i), Ts);
end
time = (0:nbSteps-1)*Ts;
settling_time_xyz = time(find(vecnorm(states,2,1)>0.05*vecnorm(x0),1,'last'))

%% plot simulation x,y,z = 2
sim = struct();
sim.x = time';
sim.y = states;
figure
quad.plot(sim,[u,u(:,end)]);
