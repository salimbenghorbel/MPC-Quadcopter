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
ctrl = quad.merge_controllers(mpc_x, mpc_y, mpc_z, mpc_yaw);

%% get decomposed indices
ind = quad.ind;
I_x = [ind.omega(2) ind.theta(2) ind.vel(1) ind.pos(1)];
I_y = [ind.omega(1) ind.theta(1) ind.vel(2) ind.pos(2)];
I_z = [ind.vel(3) ind.pos(3)];
I_yaw = [ind.omega(3) ind.theta(3)];

%%
Tf = 10;
nbSteps = ceil(Tf/Ts);
x0 = zeros(12,1); % Initial state
%x0(10:12) = 2;
%x0(I_yaw) = [0,45*pi/180];

ref = zeros(4,1); % Reference state
ref(1:3) = -2;
ref(4) = 45*pi/180;

v = zeros(4,nbSteps-1);
u = zeros(4,nbSteps-1);
states = zeros(12,nbSteps);
states(:,1) = x0;
for i=1:nbSteps-1
    u(:,i)  = ctrl(states(:,i),ref);
    states(:,i+1) = quad.step(states(:,i), u(:,i), Ts);
end
time = (0:nbSteps-1)*Ts;
settling_time = time(find(vecnorm(states([10,11,12,6],:)-ref,2,1)>0.05*vecnorm(x0([10,11,12,6])-ref),1,'last'))

%% plot
sim = struct();
sim.x = time';
sim.y = states;
quad.plot(sim,us);


