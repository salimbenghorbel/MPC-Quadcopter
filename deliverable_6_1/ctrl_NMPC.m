function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 15; % MPC horizon [SET THIS VARIABLE]

% ---- decision variables ---------
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)

X0  = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE BELOW %%%%
%%%%%%%%%%%%%%%%%%%%%%%%

% System dynamics
h = quad.Ts;
f_quad = @(x,u) RK4_quad(x,u,quad.Ts,quad);



% steady-state variables
Xs = opti.variable(12,1); % steady-state trajectory variables
Us = opti.variable(4, 1); % steady-state input


%% perform control action

gain_xs_ref = 1000;
gain_x_xs = 1;
gain_u_us = 1;

sum_minimize = gain_xs_ref * (Xs([10,11,12,6])-REF)'*(Xs([10,11,12,6])-REF);
for i = 1:N
    sum_minimize = sum_minimize + gain_x_xs * (X(:,i)-Xs)'*(X(:,i)-Xs);
    sum_minimize = sum_minimize + gain_u_us * (U(:,i)-Us)'*(U(:,i)-Us);
end
sum_minimize = sum_minimize + gain_x_xs * (X(:,N+1)-Xs)'*(X(:,N+1)-Xs);

opti.minimize(sum_minimize);

% boundary conditions
angle_limit = 0.035;
u_min = quad.thrustLimits(1,1);
u_max = quad.thrustLimits(2,1);

% steady-state conditions
            
opti.subject_to(Xs == f_quad(Xs,Us));
opti.subject_to( -angle_limit < Xs(4,:) < angle_limit);
opti.subject_to( -angle_limit < Xs(5,:) < angle_limit);
opti.subject_to(u_min <= Us <= u_max); 

% loop over control intervals
for k=1:N
    opti.subject_to(X(:,k+1) == f_quad(X(:,k), U(:,k)));
end

% angle conditions
opti.subject_to( -angle_limit < X(4,:) < angle_limit);
opti.subject_to( -angle_limit < X(5,:) < angle_limit);

% input conditions
opti.subject_to(u_min <= U <= u_max);  % control is limited

% boundary conditions
opti.subject_to(X(:,1)==X0);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE ABOVE%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end
