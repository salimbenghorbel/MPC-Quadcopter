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

% opti.minimize(...
%     1000*(Xs(10)-REF(1))'*(Xs(10)-REF(1))  + ... % min x-xref tracking
%     1000*(Xs(11)-REF(2))'*(Xs(11)-REF(2))  + ... % min y-yref tracking
%     1000*(Xs(12)-REF(3))'*(Xs(12)-REF(3))  + ... % min z-zref tracking
%     1000*(Xs(6)-REF(4))'*(Xs(6)-REF(4))  + ... % min yaw-yawref tracking
%     100*(X(1,:)-Xs(1))*(X(1,:)-Xs(1))'  + ... % min x-xs tracking
%     100*(X(2,:)-Xs(2))*(X(2,:)-Xs(2))'  + ... % min x-xs tracking
%     100*(X(3,:)-Xs(3))*(X(3,:)-Xs(3))'  + ... % min x-xs tracking
%     100*(X(4,:)-Xs(4))*(X(4,:)-Xs(4))'  + ... % min x-xs tracking
%     100*(X(5,:)-Xs(5))*(X(5,:)-Xs(5))'  + ... % min x-xs tracking
%     100*(X(6,:)-Xs(6))*(X(6,:)-Xs(6))'  + ... % min x-xs tracking
%     100*(X(7,:)-Xs(7))*(X(7,:)-Xs(7))'  + ... % min x-xs tracking
%     100*(X(8,:)-Xs(8))*(X(8,:)-Xs(8))'  + ... % min x-xs tracking
%     100*(X(9,:)-Xs(9))*(X(9,:)-Xs(9))'  + ... % min x-xs tracking
%     100*(X(10,:)-Xs(10))*(X(10,:)-Xs(10,:))'  + ... % min x-xs tracking
%     100*(X(11,:)-Xs(11))*(X(11,:)-Xs(11,:))'  + ... % min x-xs tracking
%     100*(X(12,:)-Xs(12))*(X(12,:)-Xs(12,:))'  + ... % min x-xs tracking
%     1000*(U(1,:)-Us(1))*(U(1)-Us(1))'   + ... % Minimize input
%     1000*(U(2,:)-Us(2))*(U(2)-Us(2))'   + ... % Minimize input
%     1000*(U(3,:)-Us(3))*(U(3)-Us(3))'   + ... % Minimize input
%     1000*(U(4,:)-Us(4))*(U(4)-Us(4))'   );    % Minimize input

gain_xs_ref = 5000;
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
