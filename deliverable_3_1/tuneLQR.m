%%
clc
clear all
close all
%% Decompose system
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs,us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
%% Tune Q,R gains _x
sys_d = c2d(sys_x, Ts);
[A,B,C,D] = ssdata(sys_d);

[n,m] = size(B);


% create combination structure
Q_range = 1:20;
R_range = 1:20;
% Q_range = logspace(-5,5,11);
% R_range = logspace(-5,5,11);
comb = struc(R_range,Q_range);
nb_comb = size(comb,1);
settling_time = zeros(nb_comb,1);

for k=1:nb_comb
    % cost Q,R
    R = comb(k,1);
    Q = comb(k,2) * eye(n);
    % Compute LQR controller for unconstrained system
    [K,Qf,~] = dlqr(A,B,Q,R);
    % MATLAB defines K as -K, so invert its signal
    K = -K;
    A_lqr = A+B*K;
    sys_lqr = ss(A_lqr,B,C,D,Ts);
    settling_time(k) = stepinfo(sys_lqr).SettlingTime;

end


% find first element to specify 8seconds settling time
[settling_time_opt_x,I] = min(settling_time);
disp('optimal parameters')
R_opt_x = comb(I,1)
Q_opt_x = comb(I,2)
settling_time_opt_x



%% Tune Q,R gains _y
sys_d = c2d(sys_y, Ts);
[A,B,C,D] = ssdata(sys_d);

[n,m] = size(B);


% create combination structure
Q_range = 1:20;
R_range = 1:20;
% Q_range = logspace(-5,5,11);
% R_range = logspace(-5,5,11);
comb = struc(R_range,Q_range);
nb_comb = size(comb,1);
settling_time = zeros(nb_comb,1);

for k=1:nb_comb
    % cost Q,R
    R = comb(k,1);
    Q = comb(k,2) * eye(n);
    % Compute LQR controller for unconstrained system
    [K,Qf,~] = dlqr(A,B,Q,R);
    % MATLAB defines K as -K, so invert its signal
    K = -K;
    A_lqr = A+B*K;
    sys_lqr = ss(A_lqr,B,C,D,Ts);
    settling_time(k) = stepinfo(sys_lqr).SettlingTime;

end


% find first element to specify 8seconds settling time
[settling_time_opt_y,I] = min(settling_time);
disp('optimal parameters')
R_opt_y = comb(I,1)
Q_opt_y = comb(I,2)
settling_time_opt_y

%% Tune Q,R gains _z
sys_d = c2d(sys_z, Ts);
[A,B,C,D] = ssdata(sys_d);

[n,m] = size(B);


% create combination structure
Q_range = 1:20;
R_range = 1:20;
% Q_range = logspace(-5,5,11);
% R_range = logspace(-5,5,11);
comb = struc(R_range,Q_range);
nb_comb = size(comb,1);
settling_time = zeros(nb_comb,1);
settling = zeros(nb_comb,1);

for k=1:nb_comb
    % cost Q,R
    R = comb(k,1);
    Q = comb(k,2) * eye(n);
    % Compute LQR controller for unconstrained system
    [K,Qf,~] = dlqr(A,B,Q,R);
    % MATLAB defines K as -K, so invert its signal
    K = -K;
    A_lqr = A+B*K;
    sys_lqr = ss(A_lqr,B,C,D,Ts);
    settling_time(k) = stepinfo(sys_lqr).SettlingTime;
end


% find first element to specify 8seconds settling time
[settling_time_opt_z,I] = min(settling_time);
disp('optimal parameters')
R_opt_z = comb(I,1)
Q_opt_z = comb(I,2)
settling_time_opt_z


%% Tune Q,R gains _yaw
sys_d = c2d(sys_yaw, Ts);
[A,B,C,D] = ssdata(sys_d);

[n,m] = size(B);


% create combination structure
Q_range = 1:20;
R_range = 1:20;
% Q_range = logspace(-5,5,11);
% R_range = logspace(-5,5,11);
comb = struc(R_range,Q_range);
nb_comb = size(comb,1);
settling_time = zeros(nb_comb,1);

for k=1:nb_comb
    % cost Q,R
    R = comb(k,1);
    Q = comb(k,2) * eye(n);
    % Compute LQR controller for unconstrained system
    [K,Qf,~] = dlqr(A,B,Q,R);
    % MATLAB defines K as -K, so invert its signal
    K = -K;
    A_lqr = A+B*K;
    sys_lqr = ss(A_lqr,B,C,D,Ts);
    settling_time(k) = stepinfo(sys_lqr).SettlingTime;

end


% find first element to specify 8seconds settling time
[settling_time_opt_yaw,I] = min(settling_time);
disp('optimal parameters')
R_opt_yaw = comb(I,1)
Q_opt_yaw = comb(I,2)
settling_time_opt_yaw