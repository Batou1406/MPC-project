addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

%linearize
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%discretize the model
sys_d_x = c2d(sys_x, Ts);
sys_d_y = c2d(sys_y, Ts);
sys_d_z = c2d(sys_z, Ts);
sys_d_roll = c2d(sys_roll, Ts);


% % Design MPC controller
H = 10; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
% % Get control input
% ux = mpc_x.get_u(x)
