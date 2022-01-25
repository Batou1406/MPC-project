addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

%linearize
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% % Design MPC controller
H = 5; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
%mpc_y = MPC_Control_y(sys_y, Ts, H);
%mpc_z = MPC_Control_z(sys_z, Ts, H);
%mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

Tf = 20.0; % Time to simulate for

% x0 = [0,0,0,5]'; %initial state [wy, beta, vx ,x]
% [T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
% 
% x0 = [0,0,0,5]'; %initial state [wx, alpha, vy, y]
% [T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, 0);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
% 
% x0 = [0,5]'; %initial state [vz, z]
% [T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, 0);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
% 
% x0 = [0,deg2rad(45)]'; %initial state [vz, z]
% [T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, 0);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


% Get control input
%ux = mpc_x.get_u(x)
