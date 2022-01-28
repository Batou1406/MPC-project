clear all
close all
clc

%addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
rocket = Rocket(Ts);

%initial states
x0 = [0 0 0 0]';
y0 = [0 0 0 0]';
z0 = [0 0]';
roll0 = [0 0]';

%reference
x_pos_ref = -5;
y_pos_ref = -5;
z_pos_ref = -5;
roll_angle_ref = 0.785;

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% % Design MPC controller
H = 4; % Horizon length in seconds

mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

ux = mpc_x.get_u(x0, x_pos_ref)
uy = mpc_y.get_u(y0, y_pos_ref)
uz = mpc_z.get_u(z0, z_pos_ref)
uroll = mpc_roll.get_u(roll0, roll_angle_ref)

Tf = 20.0; % Time to simulate for


[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, x_pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_pos_ref);
  
[T, X_sub, U_sub] = rocket.simulate(sys_y, y0, Tf, @mpc_y.get_u, y_pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_pos_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_z, z0, Tf, @mpc_z.get_u, z_pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_pos_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_roll, z0, Tf, @mpc_roll.get_u, roll_angle_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_angle_ref);







