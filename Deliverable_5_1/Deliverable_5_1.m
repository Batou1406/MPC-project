%clear all
close all
clc

addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Tf = 30; % time of flight
Ts = 1/20; % Sample time
x0 = zeros(12,1); % inital state
rocket = Rocket(Ts);

%linearize
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

%decompose in indepandant system
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% % Design MPC controller
H = 5; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);


% Setup reference function
%ref = @(t_, x_) rocket.MPC_ref(t_, Tf);

%simulate
rocket.mass = 1.783; % Manipulate mass for simulation
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
%[T, X, U, Ref, Z_hat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);
ph = rocket.plotvis(T, X, U, Ref);