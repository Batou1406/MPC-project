addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Tf = 30;
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

rocket.mass = 1.783; % Manipulate mass for simulation
%[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
[T, X, U, Ref, Z_hat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);
ph = rocket.plotvis(T, X, U, Ref);