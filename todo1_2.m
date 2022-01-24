Ts = 1/20;
rocket = Rocket(Ts);
Tf = 2.0; % Time to simulate for

%x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state

%simple initial state
w = [0,0,0];
phi = [0,0,0];
v = [0,0,0];
p = [0,0,0];
x0 = [w, phi, v, p]';

%Control input
% %ascend vertically
% d1 = 0;
% d2 = 0;
% Pavg = 60;
% Pdiff = 0;
% 
% %descend vertically
% d1 = 0;
% d2 = 0;
% Pavg = 55;
% % Pdiff = 0;
% 
% %turn arround x
% d1 = 0;
% d2 = 0.1;
% Pavg = 55;
% Pdiff = 0;
% 
% %turn arround y
% d1 = 0.1;
% d2 = 0;
% Pavg = 55;
% Pdiff = 0;
% 
% %turn arround z
% d1 = 0;
% d2 = 0;
% Pavg = 56;
% Pdiff = 10;
% 
% %move along x
% d1 = 0.2;
% d2 = 0;
% Pavg = 57;
% Pdiff = 0;
% 
% %move along y
% d1 = 0;
% d2 = 0.2;
% Pavg = 57;
% Pdiff = 0;

%hower in space
d1 = 0.05;
d2 = 0.1;
Pavg = 73;
Pdiff =10;

u = [deg2rad([d1 d2]), Pavg, Pdiff ]'; 

[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U); % Trajectory visualization at 1.0x real−time