Ts = 1/20;
rocket = Rocket(Ts);

d1 = 0;
d2 = 0;
Pavg = 57;
Pdiff = 0;

u = [d1, d2, Pavg, Pdiff]'; 

[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)

w = [0,0,0];
phi = [0,0,0];
v = [0,0,0];
p = [0,0,0];

x = [w, phi, v, p]';

x_dot = rocket.f(x, u)
