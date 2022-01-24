Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)

w = [0,0,0];
phi = [0,0,0];
v = [0,0,0];
p = [0,0,0];
xs = [w, phi, v, p]';
 
sys = rocket.linearize(xs, us)