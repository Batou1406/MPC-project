function [x_next] = f_discrete(x,u,h,rocket)
% Inputs : 
%    x, u current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    x_next : discrete integration of the state at time h in the futur
%
% Runge-Kutta 4 integration

   k1 = rocket.f(x, u);
   k2 = rocket.f(x+h/2*k1, u);
   k3 = rocket.f(x+h/2*k2, u);
   k4 = rocket.f(x+h*k3,   u);
   x_next = x + h/6*(k1+2*k2+2*k3+k4);
end
