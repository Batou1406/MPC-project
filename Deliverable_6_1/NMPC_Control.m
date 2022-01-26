function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
Q= diag([1,1,0.1,1,1,50,0.1,0.1,0.1,100,100,200]) ;
R= diag([1,1,0.1,0.3]);

H = [0 0 0 0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 0 0 0 1; 0 0 0 0 0 1 0 0 0 0 0 0]';

%terminal weight
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
sys_d = c2d(sys, rocket.Ts);
% Compute LQR controller for unconstrained system
[K,Qf,~] = dlqr(sys_d.A,sys_d.B,Q,R);


big_Q = blkdiag(kron(eye(N-1),Q),Qf);
big_R = kron(eye(N-1),R);


%initial state
opti.subject_to(X_sym(:,1) == x0_sym);

% objective
opti.minimize((vec(X_sym - kron(ones(1,N),H*ref_sym)))'*big_Q*(vec(X_sym - kron(ones(1,N),H*ref_sym))) +  (vec(U_sym))'*big_R*vec(U_sym));

for i = 1:N-1
    % objective
    %opti.minimize((X_sym(:,i) - H*ref_sym)'*Q*(X_sym(:,i) - H*ref_sym) +  (U_sym(:,i))'*R*(U_sym(:,i)));
    
    % constraint
    %state dynamics
    opti.subject_to(X_sym(:,i+1) == f_discrete(X_sym(:,i), U_sym(:,i),rocket.Ts,rocket)); 
    
    %input constraints
    opti.subject_to(-0.26 <= U_sym(1,i) <= 0.26);  % |d1| < 15°
    opti.subject_to(-0.26 <= U_sym(2,i) <= 0.26);  % |d2| < 15°
    opti.subject_to(50 <= U_sym(3,i) <= 80);  % 50% < Pavg < 80%
    opti.subject_to(-20 <= U_sym(4,i) <=20);  % -20% < Pdiff < 20%
    
    %state constraints
    opti.subject_to(-1.48 <= X_sym(5,i) <=1.48);  % -85° < beta < 85°

end

%opti.minimize((X_sym(:,N) - H*ref_sym)'*Qf*(X_sym(:,N) - H*ref_sym)); % Terminal weight

% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
