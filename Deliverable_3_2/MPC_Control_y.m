classdef MPC_Control_y < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            %objectives weight
            Q = eye(nx);
            R = 100*eye(nu);
            
            %state constraints
            F =[0,1,0,0;0,-1,0,0]; % |alpha| <= 5° 
            f = [0.0873;0.0873];
            
            %input constraints
            M = [1;-1];  % |d1| <= 15°
            m = [0.26;0.26];
            
            
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K = -K;
            
            %maximal invariant set is not require for 3.2
%             % Compute maximal invariant set
%             Xf = polytope([F;M*K],[f;m]);
%             Acl = [mpc.A+mpc.B*K];
%             while 1
%                 prevXf = Xf;
%                 [T,t] = double(Xf);
%                 preXf = polytope(T*Acl,t);
%                 Xf = intersect(Xf, preXf);
%                 if isequal(prevXf, Xf)
%                     break
%                 end
%             end
%             [Ff,ff] = double(Xf);
            
            obj = 0;
            con = [];
            
            con = ((X(:,2)-x_ref) == mpc.A*(X(:,1)-x_ref) + mpc.B*(U(:,1)-u_ref)) + (M*U(:,1) <= m);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            
            for i = 2:N-1
                con = [con, (X(:,i+1)- x_ref) == mpc.A*(X(:,i)- x_ref)+ mpc.B*(U(:,i) - u_ref)]; % System dynamics
                con = [con, F*X(:,i) <= f]; % State constraints
                con = [con, M*U(:,i) <= m]; % Input constraints
                obj = obj + (X(:,i) - x_ref)'*Q*(X(:,i) - x_ref) + (U(:,i) - u_ref)'*R*(U(:,i) - u_ref); % Cost function
            end
            
            %con = [con, Ff*X(:,N) <= ff]; % Terminal constraint
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); % Terminal weight
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, U(:,1));
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

            %input constraints
            M = [1;-1]; % |d2| <= 15°
            m = [0.26;0.26];      
            
            con = (xs == mpc.A*xs + mpc.B*us) + (M*us <= m) + (ref == mpc.C*xs);
            obj = us*us;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
