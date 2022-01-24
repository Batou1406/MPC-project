classdef MPC_Control_x < MPC_Control
    
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
            
            % System dynamics
            A = mpc.A;
            B = mpc.B;
            
            % Constraints
            % x in X = { x | Fx <= f }
            F = [0 1 0 0; 0 -1 0 0]; f = [0,0873; 0,0873];
            
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [0,26; 0,26];
            
            % Cost matrices
            Q = 10 * eye(2);
            R = 1;
            
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A,B,Q,R);
            
            % Compute maximal invariant set
            Xf = intersect(polytope(F,f) , polytope(M*K,m));
            Acl = [A+B*K];
            
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);
            
            % Visualizing the sets
            figure
            hold on; grid on;
            plot(polytope(F,f),'g'); plot(Xf,'r');
            xlabel('position'); ylabel('velocity');
            
            % Plot Terminal Invariant Set
            figure
            hold on; grid on;
            subplot(1,3,1)
            plot(projection(Xf,(1:2)),'b');
            title('Dimension 1');
            xlabel('Pitch velocity');
            ylabel('Pitch angle');

            subplot(1,3,2)
            plot(projection(Xf,(2:3)),'r');
            title('Dimension 2');
            xlabel('Pitch angle');
            ylabel('Velocity x');

            subplot(1,3,3)
            sgtitle('Terminal Invariant Set for X');
            plot(projection(Xf,(3:4)),'g');
            title('Dimension 3');
            xlabel('Velocity x');
            ylabel('Position x');
            
            % Defining the MPC controller
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (M*u(:,1) <= m);
            obj = u(:,1)'*R*u(:,1);
            for i = 2:N-1
                con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
                con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
                obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
            end
            con = con + (Ff*x(:,N) <= ff);
            obj = obj + x(:,N)'*Qf*x(:,N);


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
            obj = 0;
            con = [xs == 0, us == 0];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
