%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef HastedtAgent < DoubleIntegratorAgent
    
    properties
        d;              % equilibrium distance(or desired distance) to neighbours
        do;             % equilibrium distance(or desired distance) to obstacles
        lambda;         % weight on control action
        lambda_o;       % weight on obstacle avoidance
        lambda_a;       % weight on positive deviation form desired distance
        lookAhead;      % look ahead distance for reference
        q_pos;          % reference position weighting factor
        q_vel;          % reference velocity weighting factor
        Hp;             % prediction horizon
        m;              % state dimensions
        u_max;          % maximum input
        obstacles;      % obstacle matrix
        reference;      % reference 
        ro;             % obstacle interaction range
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;          % number of neighbors
        num_O;          % number of obstacles
        neighbors;      % index of neighbors
        U_opt;          % optimal control input     
    end
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        T;              % sampling time
        p_x;            % single agent state transition matrix
        p_u;            % single agent input matrix
        numberAgents;   % number of agents in simulation
    end
    
    methods
        P_x = calculatePx(obj, size);
        P_u = calculatePu(obj, size, isZero);
        Tf = transformationMatrix(obj, dimension, size);
        h = calculateH(obj, xi, size, desiredDistance);
        J = constructJacobian(obj, x0, h0, size, desiredDistance);
        fillUopt(obj,U,messages);
    end
    
    methods(Static)
        function e_nm = e_nm(n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end     
    end
    
    methods
        %HastedtAgent Initialize agent
        % Inputs:
        %   id          : agent id
        %   param       : struct containing simulation parameters
        %   initialPos  : initial position
        %   initialVel  : initial velocity
        %   configFile  : file containing agent configuration parameters
        function obj = HastedtAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@DoubleIntegratorAgent(id, param.dT, initialPos, initialVel);
            
            % load data from config file
            obj.d = cfg.d;
            obj.do = cfg.do;
            obj.lambda = cfg.lambda;
            obj.lambda_o = cfg.lambda_o;
            obj.lookAhead = cfg.lookAhead;
            obj.q_pos = cfg.q_pos;
            obj.q_vel = cfg.q_vel;
            obj.Hp = cfg.Hp;
            obj.m = cfg.m;
            obj.u_max = cfg.u_max;
            obj.lambda_a = cfg.lambda_a;
            
            % initialize member variables
            obj.T = param.dT;
            obj.ro = param.ro;
            obj.obstacles = param.obstacles;
            obj.num_N = 0;
            obj.numberAgents = param.agentCount;
            obj.U_opt = nan*zeros(obj.Hp * obj.m * obj.numberAgents,1);
            
            % set reference
            if numel(param.reference) == 0
                obj.reference = [];
            elseif numel(param.reference) == 2*obj.m
                obj.reference = param.reference;
            else
                obj.reference = param.reference(:,id);
            end
            
            % set up p_x and p_u for prediction horizon
            A = kron([1, obj.T; 0, 1], eye(obj.m));
            B = kron([0; obj.T], eye(obj.m));
            % p_x dimensions: 2*m*Hp x 2*m
            p_x = [];
            for h = 1:obj.Hp
                p_x = [p_x; A^h];
            end
            obj.p_x = p_x;
            % p_u dimensions: 2*m*Hp x m*Hp
            p_u = zeros(2 * obj.m * obj.Hp, obj.m * obj.Hp);
            for h = 1:obj.Hp
                % construct from kroneckered identity matrices that are
                % getting smaller in size
                row_index = (h - 1) * 2 * obj.m + 1;
                column_index = (obj.Hp - (h - 1)) * obj.m;
                p_u(row_index:end, 1:column_index) = p_u(row_index:end, 1:column_index)+ kron(eye(obj.Hp - (h - 1)), A^(h - 1) * B);
            end
            obj.p_u = p_u;
        end
        
        %step Calculate and appy input, send data
        function step(obj)
            obj.neighbors = zeros(obj.numberAgents,1);
            obj.U_opt = nan*zeros(obj.Hp * obj.m * obj.numberAgents,1);
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            obj.U_opt((obj.id-1)*obj.Hp*obj.m + 1:obj.id*obj.Hp * obj.m,1) = zeros(obj.Hp * obj.m,1);
            
            
            obj.num_N = length(messages);
            x_i = [obj.position; obj.velocity];
            xi = [x_i];
            Tf = obj.transformationMatrix(2*obj.m, obj.num_N);
            P_x = Tf*obj.calculatePx(obj.num_N);
            P_u = Tf*obj.calculatePu(false, false);
            
            % collision avoidance
            Hc = [];
            fc = [];
            ub_c = [];
            lb_c = [];
            A_c_u_minus = [];
            A_c_u_plus = [];
            A_c_e_plus = [];
            A_c_e_minus = [];
            b_c = [];
            if ~(obj.lambda_a~=1)
                Hc = 0;
                fc = 0;
            end
            if obj.num_N ~= 0
                for message = messages
                    x_j = [message.data.position; message.data.velocity];
                    obj.neighbors(message.data.id) = 1;
                    xi = [xi; x_j];
                end
                
                % construct H0 and H1
                h = obj.calculateH(xi, obj.num_N, obj.d);
                Jacobi = obj.constructJacobian(xi, h, obj.num_N, obj.d);
                h0 = (h-Jacobi*xi);
                h1 = Jacobi;
                H0 = kron(ones(obj.Hp,1), h0);
                H1 = kron(eye(obj.Hp), h1);
                if ~(obj.lambda_a~=1)
                    % cost function terms
                    Hc = (H1*P_u)'*H1*P_u;
                    fc = xi'*P_x'*(H1'*H1)*P_u + H0'*H1*P_u;
                else
                    A_c_u_minus = -H1*P_u;
                    A_c_u_plus = H1*P_u;
                    A_c_e_minus = -eye(obj.num_N*obj.Hp);
                    A_c_e_plus = -eye(obj.num_N*obj.Hp);
                    b_minus = H0 + H1*P_x*xi;
                    b_plus = -b_minus;
                    b_c = [b_minus; b_plus];
                    Hc = kron(diag([1 obj.lambda_a]),eye(obj.num_N*obj.Hp));
                    fc = zeros(1,2*obj.num_N*obj.Hp);
                    lb_c = zeros(2*obj.num_N*obj.Hp,1);
                    ub_c = inf*ones(2*obj.num_N*obj.Hp,1);
                end
            end
                      
            % obstacle avoidance
            Ho = [];
            fo = [];
            A_o_u = [];
            A_o_e = [];
            b_o = [];
            ub_o = [];
            lb_o = [];
            if ~isempty(obj.obstacles)
                [obj.num_O, betaAgents] = getObstaclesInRange(x_i, obj.obstacles, obj.ro);
                if obj.num_O ~= 0
                    Tf_o = obj.transformationMatrix(2*obj.m, obj.num_O);
                    P_x_o = Tf_o*obj.calculatePx(obj.num_O);
                    P_u_o = Tf_o*obj.calculatePu(true, false);
                    xio = [x_i; betaAgents];
                    
                    % construct H0 and H1 for obstacle avoidance
                    h_o = obj.calculateH(xio, obj.num_O, obj.do);
                    Jacobi_o = obj.constructJacobian(xio, h_o, obj.num_O, obj.do);
                    h0 = (h_o-Jacobi_o*xio);
                    h1 = Jacobi_o;
                    H0_o = kron(ones(obj.Hp,1), h0);
                    H1_o = kron(eye(obj.Hp), h1);
                    
                    % inequality constraints
                    A_o_u = -H1_o*P_u_o;
                    A_o_e = -eye(obj.num_O*obj.Hp);
                    b_o = H0_o + H1_o*P_x_o*xio;
                    Ho = obj.lambda_o*eye(obj.num_O*obj.Hp);
                    fo = zeros(1,obj.num_O*obj.Hp);
                    lb_o = zeros(obj.num_O*obj.Hp,1);
                    ub_o = inf*ones(obj.num_O*obj.Hp,1);
                end
            end
            
            % Gamma Agent
            Hr = 0;
            fr = 0;
            if ~isempty(obj.reference)
                xr = kron(ones(obj.num_N+1,1), obj.reference);
                error_r = reshape((xr-xi),[2*obj.m,obj.num_N+1]);
                Xr = [];
                for i = 1:(obj.num_N+1)
                    dist_r = norm(error_r(1:obj.m,i));
                    if dist_r < obj.lookAhead
                        normed = error_r(1:obj.m,i);
                    else
                        normed = obj.lookAhead*error_r(1:obj.m,i)/dist_r;
                    end
                    xi_reshaped = reshape(xi,[2*obj.m,obj.num_N+1]);
                    Xr = [Xr;kron(ones(obj.Hp,1),[normed + xi_reshaped(1:obj.m,i);obj.reference(obj.m+1:end)])];
                end
                Xr = Tf*Xr;
                Qr = kron(eye(obj.Hp*(obj.num_N+1)), diag([obj.q_pos*ones(1, obj.m) obj.q_vel*ones(1, obj.m)]));
                Hr = P_u'*Qr*P_u;
                fr = (xi'*P_x' - Xr')*Qr*P_u;
            end
            
            % input
            Hu = obj.lambda*eye(obj.m*obj.Hp*(obj.num_N+1));
            fu = zeros(1,length(Hu));
            ub_u = obj.u_max * ones(obj.m*obj.Hp*(obj.num_N+1), 1);
            lb_u = -1*ub_u;
            
            % set up optimization problem
            if ~(obj.lambda_a~=1)
                H = blkdiag(Hc + Hu + Hr, Ho);
                H = (H + H')/2;
                f = [fc + fr + fu, fo];
                A = [A_o_u,A_o_e];
                b = b_o;
            else
                H = blkdiag(Hu + Hr, Hc, Ho);
                H = (H + H')/2;               
                f = [fr + fu, fc, fo];
                A = [A_c_u_minus, A_c_e_minus, 0*A_c_e_plus, zeros(size(A_c_u_minus,1),size(A_o_e,2));...
                    A_c_u_plus, 0*A_c_e_minus, A_c_e_plus, zeros(size(A_c_u_minus,1),size(A_o_e,2))];
                if obj.num_O~=0
                    A = [A; A_o_u, zeros(size(A_o_u,1),2*size(A_c_e_plus,2)), A_o_e] ;
                end
                b = [b_c; b_o];
            end
            
            ub = [ub_u; ub_c; ub_o];
            lb = [lb_u; lb_c; lb_o];
            
            Aeq = [];
            beq = [];
            options = optimoptions(@quadprog, 'Display', 'off');
            x0 = zeros(obj.m*obj.Hp*(obj.num_N+1),1);
            
            U = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
            u = U(1:obj.m);
            
            % assign the optimal inputs to the corresponding locations in U_opt
            obj.fillUopt(U(1:obj.Hp*obj.m*(obj.num_N+1)),messages);
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            data.u = u;
            data.num_N = obj.num_N;
            data.id = obj.id;
            data.neighbors = obj.neighbors;
            data.U_opt = obj.U_opt;
            obj.send(data)
        end
    end
end