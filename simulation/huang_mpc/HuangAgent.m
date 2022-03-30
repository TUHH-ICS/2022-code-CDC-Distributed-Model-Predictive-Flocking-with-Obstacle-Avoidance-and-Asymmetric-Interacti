%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

% This class our implementation of the MPC flocking algorithm proposed by
% Huang et al. in their paper "Decentralized flocking of multi-agent 
% system based on MPC with obstacle/collision avoidance"
% (https://doi.org/10.23919/ChiCC.2019.8865184)
classdef HuangAgent < DoubleIntegratorAgent
    
    properties
        d;          % equilibrium distance(or desired distance) to neighbours
        do;         % equilibrium distance(or desired distance) to obstacles
        lambda;     % weight on control action
        c;          % velocity consensus weight
        Hp;         % prediction horizon
        m;          % state dimensions
        u_max;      % maximum input
        reference;  % reference point
        C1;         % reference position control gain
        C2;         % reference velocity control gain
        obstacles;  % obstacle matrix
        ro;         % obstacle interaction range 
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
        %PredictiveFlockingAgent Initialize agent
        % Inputs:
        %   id          : agent id
        %   dT          : sampling time
        %   initialPos  : initial position
        %   initialVel  : initial velocity
        function obj = HuangAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@DoubleIntegratorAgent(id, param.dT, initialPos, initialVel);
            
            % load data from config file
            obj.d = cfg.d;
            obj.do = cfg.do;
            obj.lambda = cfg.lambda;
            obj.c = cfg.c;
            obj.Hp = cfg.Hp;
            obj.m = cfg.m;
            obj.u_max = cfg.u_max;
            obj.C1 = cfg.C1;
            obj.C2 = cfg.C2;
            
            % initialize member variables
            obj.T = param.dT;
            obj.num_N = 0;
            obj.numberAgents = param.agentCount;
            obj.obstacles = param.obstacles;
            obj.ro = obj.do;
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
        
        function P_x = calculatePx(obj)
            % P_x dimensions: 2*m*Hp(N+1) x 2*m*Hp(N+1)
            P_x = blkdiag(kron(eye(obj.num_N+1), obj.p_x), kron(eye(obj.num_O), obj.p_x));
        end
        
        function P_u = calculatePu(obj)
            % P_u dimensions: 2*m*Hp(N+1) x m*Hp
            P_u = kron(eye(obj.num_N+1), obj.p_u);
            P_u = [P_u; zeros(2*obj.m*obj.num_O*obj.Hp, obj.m*obj.Hp*(obj.num_N+1))];
        end
        
        function e_nm = e_nm(obj,n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end
        
        function Tf = transformationMatrix(obj, dimension, size, varargin)
            if nargin == 2
                size = obj.num_N+obj.num_O+1;
            end
            Tf = [];
            for i = 1:obj.Hp
                Tf = [Tf;kron(kron(eye(size), obj.e_nm(obj.Hp,i)), eye(dimension))];
            end
        end
        
        function D = getDistanceMatrix(obj)
            D = kron([-1*ones(obj.num_N + obj.num_O,1) eye(obj.num_N + obj.num_O)], kron([1 0], eye(obj.m)));
        end
        
        % L: desired pairwise distance matrix
        function L = calculateL(obj, xi)
            Tf_x = obj.transformationMatrix(2*obj.m);
            P_x = Tf_x*obj.calculatePx(); % 2*m*Hp(N+1) x 2*m*Hp(N+1)
            qij = kron(eye(obj.Hp),obj.getDistanceMatrix())*P_x*xi;
            qij_reshaped = reshape(qij, [obj.m numel(qij)/obj.m ])';
            qij_norm = zeros(size(qij_reshaped,1), 1);
            desiredDistances = kron(ones(obj.Hp,1),[obj.d*ones(obj.num_N,1); obj.do*ones(obj.num_O,1)]);
            for i = 1:size(qij_reshaped,1)
                qij_norm(i) = norm(qij_reshaped(i,:))/desiredDistances(i);
            end
            L = diag(kron(qij_norm, ones(obj.m,1)))\qij;
        end
        
        function fillUopt(obj,U,messages)
            Tf_U = obj.transformationMatrix(obj.m, obj.num_N+1);
            U_ = reshape(Tf_U\U,[obj.m*obj.Hp, (obj.num_N+1)]);
            U_opt_ = reshape(obj.U_opt,[obj.m*obj.Hp, obj.numberAgents]);
            U_opt_(:,obj.id) = U_(:,1);
            obj.U_opt(1:obj.m*obj.Hp) = U_(:,1);
            messageCounter = 1;
            for message = messages
                neighborID = message.data.id;
                U_opt_(:,neighborID) = U_(:,messageCounter+1);
                messageCounter = messageCounter + 1;
            end
            obj.U_opt = reshape(U_opt_,[numel(U_opt_), 1]);
        end
        
        
        function step(obj)
            obj.neighbors = zeros(obj.numberAgents,1);
            obj.U_opt = nan*zeros(obj.Hp * obj.m * obj.numberAgents,1);
            
            % Receive messages from the network
            messages = obj.receive();
            
            % Get obstacles in range
            x_i = [obj.position; obj.velocity];
            [obj.num_O, betaAgents] = getObstaclesInRange(x_i, obj.obstacles, obj.ro);
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            obj.U_opt((obj.id-1)*obj.Hp*obj.m + 1:obj.id*obj.Hp * obj.m,1) = zeros(obj.Hp * obj.m,1);
            obj.num_N = length(messages);
            
            if (obj.num_N+obj.num_O) ~= 0
                xi = [x_i];
                for message = messages
                    x_j = [message.data.position; message.data.velocity];
                    obj.neighbors(message.data.id) = 1;
                    xi = [xi; x_j];
                end
                xi = [xi; betaAgents];
                
                Tf = obj.transformationMatrix(2*obj.m);
                P_x = Tf*obj.calculatePx();
                P_u = Tf*obj.calculatePu();
                
                % desired distances
                C = kron(eye(obj.Hp),obj.getDistanceMatrix());
                L = obj.calculateL(xi);
                
                % velocity missmatch
                a = (1/(obj.num_N+1)*ones(obj.num_N+1,obj.num_N+1)-eye(obj.num_N+1));
                b = kron(a, kron([1 0],eye(obj.m)));
                z = kron([eye(obj.num_N+1), zeros(obj.num_N+1,obj.num_O)], eye(2*obj.m));
                qp = b*z;
                Qp = kron(eye(obj.Hp),qp);
                
                % input penalties
                a = [-1*eye(obj.Hp-1), zeros(obj.Hp-1,1)] + [zeros(obj.Hp-1,1), eye(obj.Hp-1)];
                r_du = kron(a, eye(obj.m));
                R_du = kron(eye(obj.num_N+1), r_du);
                
                R_uk = kron(kron(eye(obj.num_N+1), obj.e_nm(obj.Hp,1)), eye(obj.m));
                
                % set up optimization problem
                H_1 = C*P_u;
                H_2 = Qp*P_u;
                H = H_1'*H_1 + obj.c*(H_2'*H_2) + obj.lambda*((R_du'*R_du) + (R_uk'*R_uk));
                f = xi'*P_x'*((C'*C) + obj.c*(Qp'*Qp))*P_u - L'*C*P_u;
                
                ub = obj.u_max * ones(obj.m*obj.Hp*(obj.num_N+1), 1);
                lb = -1*ub;
                A = [];
                b = [];
                Aeq = [];
                beq = [];
                options = optimoptions(@quadprog, 'Display', 'off');
                x0 = zeros(obj.m*obj.Hp*(obj.num_N+1),1);
                
                U = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
                u = U(1:obj.m);
                
                % assign the optimal inputs to the corresponding locations
                % in U_opt
                obj.fillUopt(U,messages);
            end
            
            % calculate naviagional input component
            u_ref = 0*u;
            if ~isempty(obj.reference)
                error = x_i - obj.reference;
                u_ref = -obj.C1*sigmaNorm(error(1:obj.m)) - obj.C2*sigmaNorm(error(obj.m+1:end));
            end   
            
            % calculate input and apply input limits
            u = u + u_ref;
            for i = 1:obj.m
                u(i) = min(obj.u_max, max(-obj.u_max, u(i)));
            end
            
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