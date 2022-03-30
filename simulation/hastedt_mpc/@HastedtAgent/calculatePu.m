%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function P_u = calculatePu(obj, isObstacles, isZero)
%CALCULATEPU calculates input matrix for subsystem
% Inputs
%   isObstacles    :   if true, consider obstacle avoidance
%   isZero         :   if true, neighbor inputs are assumed to be zero
% Outputs
%   P_u            :   input matrix for subsystem

if ~isObstacles
    if isZero
        % P_u dimensions: 2*m*Hp*(N+1) x m*Hp
        P_u = kron([1; zeros(obj.num_N,1)], obj.p_u);
    else
        % P_u dimensions: 2*m*Hp*(N+1) x m*Hp*(N+1)
        P_u = kron(eye(obj.num_N+1), obj.p_u);
    end
else
    if isZero
        % P_u dimensions: 2*m*Hp*(O+1) x m*Hp
        P_u = kron([1; zeros(obj.num_N,1)], obj.p_u);
    else
        % U_i dimensions: m*Hp x m*Hp*(N+1)
        U_i = kron([1, zeros(1,obj.num_N)], eye(obj.m*obj.Hp));
        % P_u dimensions: 2*m*Hp*(O+1) x m*Hp
        P_u = kron([1; zeros(obj.num_O,1)], obj.p_u);
        P_u = P_u*U_i;
    end
end
end