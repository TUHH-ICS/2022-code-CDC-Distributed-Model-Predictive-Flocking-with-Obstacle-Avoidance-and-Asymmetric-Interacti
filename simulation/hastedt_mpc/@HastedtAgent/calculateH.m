%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function h = calculateH(obj, xi, size, desiredDistance)
%CALCULATEPH calculate deviation from desired distance for given stacked state vector
% Inputs
%   xi                 :   stacked state vector of subsystem
%   size               :   number of neighbors/obstacles
%   desiredDistance    :   desired distance to neighbors/obstacles
% Outputs
%   h                  :   vector of deviations from desired distance

q = kron([-1*ones(size,1) eye(size)], kron([1 0], eye(obj.m)))*xi;
h = zeros(size,1);
for i = 1:size
    h(i) = norm(q((i-1)*obj.m + 1:i*obj.m)) - desiredDistance;
end
end