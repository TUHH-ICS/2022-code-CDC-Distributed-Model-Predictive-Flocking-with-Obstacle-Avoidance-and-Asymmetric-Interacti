%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function J = constructJacobian(obj, x0, h0, size, desiredDistance)
%CONSTRUCTJACOBIAN construct Jacobian
% Inputs
%   x0                 :   evaluation point
%   h0                 :   evaluation of deviation from desired distance at x0
%   size               :   number of neighbors/obstacles
%   desiredDistance    :   desired distance to neighbors/obstacles
% Outputs
%   J                  :   Jacobian

J = [];
for i = 1:size
    s = kron([-1*ones(size,1) eye(size)], kron([1 0], eye(obj.m)));
    a = 1/(obj.e_nm(size,i)*h0 + desiredDistance)*kron([-1 obj.e_nm(size,i)], [1 0]);
    b = kron(a,eye(obj.m));
    c = s'*kron(obj.e_nm(size,i)', b);
    J = [J; x0'*c];
end
end