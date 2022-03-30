%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function P_x = calculatePx(obj, size)
%CALCULATEPX calculates state transition matrix for subsystem
% Inputs
%   size    :   number of neighbors/obstacles
% Outputs
%   P_x     :   state transition matrix for subsystem

% P_x dimensions: 2*m*Hp(N+1) x 2*m*Hp(N+1)
P_x = kron(eye(size+1), obj.p_x); 
end