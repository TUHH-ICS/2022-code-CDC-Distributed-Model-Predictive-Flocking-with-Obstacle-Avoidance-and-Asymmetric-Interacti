%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function Tf = transformationMatrix(obj, dimension, size)
%TRANSFORMATIONMATRIX calculate transformation matrix
%   Calculates a transformation matrix that transforms a vector that is
%   ordered by agent to a vector that is ordered by prediction horizon
% Input
%   dimension   :   dimension of stacked vectors
%   size        :   number of neighbors/obstacles
% Output
%   Tf          :   transformation matrix

Tf = [];
for i = 1:obj.Hp
    Tf = [Tf;kron(kron(eye(size+1), obj.e_nm(obj.Hp,i)), eye(dimension))];
end
end