%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function isViolated = checkDistanceViolation(positions, minimumDistance)
    isViolated = false;
    dist = minimumDistance;
    for i = 1:size(positions,2)
       for j = size(positions,2)
           if i ~= j
               qij = norm(positions(:,i)-positions(:,j));
               dist = min(qij, dist);
           end
       end
    end
    if dist < minimumDistance
       isViolated = true; 
    end
end