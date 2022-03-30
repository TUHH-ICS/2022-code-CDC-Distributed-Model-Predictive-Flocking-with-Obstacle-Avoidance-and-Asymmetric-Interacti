%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [pos, vel] = calculateInitialStates(agentCount, dimension, field_size, setVelocity)
Agents = cell(agentCount, 1);
pos = zeros(dimension, agentCount);
vel = zeros(dimension, agentCount);
counter = 0;
i = 1;
while i <= length(Agents)
    % Randomly place the agents in the square [0,fieldsize]^2
    pos(:,i) = field_size/2 + field_size * (rand(dimension, 1) - 0.5);
    
    % Point the agents to the center of the square at the start. In this
    % way, the agents will not fragment into multiple groups. This is not
    % required if a gamma agent would be included
    
    vel(:,i) = 0.04 * ([ field_size/2; field_size/2 ] - pos(:,i));
    if ~setVelocity
        vel(:,i) = 0*vel(:,i);
    end
    
    isViolated = checkDistanceViolation(pos(:,1:i),3.5);
    if isViolated
        i = i-1;
        counter = counter + 1;
        if counter >= 50
            error('Could not initialize positions without violating minimum distance');
        end
    else
        counter = 0;
    end
    i = i+1;
end
end