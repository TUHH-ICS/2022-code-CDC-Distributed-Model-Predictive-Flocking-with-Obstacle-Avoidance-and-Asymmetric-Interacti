%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function cfg = generateConfig(algorithms, index)

% Algorithms
% 1: Hastedt MPC
% 2: Huang MPC

% common parameters
cfg.d       = 7;    % desired inter-agent distance
cfg.do      = 6;    % desired obstacle separation
cfg.m       = 2;    % space dimension
cfg.Hp      = 9;    % prediction horizon
cfg.u_max   = 1;    % input constraint

switch(index)
    % Hastedt MPC parameters
    case (1)
        cfg.lambda      = 0.1;  % control weight
        cfg.lambda_o    = 1.5;  % obstacle weight
        cfg.lambda_a    = 0.2;  % attractive force multiplier
        cfg.lookAhead   = 1;    % reference look ahead distance
        cfg.q_pos       = 0.07; % reference position weight
        cfg.q_vel       = 0.01; % reference velocity weight
        
    % Huang MPC parameters
    case (2)
        cfg.lambda  = 0.001;    % control weight
        cfg.c       = 0.2;      % velocity consensus weight
        cfg.C1      = 0.2;      % reference position gain
        cfg.C2      = 0.4;      % reference velocity gain
end
save(strcat(algorithms(index),'/cfg/config.mat'),'cfg');
end

