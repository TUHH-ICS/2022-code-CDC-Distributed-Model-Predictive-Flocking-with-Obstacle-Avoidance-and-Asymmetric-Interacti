%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear all;
close all;

%% Select Algorithm and Scenario
% Algorithms
% 1: Hastedt MPC
% 2: Huang MPC

% Scenarios
% 1: Large obstacle
% 2: Field of small Obstacles

algorithmIndex  = 1;
scenarioIndex   = 1;

%% Setup
addpath(genpath('simulation/mas-simulation/lib'))
addpath(genpath('simulation'))
simPath = "simulation/";

algorithms = simPath + [...
    "hastedt_mpc"
    "huang_mpc"
    ];

preallocate = {...
    @preallocateHastedt
    @preallocateHuang
    };

generateSetup = {...
    @generateSetupHastedt
    @generateSetupHuang
    };
%% Set Agent Parameters
cfg = generateConfig(algorithms, algorithmIndex);

%% Simulation Setup
% define output and initialization files and paths
outPath = strcat(simPath,"out/",erase(algorithms(algorithmIndex),simPath),"/");
outFile = outPath+"results.mat";
initializationFile = simPath+"initialStates_20.mat";

% simulation parameters
Tf               = 400;  % Simulation duration [s]
param.agentCount = 20;   % Number of agents in the network
param.dimension  = 2;    % Dimension of the space the agents move in
param.dT         = 0.3;  % Size of the simulation time steps [s]
param.range      = 8.4;  % Agent interaction range
param.ro         = 8.4;  % Obstacle interaction range

% scenario
param.reference = [80;80;0;0];
if (scenarioIndex == 1)
    param.obstacles = [50;50;5];
elseif (scenarioIndex == 2)
    param.obstacles = [ 50  67  50  50  33  33  67
                        50  50  67  33  50  67  33
                        1   1   1   1   1   1   1];
end

init = load(initializationFile);
setup = generateSetup{algorithmIndex}(cfg, param, 1, init.pos, init.vel);

%% Run Simulation
sim = SimulationManager(setup.Network, setup.Agents);
leech = preallocate{algorithmIndex}(setup, sim, Tf);
data = performSimulation(sim, leech, Tf);
out.t = data.t;
out.data = data.data;
save(outFile,'out','setup','param', 'cfg');