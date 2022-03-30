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
addpath(genpath('evaluation'),genpath('simulation'), genpath('data'))
dataPath = "data/";

%% Select Data to Evaluate
% videos are available in the "root/video" directory

% Available Data
simData = [
    "hastedt_scenario_1"    % 1
    "hastedt_scenario_2"    % 2          
    "huang_scenario_1"      % 3
    "huang_scenario_2"      % 4
    ];

% Select Data to compare by adding the indices from the simData array
dataSelection = [1,3]; % scenario 1 comparison
% dataSelection = [2,4]; % scenario 2 comparison

%% Comparison/Evaluation
% Minimum distance variable initialization
t_distN = [];
distN = [];
t_distO = [];
distO = [];

% Formation error variable initialization
t_error = [];
error = [];

% calculate performance indicators
for i=1:length(dataSelection)
    load(dataPath+simData(dataSelection(i)));
    t_distN(:,i) = out.t;
    t_distO(:,i) = out.t;
    t_vel(:,i) = out.t;
    t_error(:,i) = out.t;
    [distN(:,i),distO(:,i)] = calculateMinimumDistances(out,param,0,7,6);
    [error(:,i),~] = calculatePerformanceIndices(out,8.4,7,0);
end

%% Agent trajectories with markers for initial and final position
for j=1:length(dataSelection)
    figure()
    load(dataPath+simData(dataSelection(j)));
    viscircles(param.obstacles(1:2,:)',param.obstacles(3,:),'Color','black', 'LineWidth', 1); hold on;
    for i = 1:size(out.data.position,3)
       plot(out.data.position(:,1,i),out.data.position(:,2,i),'b'); hold on;
    end
    plot(squeeze(out.data.position(1,1,:)),squeeze(out.data.position(1,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
    plot(squeeze(out.data.position(end,1,:)),squeeze(out.data.position(end,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
    title("Agent Trajectories "+ replace(erase(simData(dataSelection(j)),".mat"),"_","\_"));
    xlabel('x');
    ylabel('y');
    xlim([-5 100]);
    ylim([-5 100]);
end


%% Minimum distances
figure()
distO=changem(distO,nan);
for i = 1:size(distN,2)
   plot(t_distN(:,i),distN(:,i), 'DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_")+" q_{ij}");
   hold on;  
   plot(t_distO(:,i),distO(:,i), '--','DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_")+" q_{io}");
   hold on; 
end
xlim([0,400])
ylim([0,8.4])
grid on;
title('Minimum inter-agent and agent-obstacle distances')
xlabel('time in s');
ylabel('distance');
legend show;
legend('Location','southeast');

%% Lattice Irregularity
figure()
for i = 1:size(distN,2)
   plot(t_error(:,i),error(:,i), 'DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_"));
   hold on;  
end
title('\alpha-lattice irregularity')
xlabel('time in s');
ylabel('irregularity');
xlim([0,400])
grid on;
legend show;

%% Potential Comparison
plotPotentials()