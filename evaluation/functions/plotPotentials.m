%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function  plotPotentials()
%PLOTPOTENTIALS reprodueces the comparison figure from the paper

d = 7;              % desired distance
z = [5:0.1:8.4];

% Olfati-Saber
epsilon = 0.1;      % Used to define the sigma_norm
r       = 1.2 * d;  % Sensing radius (no interaction with agents outside ra disc)
ha      = 0.2;      % Bump function coefficient for alpha agents
za = sigma_norm(z, epsilon);
da = sigma_norm(d, epsilon);
ra = sigma_norm(r, epsilon);
pot_Olfati = psi_alpha(za, ra, da, ha);

% Huang
f = 2; % scaling factor for comparability
% Zhang
pot_zhang = f*(z-d).^2;

% Hastedt
lambda_a = f*0.2;
pot_hastedt = zeros(size(z));
for i = 1:length(z)
   if (z(i)-d)>=0
     pot_hastedt(i) = lambda_a*(z(i)-d)^2;
   else
     pot_hastedt(i) = f*(z(i)-d)^2;
   end
end

% plot
figure()
plot(z-d, pot_hastedt,'k'); hold on;
plot(z-d, pot_zhang,'Color','#D95319','LineStyle','--'); hold on;
plot(z-d,pot_Olfati-min(pot_Olfati),'b'); hold on;
xlim([-2 1.4]);
ylim([0 7]);
grid on;
plot([0 0],[0 10],'k--'); hold on;
xlabel('||q_{ij}||-d')
ylabel('interaction force')
title('Qualitative comparison of interaction forces')
text(-0.15,3,'repulsive \leftarrow','HorizontalAlignment','right');
text(0.15,3,'\rightarrow attractive');
legend('Hastedt MPC','Huang MPC', 'Olfati-Saber')
end

