% proNEu: A tool for rigid multi-body mechanics in robotics.
% 
% Copyright (C) 2017  Marco Hutter, Christian Gehring, C. Dario Bellicoso,
% Vassilios Tsounis
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

%%
%   File:           spacearm_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a free-floating robotic manipulator in
%                   micro-gravity.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Load the Model Data

load('SpaceArmModel.mat');
% load('SpaceArmWorld.mat');

%% Set the Numerical Parameters

r0      = [0 0 0].';    % Base position
phi0    = [1 0 0 0].';  % Base orientation
qj0     = [0 0 0].';    % Joint configuration
v0      = [0 0 0].';    % Base linear velocity
omega0  = [0 0 0].';    % Base andular veclocity
dqj0    = [0 0 0].';    % Joint velocities

% Initial system state
xinit = [r0; phi0; qj0; v0; omega0; dqj0].';

% System parameter values
% params = [d_1x, d_1y, d_1z, d_2x, d_2y, d_2z, d_3x, d_3y, d_3z, d_Bx, d_By, d_Bz, grav, h_B, l_1, l_2, l_3, l_B, m_1, m_2, m_3, m_B, r_1,  r_2, r_3, w_B]
params   = [0.0   0.0   0.5   0.0   0.0   0.5   0.0   0.0   0.5   0.0   0.0   0.0   0.00  0.2  1.0  1.0  1.0  0.3  0.2  0.2  0.2  2.0  0.05  0.05 0.05 0.3].';

% Store the parameters
robotmdl.parameters.values = params;

%% External Force/Torque Callbacks

% Define the internal actuator forces callback function.
controller = RobotController(@spacearm_controller);

% Define the external environment forces callback function.
worldmdl = WorldModel(@spacearm_world);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'fsfb');

%% Create the Visualizations

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl);
robotviz.open();
robotviz.load();

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Run Simulation

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

% Set simulation time parameters
simconf.tstart = 0;
simconf.tstop = 10.0;
simconf.tstep = 5.0e-3;
simconf.fps = 30.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
