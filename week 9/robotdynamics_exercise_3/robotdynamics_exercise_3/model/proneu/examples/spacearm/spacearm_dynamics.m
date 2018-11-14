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
%   File:           spacearm_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           28/9/2016
%
%   Desciption:     Analytical derivation of the EoM for a robotic
%                   manipulator in micro-gravity.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% World Gravity

% Gravitational potential field in inertial world-frame
syms grav real;
e_I_g = [0 0 1].';
I_a_g = grav*e_I_g;

%% System Parameters & Variables

% Declare all system parametes
syms m_B m_1 m_2 m_3 real;

syms d_Bx d_By d_Bz real;
syms l_B w_B h_B real;

syms d_1x d_1y d_1z real;
syms r_1 l_1 real;

syms d_2x d_2y d_2z real;
syms r_2 l_2 real;

syms d_3x d_3y d_3z real;
syms r_3 l_3 real;

% Declare all system variables
syms phi_1 phi_2 phi_3 real;
syms T_1 T_2 T_3 real;
syms F_EEx F_EEy F_EEz T_EEx T_EEy T_EEz real;

%% Multi-Body System Description

% Satelite Base
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'B';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [];
body(i).cs.C_PB             = [];
body(i).param.m             = m_B;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [d_Bx d_By d_Bz].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_B w_B h_B] ; 
body(i).geometry.values     = [0.3 0.3 0.2];
body(i).geometry.offsets.r  = [0 0 -0.1].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% Arm - first link
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 w_B/2 0].';
body(i).cs.C_PB             = getRotationMatrixX(phi_1);
body(i).param.m             = m_1;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [d_1x d_1y d_1z]; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_1 l_1]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Arm - second link
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_2);
body(i).param.m             = m_2;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [d_2x d_2y d_2z]; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_2 l_2]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Arm - third link
i=4;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_3);
body(i).param.m             = m_3;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [d_3x d_3y d_3z]; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_3 l_3]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% % Arm - end-effector
i=5;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'ee';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_3].';
body(i).cs.C_PB             = sym(eye(3));

%% Definition of External Forces & Torques

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T1';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 2;
ftel(j).B_T     = [T_1 0 0].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T2';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).B_T     = [0 0 T_2].';

j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T3';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 3; 
ftel(j).body_B  = 4;
ftel(j).B_T     = [0 0 T_3].';

j=4;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_EE';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 5;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 l_3].');
ftel(j).I_F     = [F_EEx F_EEy F_EEz].';
ftel(j).I_T     = [T_EEx T_EEy T_EEz].';

%% System Definition

% Definition of the joint DoFs of 2-link system
q_j  = [phi_1 phi_2 phi_3].';

% Controllable joint forces/torques
tau_j = [T_1 T_2 T_3].';

% External forces/torques
tau_env = [F_EEx F_EEy F_EEz T_EEx T_EEy T_EEz].';

%% Generate Full System Model using proNEu.v2

robot_name = 'SpaceArmModel';

% Generate the model object
robotmdl = RobotModel(body, ftel, q_j, tau_j, tau_env, I_a_g, 'name', robot_name, 'type', 'floating', 'orientation', 'quaternion', 'method', 'proneu', 'symsteps', 100);

%% Generate Numerical Computations

robotmdl.generatefunctions();

%% Save to MAT File

% Generate file and directory paths
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

% Store generated model in the appropriate directory
save(strcat(dpath,robotmdl.name), 'robotmdl');

%% 
% EOF
