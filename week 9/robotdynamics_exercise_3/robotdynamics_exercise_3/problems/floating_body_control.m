function [ tau ] = floating_body_control(model, t, state)
% Implement a controller that controls the floating body of the legged system
% Outputs:
% 	tau : [7x1] torques [tau_F, tau_H, tau_A]'
%		tau_F : [2x1] torques [hip, knee]' (front leg)
%		tau_H : [2X1] torques [hip, knee]' (hind leg)
%		tau_A : [3x1] torques [shoulder, elbow, wrist]' (arm)
% 
% Inputs:
%   model : object containing kinematic and dynamic quantities
%   t : Current time [s]
%   state : Current state of the system (position, velocity)

q = state(1:10); % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
qd = state(11:20); % [10X1] Generalized velocities
u = qd ;

% Extract dynamics at current state
params = model.parameters.values;
M = model.dynamics.compute.M(q,qd,[],[],params); % [10X10] Inertia matrix
b = model.dynamics.compute.b(q,qd,[],[],params); % [10X1] Nonlinear-dynamics vector
g = model.dynamics.compute.g(q,qd,[],[],params); % [10X1] Gravity vector

% Get Jacobians and Derivatives at current state
I_J_b  = eval_jac(model.body( 1).kinematics.compute.I_J_IBi, q, [], params);    % [3X10] body position and orientation Jacobian
I_J_Ff = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X10] Front foot position and orientation Jacobian
I_J_Hf = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Hind foot position and orientation Jacobian
I_J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian

I_Jd_b  = eval_jac(model.body( 1).kinematics.compute.I_dJ_IBi, q, qd, params);  % [3X10] body position and orientation Jacobian derivative
I_Jd_Ff = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, qd, params);  % [3X10] Front foot position and orientation Jacobian derivative
I_Jd_Hf = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, qd, params);  % [3x10] Hind foot position and orientation Jacobian derivative
I_Jd_EE = eval_jac(model.body(11).kinematics.compute.I_dJ_IBi, q, qd, params);  % [3x10] Arm End-effector position and orientation Jacobian derivative

% Assemble constraint Jacobian -> Only constrain linear velocity at feet
I_J_c = [I_J_Ff(1:2,:) ; I_J_Hf(1:2,:)];
I_Jd_c = [I_Jd_Ff(1:2,:); I_Jd_Hf(1:2,:)];

S = [zeros(7, 3), eye(7)]; % selection matrix

%% Reference:
% Given for this task: Desired body position and velocity
%   q_b_des  : [3X1] Desired body motion
%	qd_b_des : [3x1] Desired body velocities
amplitude = [0.05, 0.05, 0.05]';
frequency = [0.2, 0.4, 0.3]'*2*pi;
offset = [0, 0.5, 0]';
q_b_des = amplitude.*sin(frequency*t) + offset;
qd_b_des = amplitude.*frequency.*cos(frequency*t);

%% Optimizations
% x = [qdd', f_c', tau']

% Equations of motions
A_eom = [M -I_J_c' -S'];
b_eom = -b-g ;

% No foot contact motions
A_c = [I_J_c zeros(4,4) zeros(4, 7)];
b_c = [- I_Jd_c*qd];

% Body motion
kp = 10;
kd = 2*sqrt(kp);
wd_b = kp*(q_b_des - q(1:3)) + kd*(qd_b_des - qd(1:3));
A_b = [I_J_b, zeros(3, 4), zeros(3, 7)];
b_b =  wd_b - I_Jd_b*qd;

% ===== Additional objectives and constraints ============
% Kinematic null space position control
q0 = [0 0.50 0 0.9 -1.5 0.9 -1.5 0.9 0.7 0.4]'; % [10x1] Default generalized coordinates
kp_0 = 1;
kd_0 = 2*sqrt(kp_0);
A_0 = [eye(10), zeros(10, 4), zeros(10, 7)];
b_0 = kp_0*(q0 - q) - kd_0*qd;

% Torque limits
tau_max = 50;
C_tau_up = [zeros(7, 10), zeros(7, 4), eye(7)];
d_tau_up = tau_max*ones(7,1);
C_tau_low = -C_tau_up;
d_tau_low = d_tau_up;

% Friction Cone
mu = 0.5;
C_cone = [0 -1; 1 -mu; -1 -mu];
C_f_c = [zeros(6, 10), blkdiag(C_cone, C_cone), zeros(6, 7)];
d_f_c = zeros(6, 1);

% Torque minimization
A_tau = [zeros(7, 10), zeros(7, 4), eye(7)];
b_tau = zeros(7,1);

% Contact force minimization
A_f_c = [zeros(4, 10), eye(4), zeros(4, 7)];
b_f_c = zeros(4,1);

%% Assemble and solve
% Define hierarchy of tasks
A = {A_eom, A_c, A_b, A_0, A_tau, A_f_c};
b = {b_eom, b_c, b_b, b_0, b_tau, b_f_c};

% Define inequality constraints
C = [C_tau_up; C_tau_low; C_f_c];
d = [d_tau_up; d_tau_low; d_f_c];

% Solve hierarchical QPs
printSolution = 0;
x_opt = sequentialQuadraticOptimization(A, b, C, d, printSolution);
qdd = x_opt(1:10);
f_c = x_opt(11:14);
tau = x_opt(15:end);

end

function jac = eval_jac(f, q, dq, params)

jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end
