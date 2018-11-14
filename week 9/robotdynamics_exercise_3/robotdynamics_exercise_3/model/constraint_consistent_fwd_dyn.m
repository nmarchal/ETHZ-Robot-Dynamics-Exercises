function [qdd, F] = constraint_consistent_fwd_dyn(model, x, tau, F_EE)


persistent I_p_Ff0 I_p_Hf0;

q = x(1:10); % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
qd = x(11:20); % [10X1] Generalized velocities

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

I_T_Ff = model.body(7).kinematics.compute.T_IBi(q, qd, [], params);
I_p_Ff = I_T_Ff([1,3], 4);
I_T_Hf = model.body(4).kinematics.compute.T_IBi(q, qd, [], params);
I_p_Hf = I_T_Hf([1,3], 4);

if isempty(I_p_Ff0)
    I_p_Ff0 = I_p_Ff;
    I_p_Hf0 = I_p_Hf;
end

% Assemble foot constraints -> no linear velocity
I_J_c = [I_J_Ff(1:2,:) ; I_J_Hf(1:2,:)];
I_Jd_c = [I_Jd_Ff(1:2,:); I_Jd_Hf(1:2,:)];
num_constr = size(I_J_c,1);

% Selection Matrix
S = [zeros(7, 3), eye(7)]; 

% Drift correction
kp = 10;
kd = 2*sqrt(kp);
wd_drift = [kp*(I_p_Ff0 - I_p_Ff); kp*(I_p_Hf0 - I_p_Hf)] - kd*I_J_c*qd;

% Solve for acceleration and constraint simultaniously
A = [M    ,  -I_J_c'; ...
     -I_J_c, zeros(num_constr)];
c = [S'*tau + I_J_EE'*F_EE - b - g; ...
     I_Jd_c * qd - wd_drift];

x_dyn = A \ c; 
qdd = x_dyn(1:10);
F = x_dyn(11:14);

end

function jac = eval_jac(f, q, dq, params)

jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end

