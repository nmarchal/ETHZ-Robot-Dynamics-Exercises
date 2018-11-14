try % Try if simulation is still open
    x = [0 0.50 0 0.9 -1.5 0.9 -1.5 0.9 0.7 0.4 zeros(1,10)].';
    robotviz_withwall.update(x(1:10));
    delete(forcevec{1});
    delete(forcevec{2});
catch % If it doesn't work open a new visualization
    close all; clearvars; clear classes;

    load('PlanarSpotMiniModel.mat');
    load('PlanarSpotMiniWorld.mat');

    %% Set the Model Parameters

    % System parameter values
    % params = [grav, h_B, l_B, l_FS, l_FT, l_HA, l_LA, l_RS, l_RT, l_UA, m_B, m_FS, m_FT, m_HA, m_LA, m_RS, m_RT, m_UA, r_EE, r_FF, r_FT,  r_HA,  r_LA,  r_RF, r_RT,  r_UA,  w_B]
    mparams  = [9.81  0.1  0.6  0.3   0.3   0.15   0.3   0.3   0.3   0.3  10.0 0.5   0.5   0.1   0.5   0.5   0.5   0.5   0.02  0.02  0.015  0.015  0.015  0.02  0.015  0.015  0.2].';
    robotmdl.parameters.values = mparams;

    %% Create Visualizations
    fontsize = 10;
    csfscale = 0.1;
    posoffset = [0; 0; 0.5];
    zoom = 1.0;

    % Generate 3D Visualization instance
    robotviz_withwall = RobotVisualization(robotmdl, worldmdl, fontsize, csfscale, zoom, posoffset);
    robotviz_withwall.open();
    robotviz_withwall.load();

    % Add wall
    dx = 0.8;
    patch( dx*ones(4,1) , [-1 1 1 -1], [0 0 1 1], [0.9600 0.9600 0.9600])
end

%% Run Simulation
x = [0 0.50 0 0.9 -1.5 0.9 -1.5 0.9 0.7 0.4 zeros(1,10)].';

dt_ctrl = 1/30;
dt_sim = 0.001;
dt_viz = 1/10;

forcevec = cell(2,1);
vizT = tic;
totT = tic;
ctrlT = 0;
tau = floating_body_control(robotmdl, 0.0, x);
for t = 0:dt_sim:10
    simT = tic;

    % Wall Dynamics
    F_EE = wall_dynamics(robotmdl, x, dx);

    % Integrate
    qdd = constraint_consistent_fwd_dyn(robotmdl, x, tau, F_EE);
    x = x + [x(11:end); qdd]*dt_sim;

    % Update ctrl
    if t-ctrlT > dt_ctrl
        tau = hybrid_force_control(robotmdl, t, x);
        ctrlT = t;
    end

    % Visualize
    if toc(vizT) > dt_viz
        % Update robot visualization
        robotviz_withwall.update(x(1:10));
        % Visualize contact forces
        delete(forcevec{1});
        delete(forcevec{2});
        T_ICee = robotmdl.body(11).kinematics.compute.T_IBi(x(1:10), x(11:20), [], robotmdl.parameters.values);
        I_r_ICee = T_ICee(1:3,4);
        forcevec = genarrowvec(robotviz_withwall.rvaxes, [], 12, 1.0, 'magenta', '$\vec{\bf{f}}_{EE}$', I_r_ICee, F_EE(:));
        set(forcevec{1},'visible','on');
        set(forcevec{2},'visible','on');
        % Visualization timing
        vizT = tic;
        if t > toc(totT)
            pause(t - toc(totT)); % Synchronize
        end
    end
end
