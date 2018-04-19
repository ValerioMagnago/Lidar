function [v, omega] = lidar2force_field(lidarScan, state, v_measured, omega_measured, dt)

%% Define here parameters
point_closeness                      = 0.15; % distance between two point belonging to the same obstacle
walker_positions                     = {[0;0]}; % positions around which the fields is computed. Use more than one position to filter
% Grad_U                               = @(R, U_0_alpha_B, r_alpha_B_x, r_alpha_B_y)[-(U_0_alpha_B.*exp(-sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2)./R).*abs(r_alpha_B_x).*sign(r_alpha_B_x).*1.0./sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2))./R;-(U_0_alpha_B.*exp(-sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2)./R).*abs(r_alpha_B_y).*sign(r_alpha_B_y).*1.0./sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2))./R];
[R_potential, U_0_alpha_B_potential] = tune_potential(0.5, 1, 10, 0.01);
[R_vortex, U_0_alpha_B_vortex]       = tune_potential(0.5, 3, 3, 1);
max_deceleration                     = 10;
braking_angle                        = 75 * pi/180;
k_prj_omega                          = 2;
omega_max                            = 45 * pi /180;
v_des                                = 1;
v_max                                = 1;
k_i_vel                              = 1;
%% Generate angles from lidar scan
angles = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));


%% Elaborate lidar data
% put the obstacles measured in groups
obst_group = group_obstacles(angles, lidarScan.ranges, point_closeness);


%% Filtered Force fields
% initialize vortex and
repulsive_force  = [0;0];
vortex_force     = [0;0];

for kk = 1:length(walker_positions)
    points         = point_obst(obst_group, angles, lidarScan.ranges, walker_positions{kk}); % find the closest point for each obstacle
    %% Repulsive actions
    tmp             = size(points);
    for j = 1:tmp(2) % number of column of points
        r_alpha         = [0;0];
        r_B             = [points(1,j); points(2,j)];
        r_alpha_B       = r_alpha - r_B;
        r_alpha_B_x     = r_alpha_B(1);
        r_alpha_B_y     = r_alpha_B(2);
        repulsive_force = repulsive_force - Grad_U(R_potential, U_0_alpha_B_potential,r_alpha_B_x, r_alpha_B_y);
    end
    %% Vortex force
    for j = 1:tmp(2) % number of column of points
        r_alpha         = [0;0];
        r_B             = [points(1,j); points(2,j)];
        r_alpha_B       = r_alpha - r_B;
        r_alpha_B_x     = r_alpha_B(1);
        r_alpha_B_y     = r_alpha_B(2);
        tmp             = Grad_U(R_vortex, U_0_alpha_B_vortex, r_alpha_B_x, r_alpha_B_y);
        grad_x          = tmp(1);
        grad_y          = tmp(2);
        positive_vortex = [grad_y; -grad_x];
        
        
        vortex_dot_robot = dot([1;0], positive_vortex);
        if vortex_dot_robot < 0
            positive_vortex = -positive_vortex;
        end
        vortex_force = vortex_force + positive_vortex;
    end
    
end
vortex_force     = vortex_force    / length(walker_positions);
repulsive_force  = repulsive_force / length(walker_positions);

%% Total force
total_force  = repulsive_force + vortex_force * 1;


%% Run controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choose the forward vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

force_angle   = dot([1;0], total_force / (eps + norm(total_force)));   % cos(angle) between force and vehicle
if force_angle >= cos(braking_angle) % then acelerate using a PI to the desired speed
    v = v_measured + dt * k_i_vel * (v_des - v_measured);
    v = min(v, v_max); % this should be useless
    v = max(0, v);
else % then brake
    braking_action = max_deceleration *  (force_angle - cos(braking_angle));
    v              = v_measured + dt * braking_action;
    v              = max(0, v);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choose the angular vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omega = k_prj_omega * dot([0;1], total_force);
omega = min(omega_max, max(-omega_max, omega));
