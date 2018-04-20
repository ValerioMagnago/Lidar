function [v, omega] = lidar2force_field(lidarScan, state, v_measured, omega_measured, dt, params)

%% Define here parameters
point_closeness = 0.15; % distance between two point belonging to the same obstacle
walker_positions = {[0;0]}; % positions around which the fields is computed. Use more than one position to filter
% Grad_U = @(R, U_0_alpha_B, r_alpha_B_x, r_alpha_B_y)[-(U_0_alpha_B.*exp(-sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2)./R).*abs(r_alpha_B_x).*sign(r_alpha_B_x).*1.0./sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2))./R;-(U_0_alpha_B.*exp(-sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2)./R).*abs(r_alpha_B_y).*sign(r_alpha_B_y).*1.0./sqrt(abs(r_alpha_B_x).^2+abs(r_alpha_B_y).^2))./R];

R_potential = params.R_potential;
U_0_alpha_B_potential = params.U_0_alpha_B_potential;
R_vortex = params.R_vortex;
U_0_alpha_B_vortex = params.U_0_alpha_B_vortex;
max_deceleration = params.max_deceleration;
braking_angle = params.braking_angle;
k_prj_omega = params.k_prj_omega;
omega_max = params.omega_max;
v_des = params.v_des;
v_max = params.v_max;
k_i_vel = params.k_i_vel;

%% Generate angles from lidar scan
angles = linspace(lidarScan.AngleMin,lidarScan.AngleMax,numel(lidarScan.Ranges));


%% Elaborate lidar data
% put the obstacles measured in groups
obst_group = group_obstacles(angles, lidarScan.Ranges, point_closeness);


%% Filtered Force fields
% initialize vortex and
repulsive_force = [0;0];
vortex_force = [0;0];

for kk = 1:length(walker_positions)
points = point_obst(obst_group, angles, lidarScan.Ranges, walker_positions{kk}); % find the closest point for each obstacle
%% Repulsive actions
tmp = size(points);
for j = 1:tmp(2) % number of column of points
r_alpha = [0;0];
r_B = [points(1,j); points(2,j)];
r_alpha_B = r_alpha - r_B;
r_alpha_B_x = r_alpha_B(1);
r_alpha_B_y = r_alpha_B(2);
repulsive_force = repulsive_force - Grad_U(R_potential, U_0_alpha_B_potential,r_alpha_B_x, r_alpha_B_y);
end
%% Vortex force
for j = 1:tmp(2) % number of column of points
r_alpha = [0;0];
r_B = [points(1,j); points(2,j)];
r_alpha_B = r_alpha - r_B;
r_alpha_B_x = r_alpha_B(1);
r_alpha_B_y = r_alpha_B(2);
tmp2 = Grad_U(R_vortex, U_0_alpha_B_vortex, r_alpha_B_x, r_alpha_B_y);
grad_x = tmp2(1);
grad_y = tmp2(2);
positive_vortex = [grad_y; -grad_x];


vortex_dot_robot = dot([1;0], positive_vortex);
if vortex_dot_robot < 0
positive_vortex = -positive_vortex;
end
vortex_force = vortex_force + positive_vortex;
end

end
vortex_force = vortex_force / length(walker_positions);
repulsive_force = repulsive_force / length(walker_positions);

%% Total force
total_force = repulsive_force + vortex_force * 1;


%% Run controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choose the forward vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

force_angle = dot([1;0], total_force / (eps + norm(total_force))); % cos(angle) between force and vehicle
if force_angle >= cos(braking_angle) % then acelerate using a PI to the desired speed
v = v_measured + dt * k_i_vel * (v_des - v_measured);
v = min(v, v_max); % this should be useless
v = max(0, v);
else % then brake
braking_action = max_deceleration * (force_angle - cos(braking_angle));
v = v_measured + dt * braking_action;
v = max(0, v);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choose the angular vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omega = k_prj_omega * dot([0;1], total_force);
omega = min(omega_max, max(-omega_max, omega));
