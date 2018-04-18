clear;
clc;
close all force;
addpath('tools_control');
%% Vehicle initial condition
x_0     = 0;
y_0     = 0;
theta_0 = 0*pi/3;

%% Simulation data
dt   = 0.1; % time step
tf   = 50; % final time
time = 0:dt:tf;

%% Lidar data
angle_increment = 0.006135923322290;
max_radius      = 5;

%% Timing data
t_lidar = 0.4; % lidar is used every 0.4 s

%% Map creation
x_limit = 0.5*[-20; 20];
y_limit = 0.5*[-20; 20];

graphical_introduction = 0;

n_obstacles = 15;
lidar_pos   = [x_0; y_0];
obs_cell    = cell(n_obstacles,1);
if graphical_introduction == 1
    figure('Name','Obstacle map');
    hold on;
    plot_unicycle(x_0, y_0, theta_0, 'k');
    daspect([1 1 1]);
    xlim(lidar_pos(1) + [x_limit(1) x_limit(2)]);
    ylim(lidar_pos(2) + [y_limit(1) y_limit(2)]);
    box on;
    grid on;
    for i = 1:n_obstacles
        
        display(['insert point 1, obstacle ', num2str(i)]);
        point_1 = ginput(1);
        tmp     = plot(point_1(1), point_1(2), '.', 'markersize', 8);
        display(['insert point 2, obstacle ', num2str(i)]);
        point_2 = ginput(1);
        delete(tmp);
        plot([point_1(1), point_2(1)], [point_1(2), point_2(2)], 'color', 'k', 'linewidth', 2, 'linestyle', '-');
        obs_cell{i} = [point_1(1), point_2(1), point_1(2), point_2(2)];
        
    end
    pause;
    close('Obstacle map');
else
    % define obstacle as segments
    %     obs_cell{1} = [-0.2  0.2  0.4  0.4]; % [x1 x2, y1 y2]
    load('chattering.mat')
    load('percorso_strano.mat')
end
%% Define potential repulsive functions
% Define repulsive force U_alpha_B (eq.13 SFM)
syms U_0_alpha_B r_alpha_B_x r_alpha_B_y R

r_alpha_B = [r_alpha_B_x; r_alpha_B_y];
U_alpha_B = U_0_alpha_B * exp(-(norm(r_alpha_B))/R);
Grad_U    = gradient(U_alpha_B,[r_alpha_B_x, r_alpha_B_y]);
Grad_U    = matlabFunction(Grad_U);

clear U_0_alpha_B
clear r_alpha_B_x
clear r_alpha_B_y
clear R
% symObj    = syms;
%
% cellfun(@clear,symObj)
% use like this: Grad_U(R,U_0_alpha_B,r_alpha_B_x,r_alpha_B_y)
% Parameters:
R_potential           = .4;
U_0_alpha_B_potential = 30;
R_vortex              = .4;
U_0_alpha_B_vortex    = 20;
dist                  = 0.1;
walker_positions      = {[dist; 0], [-dist; 0], [0; dist], [0; -dist]}; % filter the potential arond the walker position
walker_positions      = {[0;0]};

%% Define control data
omega_max        = 45 * pi / 180; % max angula velocity for the robot
k_i_vel          = 1; % integral gain on velocity
v_0              = 0;   % initial velocity (integrator)
v_max            = 1; % max forward velocity for the robot
v_des            = 1;
k_prj_vel        = 1; % gain for the projection of the total force along the x axis of the vehicle
k_prj_omega      = 2; % gain for the projection of the total force along the y axis of the vehicle
braking_angle    = 75*pi/180;
max_deceleration = 5;
%% Run simulation

% Store data
x            = zeros(length(time), 1);
y            = zeros(length(time), 1);
theta        = zeros(length(time), 1);
v            = zeros(length(time), 1);
omega        = zeros(length(time), 1);
angle_vec    = cell(length(time), 1);
distance_vec = cell(length(time), 1);
points_cell  = cell(length(time), 1);
force_vec    = cell(length(time), 1);
% initialization
x(1)     = x_0;
y(1)     = y_0;
theta(1) = theta_0;

vortex_force_old = [0;0];

h_waitbar = waitbar(0, 'Simulating...');
for i = 1:(length(time)-1)
    
    %% Acquire lidar data
    
    [angle, distance] = lidar_acquisition([x(i); y(i)], obs_cell, max_radius, angle_increment); % the point having polar coordinates angle(i), distance(i) is written in a reference frame centered in the lidar and oriented as the ground
    
    
    % now convert [angle, distance] in the frame of the walker
    [angle_wf, distance_wf] = convert_in_walker_frame(angle, distance, theta(i));
    
    angle_vec{i}    = angle;
    distance_vec{i} = distance;
    
    %% Elaborate lidar data
    % put the obstacles measured in groups
    
    obst_group = group_obstacles(angle_wf, distance_wf, 0.15);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Filtered potential fields
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    points           = point_obst(obst_group, angle_wf, distance_wf, [0;0]); % the zero is correct
    points_cell{i}   = points; % just to store data
    repulsive_force  = [0;0];
    vortex_force     = [0;0];
    for kk = 1:length(walker_positions)
        points         = point_obst(obst_group, angle_wf, distance_wf, walker_positions{kk}); % find the closest point for each obstacle
        %% Repulsive actions using SFM
        tmp             = size(points);
        for j = 1:tmp(2) % number of column of points
            r_alpha         = [0;0];
            r_B             = [points(1,j); points(2,j)];
            r_alpha_B       = r_alpha - r_B;
            r_alpha_B_x     = r_alpha_B(1);
            r_alpha_B_y     = r_alpha_B(2);
            repulsive_force = repulsive_force - Grad_U(R_potential, U_0_alpha_B_potential,r_alpha_B_x, r_alpha_B_y);
            %         total_force = total_force + (-2 * coeff_alpha * coeff_beta)*[(points(1,j) / (points(1,j)^2 + points(2,j)^2)^(coeff_beta+1)  ); (points(2,j) / (points(1,j)^2 + points(2,j)^2)^(coeff_beta+1)  )];
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
            vortex_dot_force = dot(positive_vortex, vortex_force_old);
            if (i == 1 &&  vortex_dot_robot < 0) 
                positive_vortex = -positive_vortex; % initialize in vehicle direction
            elseif  vortex_dot_force < 0
                positive_vortex = -positive_vortex;
            end       
            vortex_force = vortex_force + positive_vortex;
        end
        
    end
    vortex_force     = vortex_force    / length(walker_positions); 
    repulsive_force  = repulsive_force / length(walker_positions);
    
    %% Add attractive force
    % Compute acceleration term
    attractive_force = [0;0];
    %% Total force
    total_force  = attractive_force + repulsive_force + vortex_force * 1;
    force_vec{i} = total_force;
    vortex_force_old = total_force;
    %% Run controller
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % choose the forward vel
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if i == 1
        v(i) = v_0; % i.e., in the initial time move
    else
        force_prj     = dot([1;0], total_force); 
        force_angle   = dot([1;0],total_force / (eps + norm(total_force)));   % cos(angle) between force and vehicle
        if force_angle >= cos(braking_angle) % then acelerate using a PI to the desired speed
            v(i) = v(i-1) + dt * k_i_vel * (v_des - v(i-1));
            v(i) = min(v(i), v_max); % this should be useless
            v(i) = max(0, v(i));
        else % then brake
            braking_action = max_deceleration *  (force_angle - cos(braking_angle));
            v(i)           = v(i-1) + dt * braking_action;
            v(i)           = max(0, v(i));
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % choose the angular vel
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    omega(i) = k_prj_omega * dot([0;1], total_force);
    omega(i) = min(omega_max, max(-omega_max, omega(i)));
    
    %% Simulate kinematics
    
    x_dot      = v(i) * cos(theta(i));
    y_dot      = v(i) * sin(theta(i));
    theta_dot  = omega(i);
    x(i+1)     = x(i)     + dt * x_dot;
    y(i+1)     = y(i)     + dt * y_dot;
    theta(i+1) = theta(i) + dt * theta_dot;
    
    waitbar(i/(length(time)-1));
end
close(h_waitbar);
%%

figure('Name','Animation');
hold on;
daspect([1 1 1]);

box on;
grid on;
for i = 1:length(time)
    %% Plot all
    clf
    box on;
    grid on;
    hold on
    xlim([x_limit(1) x_limit(2)]);
    ylim([y_limit(1) y_limit(2)]);
    
%     plot_lidar_fcn(angle_vec{i}(1:20:end), distance_vec{i}(1:20:end), max_radius, [x(i); y(i)]);
    if i ~= length(time)
        total_force = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))]*[force_vec{i}(1);force_vec{i}(2)];
        quiver(x(i),y(i),total_force(1),total_force(2), 'color', 'g', 'linewidth', 2);
    end
    plot_unicycle(x(i), y(i), theta(i), 'k');
    for j = 1:length(obs_cell)
        point_1 = [obs_cell{j}(1); obs_cell{j}(3)];
        point_2 = [obs_cell{j}(2); obs_cell{j}(4)];
        plot([point_1(1), point_2(1)], [point_1(2), point_2(2)], 'color', 'k', 'linewidth', 2, 'linestyle', '-');
    end
    points = points_cell{i};
    tmp    = size(points);
    for j = 1:tmp(2)
        
        point2plot = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))]*[points(1,j); points(2,j)] + [x(i); y(i)];
        plot(point2plot(1), point2plot(2), 'o', 'markersize', 8, 'color', 'g');
    end
    
%     pause(0.01);
    pause
end


